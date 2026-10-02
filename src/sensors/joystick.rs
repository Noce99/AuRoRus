//! [`Joystick`]: a human driving the vehicle with a gamepad, as `web_gui`'s
//! WASD control does, read through the Linux joystick API
//! (`/dev/input/js*`).

use crate::topics::{ActuatorLimits, JOYSTICK_VESC_COMMAND_TOPIC_NAME, VehicleTopics, VescCommand};
use crate::{Captain, Executor, Ticker};
use std::any::Any;
use std::fs::{File, OpenOptions};
use std::io::{ErrorKind, Read};
use std::os::fd::AsRawFd;
use std::os::unix::fs::OpenOptionsExt;
use std::path::PathBuf;
use std::time::{Duration, Instant};

/// Size of one `struct js_event`: a `u32` timestamp, an `i16` value, a `u8`
/// type and a `u8` axis or button number.
const EVENT_SIZE: usize = 8;
/// `js_event` type: a button changed.
const JS_EVENT_BUTTON: u8 = 0x01;
/// `js_event` type: an axis changed.
const JS_EVENT_AXIS: u8 = 0x02;
/// `js_event` type flag: the pad's state on opening, not a change.
const JS_EVENT_INIT: u8 = 0x80;
/// An axis's full travel either way.
const AXIS_MAX: f64 = 32767.0;
/// `JSIOCGAXES`, `_IOR('j', 0x11, __u8)`: how many axes the device has.
const JSIOCGAXES: libc::c_ulong = 0x8001_6a11;
/// `JSIOCGBUTTONS`, `_IOR('j', 0x12, __u8)`: how many buttons it has.
const JSIOCGBUTTONS: libc::c_ulong = 0x8001_6a12;
/// `JSIOCGNAME(len)` without its length, `_IOC(_IOC_READ, 'j', 0x13, 0)`:
/// the device's name.
const JSIOCGNAME: libc::c_ulong = 0x8000_6a13;
/// Room for the device's name.
const NAME_LEN: usize = 128;

/// Every tunable parameter [`Joystick`] needs - loaded from
/// `config/sensors/joystick.toml` (see [`Default`]) or from an arbitrary path
/// via [`crate::config::load`].
#[derive(Debug, Clone, PartialEq, serde::Deserialize)]
pub struct JoystickConfig {
    /// The pad's Linux joystick device, e.g. `/dev/input/js0`.
    pub device: PathBuf,
    /// How often the pad's state is published, in Hz.
    pub rate_hz: f64,
    /// How long to wait before reopening the pad after losing it, in seconds.
    pub reconnect_delay_s: f64,
    /// Speed a trigger pulled all the way commands, in m/s - capped at the
    /// vehicle's [`ActuatorLimits::max_speed_mps`].
    pub max_speed_mps: f64,
    /// Exponent the triggers' travel is raised to before scaling to speed.
    pub speed_exponent: f64,
    /// Axis steering with, right positive.
    pub steering_axis: u8,
    /// Trigger axis driving forward.
    pub throttle_axis: u8,
    /// Trigger axis driving in reverse.
    pub reverse_axis: u8,
    /// Whether [`Self::steering_axis`] reads left positive instead.
    pub invert_steering: bool,
    /// Stick travel ignored around the center, as a fraction of full travel.
    pub stick_deadzone: f64,
    /// Trigger travel ignored at rest, as a fraction of full travel.
    pub trigger_deadzone: f64,
    /// A button that must be held for the pad to drive at all, if any.
    pub deadman_button: Option<u8>,
}

impl Default for JoystickConfig {
    fn default() -> Self {
        toml::from_str(include_str!("../../config/sensors/joystick.toml"))
            .expect("config/sensors/joystick.toml must deserialize into JoystickConfig")
    }
}

/// A human driving with a gamepad: every [`JoystickConfig::rate_hz`],
/// publishes the pad's steering stick and triggers on
/// [`JOYSTICK_VESC_COMMAND_TOPIC_NAME`] - full steering is the vehicle's
/// steering lock and a full trigger [`JoystickConfig::max_speed_mps`], both
/// capped by the vehicle's [`ActuatorLimits`]. Like WASD, it keeps publishing
/// a stationary, centered command while nothing is held, which hands control
/// back (see [`crate::simulation::SimulatedVehicle`]).
///
/// Starts whether or not the pad is there: losing it (or not finding it) is
/// logged, publishes that stationary command - so its last setpoint never
/// outlives it - and retries every [`JoystickConfig::reconnect_delay_s`].
pub struct Joystick {
    id: u16,
    name: String,
    config: JoystickConfig,
}

impl Joystick {
    pub fn new(name: impl Into<String>, config: JoystickConfig) -> Self {
        Self {
            id: 0,
            name: name.into(),
            config,
        }
    }

    /// Opens the pad without blocking, so [`Self::run`] can drain its events
    /// once per tick and still notice being stopped - refusing a device that
    /// lacks an axis or button [`JoystickConfig`] drives with, since the
    /// kernel hands out `/dev/input/js*` to more than gamepads (e.g. a
    /// touchscreen, whose position would read as a stick held over).
    fn open(&self) -> std::io::Result<(File, String)> {
        let file = OpenOptions::new()
            .read(true)
            .custom_flags(libc::O_NONBLOCK)
            .open(&self.config.device)?;
        let layout = PadLayout::of(&file)?;
        layout.check(&self.config).map_err(|missing| {
            std::io::Error::other(format!(
                "{:?} is not a gamepad this config drives: it has no {missing}",
                layout.name
            ))
        })?;
        Ok((file, layout.name))
    }
}

impl Executor for Joystick {
    fn init(&mut self, id: u16) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_writer::<VescCommand>(
            JOYSTICK_VESC_COMMAND_TOPIC_NAME,
            self.id,
            VescCommand::default,
        );
    }

    fn run(&mut self, captain: &Captain) {
        let topic = captain.topic::<VescCommand>(JOYSTICK_VESC_COMMAND_TOPIC_NAME);
        let limits_topic =
            captain.try_topic::<ActuatorLimits>(&VehicleTopics::ego().vehicle_limits());
        let reconnect_delay = Duration::from_secs_f64(self.config.reconnect_delay_s);
        let mut ticker = Ticker::new(self.config.rate_hz);
        let mut device: Option<File> = None;
        let mut next_attempt = Instant::now();
        // Whether the last failure to open was already reported, so a missing
        // pad is logged once rather than every retry.
        let mut reported_missing = false;
        let mut pad = PadState::default();

        while captain.is_running(self.id) {
            if device.is_none() && Instant::now() >= next_attempt {
                match self.open() {
                    Ok((file, pad_name)) => {
                        println!(
                            "{}: driving with {} ({pad_name})",
                            self.name,
                            self.config.device.display()
                        );
                        device = Some(file);
                        reported_missing = false;
                    }
                    Err(error) => {
                        if !reported_missing {
                            eprintln!(
                                "{}: no joystick at {} ({error}), retrying every {:.1} s",
                                self.name,
                                self.config.device.display(),
                                self.config.reconnect_delay_s
                            );
                            reported_missing = true;
                        }
                        next_attempt = Instant::now() + reconnect_delay;
                    }
                }
            }
            if let Some(file) = &mut device
                && let Err(error) = pad.drain(file)
            {
                eprintln!("{}: lost the joystick ({error})", self.name);
                device = None;
                pad = PadState::default();
                next_attempt = Instant::now() + reconnect_delay;
            }

            let command = match (&device, &limits_topic) {
                (Some(_), Some(limits)) => pad.command(&self.config, &limits.read()),
                _ => VescCommand::default(),
            };
            topic
                .write(self.id, command)
                .expect("lost writer authorization for the joystick_vesc_command topic");

            ticker.wait();
        }
    }

    fn name(&self) -> String {
        self.name.clone()
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn fresh(&self) -> Box<dyn Executor> {
        Box::new(Joystick::new(self.name.clone(), self.config.clone()))
    }
}

/// What a joystick device has, as the kernel reports it on opening.
#[derive(Debug, Clone, PartialEq)]
struct PadLayout {
    name: String,
    axes: u8,
    buttons: u8,
}

impl PadLayout {
    /// Asks the kernel about the joystick device `file` is open on.
    fn of(file: &File) -> std::io::Result<Self> {
        let fd = file.as_raw_fd();
        let mut axes: u8 = 0;
        let mut buttons: u8 = 0;
        let mut name = [0u8; NAME_LEN];
        // SAFETY: `fd` is an open descriptor for the whole call, and each
        // pointer is to a live buffer of the size its request writes.
        unsafe {
            if libc::ioctl(fd, JSIOCGAXES, &mut axes as *mut u8) < 0
                || libc::ioctl(fd, JSIOCGBUTTONS, &mut buttons as *mut u8) < 0
                || libc::ioctl(
                    fd,
                    JSIOCGNAME | ((NAME_LEN as libc::c_ulong) << 16),
                    name.as_mut_ptr(),
                ) < 0
            {
                return Err(std::io::Error::last_os_error());
            }
        }
        let end = name.iter().position(|&byte| byte == 0).unwrap_or(NAME_LEN);
        Ok(Self {
            name: String::from_utf8_lossy(&name[..end]).into_owned(),
            axes,
            buttons,
        })
    }

    /// Whether every axis and button `config` drives with exists - erring
    /// with the first one missing.
    fn check(&self, config: &JoystickConfig) -> Result<(), String> {
        for (role, axis) in [
            ("steering", config.steering_axis),
            ("throttle", config.throttle_axis),
            ("reverse", config.reverse_axis),
        ] {
            if axis >= self.axes {
                return Err(format!("{role} axis {axis} (it has {} axes)", self.axes));
            }
        }
        match config.deadman_button {
            Some(button) if button >= self.buttons => Err(format!(
                "dead man's button {button} (it has {} buttons)",
                self.buttons
            )),
            _ => Ok(()),
        }
    }
}

/// The pad's axes and buttons as last reported - the kernel sends every one's
/// state on opening, then only changes.
#[derive(Debug, Clone, PartialEq)]
struct PadState {
    /// Each axis's position, -1 to 1 - `None` until reported, which reads as
    /// at rest: centered for a stick, released for a trigger (whose rest is
    /// -1, so a centered one would read half pulled).
    axes: [Option<f64>; 256],
    /// Each button's state.
    buttons: [bool; 256],
}

impl Default for PadState {
    fn default() -> Self {
        Self {
            axes: [None; 256],
            buttons: [false; 256],
        }
    }
}

impl PadState {
    /// Applies every event waiting on `file`, erring only when the pad is
    /// gone (or its device misbehaves).
    fn drain(&mut self, file: &mut File) -> std::io::Result<()> {
        let mut buffer = [0u8; EVENT_SIZE * 64];
        loop {
            let read = match file.read(&mut buffer) {
                Ok(0) => return Err(ErrorKind::UnexpectedEof.into()),
                Ok(read) => read,
                Err(error) if error.kind() == ErrorKind::WouldBlock => return Ok(()),
                Err(error) if error.kind() == ErrorKind::Interrupted => continue,
                Err(error) => return Err(error),
            };
            for event in buffer[..read].as_chunks::<EVENT_SIZE>().0 {
                self.apply(event);
            }
        }
    }

    /// Applies one raw `js_event`.
    fn apply(&mut self, event: &[u8; EVENT_SIZE]) {
        let value = i16::from_le_bytes([event[4], event[5]]);
        let number = event[7] as usize;
        match event[6] & !JS_EVENT_INIT {
            JS_EVENT_AXIS => {
                self.axes[number] = Some((f64::from(value) / AXIS_MAX).clamp(-1.0, 1.0));
            }
            JS_EVENT_BUTTON => self.buttons[number] = value != 0,
            _ => {}
        }
    }

    /// The command the pad asks for under `config`, within `limits`.
    fn command(&self, config: &JoystickConfig, limits: &ActuatorLimits) -> VescCommand {
        if config
            .deadman_button
            .is_some_and(|button| !self.buttons[button as usize])
        {
            return VescCommand::default();
        }
        let stick_at = |axis: u8| self.axes[axis as usize].unwrap_or(0.0);
        let trigger_at = |axis: u8| self.axes[axis as usize].unwrap_or(-1.0);
        let mut steering = stick(stick_at(config.steering_axis), config.stick_deadzone);
        if config.invert_steering {
            steering = -steering;
        }
        let travel = trigger(trigger_at(config.throttle_axis), config.trigger_deadzone)
            - trigger(trigger_at(config.reverse_axis), config.trigger_deadzone);
        let speed = travel.signum() * travel.abs().powf(config.speed_exponent);
        VescCommand::new(
            steering * limits.max_steering_angle_rad,
            speed * config.max_speed_mps.min(limits.max_speed_mps),
        )
    }
}

/// A stick's `position` (-1 to 1) with `deadzone` around the center cut out
/// and the rest stretched back to -1 to 1, so it starts from 0 past it.
fn stick(position: f64, deadzone: f64) -> f64 {
    let past = (position.abs() - deadzone).max(0.0) / (1.0 - deadzone);
    position.signum() * past.min(1.0)
}

/// A trigger's travel, 0 to 1, from its `position` (-1 at rest, 1 pulled all
/// the way), with `deadzone` above rest cut out and the rest stretched back
/// to 0 to 1.
fn trigger(position: f64, deadzone: f64) -> f64 {
    let travel = (position + 1.0) / 2.0;
    ((travel - deadzone).max(0.0) / (1.0 - deadzone)).min(1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn limits() -> ActuatorLimits {
        ActuatorLimits {
            max_steering_angle_rad: 0.4,
            max_steering_rate_rad_s: 5.0,
            max_speed_mps: 2.0,
            max_accel_mps2: 5.0,
            max_decel_mps2: 5.0,
        }
    }

    fn at_rest(config: &JoystickConfig) -> PadState {
        let mut pad = PadState::default();
        pad.axes[config.throttle_axis as usize] = Some(-1.0);
        pad.axes[config.reverse_axis as usize] = Some(-1.0);
        pad
    }

    #[test]
    fn a_pad_at_rest_asks_for_nothing() {
        let config = JoystickConfig::default();
        let mut pad = at_rest(&config);
        pad.axes[config.steering_axis as usize] = Some(0.05);
        assert_eq!(pad.command(&config, &limits()), VescCommand::default());
    }

    #[test]
    fn unreported_axes_read_as_at_rest() {
        let config = JoystickConfig::default();
        assert_eq!(
            PadState::default().command(&config, &limits()),
            VescCommand::default()
        );
    }

    #[test]
    fn full_travel_reaches_the_limits() {
        let config = JoystickConfig::default();
        let mut pad = at_rest(&config);
        pad.axes[config.steering_axis as usize] = Some(-1.0);
        pad.axes[config.throttle_axis as usize] = Some(1.0);
        // max_speed_mps (3) is capped by the vehicle's 2.
        assert_eq!(pad.command(&config, &limits()), VescCommand::new(-0.4, 2.0));
        pad.axes[config.throttle_axis as usize] = Some(-1.0);
        pad.axes[config.reverse_axis as usize] = Some(1.0);
        assert_eq!(
            pad.command(&config, &limits()),
            VescCommand::new(-0.4, -2.0)
        );
    }

    #[test]
    fn the_deadman_button_must_be_held() {
        let config = JoystickConfig {
            deadman_button: Some(4),
            ..JoystickConfig::default()
        };
        let mut pad = at_rest(&config);
        pad.axes[config.throttle_axis as usize] = Some(1.0);
        assert_eq!(pad.command(&config, &limits()), VescCommand::default());
        pad.buttons[4] = true;
        assert_eq!(pad.command(&config, &limits()).speed_mps, 2.0);
    }

    #[test]
    fn events_update_axes_and_buttons() {
        let mut pad = PadState::default();
        // An opening-state axis event: axis 5 all the way down.
        pad.apply(&[0, 0, 0, 0, 0x01, 0x80, JS_EVENT_AXIS | JS_EVENT_INIT, 5]);
        assert_eq!(pad.axes[5], Some(-1.0));
        pad.apply(&[0, 0, 0, 0, 1, 0, JS_EVENT_BUTTON, 4]);
        assert!(pad.buttons[4]);
    }

    #[test]
    fn a_device_without_the_configured_controls_is_refused() {
        let config = JoystickConfig::default();
        let pad = PadLayout {
            name: "Xbox 360 Controller".into(),
            axes: 8,
            buttons: 11,
        };
        assert_eq!(pad.check(&config), Ok(()));
        // A touchscreen: just x and y.
        let touchscreen = PadLayout {
            name: "ILIT2901:00 222A:5539 Mouse".into(),
            axes: 2,
            buttons: 5,
        };
        assert!(touchscreen.check(&config).is_err());
        let deadman = JoystickConfig {
            deadman_button: Some(11),
            ..JoystickConfig::default()
        };
        assert!(pad.check(&deadman).is_err());
    }

    #[test]
    fn the_stick_deadzone_is_cut_out() {
        assert_eq!(stick(0.1, 0.1), 0.0);
        assert_eq!(stick(-1.0, 0.1), -1.0);
        assert!((stick(0.55, 0.1) - 0.5).abs() < 1e-12);
    }
}
