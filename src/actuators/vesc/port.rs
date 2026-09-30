//! [`VescPort`]: the VESC's USB serial port, asking it one question at a time.

use super::protocol::{self, Decoder, Firmware, Imu, Values};
use std::fs::{File, OpenOptions};
use std::io::{Read, Write};
use std::os::fd::AsRawFd;
use std::os::unix::fs::OpenOptionsExt;
use std::path::Path;
use std::time::{Duration, Instant};

/// The VESC's serial port (over USB, so the baud rate doesn't matter),
/// asking it one question at a time. Reads the VESC's state, moves the
/// steering servo and drives the motor. The motor commands keep the VESC's
/// own timeout from stopping the motor only as long as they keep coming.
pub struct VescPort {
    file: File,
    decoder: Decoder,
    /// How long a reply may take.
    timeout: Duration,
}

impl VescPort {
    /// Opens the VESC at `path` (e.g. `/dev/sensors/vesc`), in raw mode,
    /// discarding whatever it sent before - exclusively: until this port is
    /// dropped, opening it again fails, so two programs never drive the car
    /// at once.
    pub fn open(path: &Path, timeout: Duration) -> Result<Self, String> {
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(libc::O_NOCTTY)
            .open(path)
            .map_err(|err| match err.raw_os_error() {
                // Another `VescPort` holds it - see `exclusive`.
                Some(libc::EBUSY) => {
                    format!("{} is already in use by another program", path.display())
                }
                _ => format!("can't open {}: {err}", path.display()),
            })?;
        exclusive(&file).map_err(|err| {
            format!(
                "{} is already in use by another program: {err}",
                path.display()
            )
        })?;
        raw_mode(&file).map_err(|err| format!("can't set up {}: {err}", path.display()))?;
        Ok(Self {
            file,
            decoder: Decoder::default(),
            timeout,
        })
    }

    pub fn firmware(&mut self) -> Result<Firmware, String> {
        protocol::parse_firmware(&self.ask(&protocol::request(protocol::COMM_FW_VERSION))?)
    }

    pub fn values(&mut self) -> Result<Values, String> {
        protocol::parse_values(&self.ask(&protocol::request(protocol::COMM_GET_VALUES))?)
    }

    pub fn imu(&mut self) -> Result<Imu, String> {
        protocol::parse_imu(&self.ask(&protocol::imu_request())?)
    }

    /// Moves the steering servo to `position` - see
    /// [`protocol::COMM_SET_SERVO_POS`]. The servo holds it until told
    /// otherwise.
    pub fn set_servo(&mut self, position: f64) -> Result<(), String> {
        self.send(&protocol::servo_request(position), "move the servo")
    }

    /// Holds the motor at `erpm` - see [`protocol::COMM_SET_RPM`].
    pub fn set_rpm(&mut self, erpm: i32) -> Result<(), String> {
        self.send(&protocol::rpm_request(erpm), "set the motor's speed")
    }

    /// Brakes the motor with `amps`.
    pub fn brake(&mut self, amps: f64) -> Result<(), String> {
        self.send(&protocol::brake_request(amps), "brake")
    }

    /// Releases the motor, to coast.
    pub fn release(&mut self) -> Result<(), String> {
        self.send(&protocol::release_request(), "release the motor")
    }

    /// Sends `request`, which has no reply.
    fn send(&mut self, request: &[u8], what: &str) -> Result<(), String> {
        self.file
            .write_all(&protocol::frame(request))
            .map_err(|err| format!("can't {what}: {err}"))
    }

    /// Sends `request` and waits for the reply to it - the first packet
    /// repeating its command ID. Anything else the VESC sends meanwhile
    /// (e.g. a debug print) is skipped.
    fn ask(&mut self, request: &[u8]) -> Result<Vec<u8>, String> {
        let command = request[0];
        self.file
            .write_all(&protocol::frame(request))
            .map_err(|err| format!("can't send command {command}: {err}"))?;
        let deadline = Instant::now() + self.timeout;
        let mut chunk = [0u8; 512];
        loop {
            while let Some(payload) = self.decoder.next_payload() {
                if payload[0] == command {
                    return Ok(payload);
                }
            }
            if Instant::now() > deadline {
                return Err(format!(
                    "no reply to command {command} in {} ms",
                    self.timeout.as_millis()
                ));
            }
            // Returns 0 bytes after a tenth of a second of silence - see
            // `raw_mode`.
            let read = self
                .file
                .read(&mut chunk)
                .map_err(|err| format!("can't read from the VESC: {err}"))?;
            self.decoder.push(&chunk[..read]);
        }
    }
}

/// Takes the terminal `file` is for this process alone: a `TIOCEXCL`
/// open fails for everyone else, and a lock taken by someone already holding
/// it fails for this process.
fn exclusive(file: &File) -> std::io::Result<()> {
    let fd = file.as_raw_fd();
    // SAFETY: `fd` is an open descriptor for the whole call, and neither
    // call takes a pointer.
    unsafe {
        if libc::flock(fd, libc::LOCK_EX | libc::LOCK_NB) != 0 {
            return Err(std::io::Error::last_os_error());
        }
        if libc::ioctl(fd, libc::TIOCEXCL) != 0 {
            return Err(std::io::Error::last_os_error());
        }
    }
    Ok(())
}

/// Puts the terminal `file` is in raw mode - bytes through untouched, no
/// echo - with reads returning whatever arrived, or nothing after 0.1 s of
/// silence, and drops anything already received.
fn raw_mode(file: &File) -> std::io::Result<()> {
    let fd = file.as_raw_fd();
    // SAFETY: `fd` is an open descriptor for the whole call, and `termios`
    // is plain data `tcgetattr` fills before anything reads it.
    unsafe {
        let mut termios: libc::termios = std::mem::zeroed();
        if libc::tcgetattr(fd, &mut termios) != 0 {
            return Err(std::io::Error::last_os_error());
        }
        libc::cfmakeraw(&mut termios);
        termios.c_cflag |= libc::CLOCAL | libc::CREAD;
        termios.c_cc[libc::VMIN] = 0;
        termios.c_cc[libc::VTIME] = 1;
        if libc::tcsetattr(fd, libc::TCSANOW, &termios) != 0 {
            return Err(std::io::Error::last_os_error());
        }
        if libc::tcflush(fd, libc::TCIFLUSH) != 0 {
            return Err(std::io::Error::last_os_error());
        }
    }
    Ok(())
}
