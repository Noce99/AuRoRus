//! The [`VescStatus`] topic: the real car's motor controller's own state -
//! battery, temperatures, currents, faults - and what it was last told to do.
//! Published by [`crate::actuators::Vesc`]; nothing publishes it in
//! simulation.

/// Name of the topic a [`VescStatus`] is published on.
pub const VESC_STATUS_TOPIC_NAME: &str = "vesc_status";

/// What the VESC reports, and what it was last commanded.
#[derive(Debug, Clone, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct VescStatus {
    /// Battery voltage, in volts.
    pub input_voltage_v: f64,
    /// Whether the battery is below the voltage it should be recharged at
    /// (see [`crate::actuators::VescConfig::low_battery_v`]).
    pub low_battery: bool,
    /// The battery's estimated charge, `0.0` (empty) to `1.0` (full), from
    /// its voltage averaged over a few seconds - reading emptier than it is
    /// while the motor draws current (see
    /// [`crate::actuators::vesc::battery_charge`]).
    pub battery_charge: f64,
    /// Battery current, in amperes - averaged since the previous reading.
    pub input_current_a: f64,
    /// Motor current, in amperes - averaged since the previous reading.
    pub motor_current_a: f64,
    pub temp_fet_c: f64,
    pub temp_motor_c: f64,
    /// Electrical RPM the motor actually turns at, signed.
    pub erpm: f64,
    /// The speed that makes, in meters/second - signed, negative reversing.
    pub wheel_speed_mps: f64,
    /// Motor steps counted since the VESC booted, signed - six per
    /// electrical turn.
    pub tachometer: i32,
    /// The firmware's fault, e.g. `NONE` or `UNDER_VOLTAGE`.
    pub fault: String,
    /// Whether the firmware's own command timeout has tripped (it then
    /// brakes the motor) - `None` from firmware too old to report it.
    pub timed_out: Option<bool>,
    /// The servo position last sent, in `0..=1`.
    pub servo_position: f64,
    /// The ERPM last asked for - `None` while braking.
    pub commanded_erpm: Option<i32>,
}
