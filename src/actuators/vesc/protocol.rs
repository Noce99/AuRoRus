//! The part of the VESC firmware's packet protocol (see `comm/packet.c` and
//! `comm/commands.c` in github.com/vedderb/bldc) the car needs - framing,
//! and building and decoding requests - with no I/O, so it can be tested
//! against hand-built packets.
//!
//! A packet is a start byte (`2`, `3` or `4` for a 1, 2 or 3-byte length),
//! the payload's big-endian length, the payload, its CRC16 (CCITT/XModem,
//! big-endian) and an end byte (`3`). A payload's first byte is its command
//! ID, which the VESC's reply repeats; every number after it is big-endian.

/// The firmware's version and hardware name.
pub const COMM_FW_VERSION: u8 = 0;
/// Temperatures, currents, ERPM, input voltage, tachometer and fault code.
pub const COMM_GET_VALUES: u8 = 4;
/// Orientation, accelerations and angular rates from the onboard IMU.
pub const COMM_GET_IMU_DATA: u8 = 65;
/// Drives the motor with this current, in milliamps; `0` releases it, to
/// coast.
pub const COMM_SET_CURRENT: u8 = 6;
/// Brakes the motor with this current, in milliamps.
pub const COMM_SET_CURRENT_BRAKE: u8 = 7;
/// Holds the motor at this ERPM, with the VESC's own speed controller.
pub const COMM_SET_RPM: u8 = 8;
/// Moves the steering servo: its pulse width, from the firmware's minimum
/// (`0`) to its maximum (`1`). No reply; ignored unless VESC Tool enables
/// the servo output.
pub const COMM_SET_SERVO_POS: u8 = 12;

/// `COMM_GET_IMU_DATA`'s field mask: roll/pitch/yaw (bits 0-2),
/// accelerations (3-5) and angular rates (6-8).
const IMU_MASK: u16 = 0x01FF;
/// Longest payload [`Decoder`] accepts - the firmware's own
/// `PACKET_MAX_PL_LEN`.
const MAX_PAYLOAD_LEN: usize = 512;
/// A packet's closing byte.
const END: u8 = 3;

/// The firmware's CRC16: CCITT polynomial `0x1021`, starting from `0`.
pub fn crc16(bytes: &[u8]) -> u16 {
    bytes.iter().fold(0u16, |crc, &byte| {
        (0..8).fold(crc ^ (u16::from(byte) << 8), |crc, _| {
            if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            }
        })
    })
}

/// `payload` as a packet.
pub fn frame(payload: &[u8]) -> Vec<u8> {
    let len = payload.len();
    let mut packet = Vec::with_capacity(len + 7);
    if len <= 0xFF {
        packet.extend([2, len as u8]);
    } else if len <= 0xFFFF {
        packet.push(3);
        packet.extend((len as u16).to_be_bytes());
    } else {
        packet.push(4);
        packet.extend(&(len as u32).to_be_bytes()[1..]);
    }
    packet.extend_from_slice(payload);
    packet.extend(crc16(payload).to_be_bytes());
    packet.push(END);
    packet
}

/// Splits a byte stream into packets' payloads, skipping whatever doesn't
/// frame as one - noise, or the tail of a packet it started reading halfway.
#[derive(Debug, Default)]
pub struct Decoder {
    buffer: Vec<u8>,
}

impl Decoder {
    /// Adds bytes read from the VESC.
    pub fn push(&mut self, bytes: &[u8]) {
        self.buffer.extend_from_slice(bytes);
    }

    /// The next whole packet's payload, or `None` until more bytes arrive.
    pub fn next_payload(&mut self) -> Option<Vec<u8>> {
        loop {
            let start = self.buffer.iter().position(|b| (2..=4).contains(b))?;
            self.buffer.drain(..start);
            let len_bytes = usize::from(self.buffer[0] - 1);
            if self.buffer.len() < 1 + len_bytes {
                return None;
            }
            let len = self.buffer[1..1 + len_bytes]
                .iter()
                .fold(0usize, |len, &b| (len << 8) | usize::from(b));
            if len == 0 || len > MAX_PAYLOAD_LEN {
                // Not a start byte after all.
                self.buffer.remove(0);
                continue;
            }
            let total = 1 + len_bytes + len + 3;
            if self.buffer.len() < total {
                return None;
            }
            let payload = &self.buffer[1 + len_bytes..1 + len_bytes + len];
            let crc = u16::from_be_bytes([self.buffer[total - 3], self.buffer[total - 2]]);
            if self.buffer[total - 1] == END && crc16(payload) == crc {
                let payload = payload.to_vec();
                self.buffer.drain(..total);
                return Some(payload);
            }
            self.buffer.remove(0);
        }
    }
}

/// The request for `command`, when it takes no arguments.
pub fn request(command: u8) -> Vec<u8> {
    vec![command]
}

/// The request for the IMU fields [`parse_imu`] decodes.
pub fn imu_request() -> Vec<u8> {
    let [hi, lo] = IMU_MASK.to_be_bytes();
    vec![COMM_GET_IMU_DATA, hi, lo]
}

/// The request moving the servo to `position`, clamped to `0..=1` - sent
/// as thousandths, like the firmware's `buffer_get_float16(data, 1000)`
/// reads it.
pub fn servo_request(position: f64) -> Vec<u8> {
    let thousandths = (position.clamp(0.0, 1.0) * 1000.0).round() as i16;
    let [hi, lo] = thousandths.to_be_bytes();
    vec![COMM_SET_SERVO_POS, hi, lo]
}

/// The request holding the motor at `erpm`.
pub fn rpm_request(erpm: i32) -> Vec<u8> {
    command_with_i32(COMM_SET_RPM, erpm)
}

/// The request braking the motor with `amps`.
pub fn brake_request(amps: f64) -> Vec<u8> {
    command_with_i32(COMM_SET_CURRENT_BRAKE, (amps * 1000.0).round() as i32)
}

/// The request releasing the motor, to coast: zero current.
pub fn release_request() -> Vec<u8> {
    command_with_i32(COMM_SET_CURRENT, 0)
}

fn command_with_i32(command: u8, value: i32) -> Vec<u8> {
    let mut request = vec![command];
    request.extend(value.to_be_bytes());
    request
}

/// Reads big-endian numbers off a payload, failing past its end.
struct Reader<'a> {
    bytes: &'a [u8],
    what: &'static str,
}

impl<'a> Reader<'a> {
    /// A reader for `payload`, a reply to `command`, past its command ID.
    fn new(payload: &'a [u8], command: u8, what: &'static str) -> Result<Self, String> {
        match payload.split_first() {
            Some((&id, bytes)) if id == command => Ok(Self { bytes, what }),
            Some((&id, _)) => Err(format!("expected a {what} reply, got command {id}")),
            None => Err(format!("empty {what} reply")),
        }
    }

    fn take<const N: usize>(&mut self) -> Result<[u8; N], String> {
        let Some((head, rest)) = self.bytes.split_first_chunk::<N>() else {
            return Err(format!("{} reply ends early", self.what));
        };
        self.bytes = rest;
        Ok(*head)
    }

    fn is_empty(&self) -> bool {
        self.bytes.is_empty()
    }

    fn u8(&mut self) -> Result<u8, String> {
        Ok(self.take::<1>()?[0])
    }

    fn u16(&mut self) -> Result<u16, String> {
        Ok(u16::from_be_bytes(self.take()?))
    }

    fn i32(&mut self) -> Result<i32, String> {
        Ok(i32::from_be_bytes(self.take()?))
    }

    /// The firmware's `buffer_get_float16`: an `i16` over `scale`.
    fn f16(&mut self, scale: f64) -> Result<f64, String> {
        Ok(f64::from(i16::from_be_bytes(self.take()?)) / scale)
    }

    /// The firmware's `buffer_get_float32`: an `i32` over `scale`.
    fn f32(&mut self, scale: f64) -> Result<f64, String> {
        Ok(f64::from(self.i32()?) / scale)
    }

    /// The firmware's `buffer_get_float32_auto`: IEEE-754-like, with its
    /// own exponent bias and no subnormals.
    fn f32_auto(&mut self) -> Result<f64, String> {
        let bits = u32::from_be_bytes(self.take()?);
        let exponent = ((bits >> 23) & 0xFF) as i32;
        let significand = bits & 0x7F_FFFF;
        let value = if exponent == 0 && significand == 0 {
            0.0
        } else {
            (f64::from(significand) / 16_777_216.0 + 0.5) * 2f64.powi(exponent - 126)
        };
        Ok(if bits >> 31 == 1 { -value } else { value })
    }

    /// A NUL-terminated string.
    fn string(&mut self) -> Result<String, String> {
        let Some(end) = self.bytes.iter().position(|&b| b == 0) else {
            return Err(format!("{} reply has an unterminated string", self.what));
        };
        let string = String::from_utf8_lossy(&self.bytes[..end]).into_owned();
        self.bytes = &self.bytes[end + 1..];
        Ok(string)
    }
}

/// Which firmware the VESC runs, on what.
#[derive(Debug, Clone, PartialEq)]
pub struct Firmware {
    pub major: u8,
    pub minor: u8,
    /// The hardware it was built for, e.g. `60_MK5`.
    pub hardware: String,
    /// The microcontroller's unique ID, as hex.
    pub uuid: String,
}

/// Parses the reply to [`COMM_FW_VERSION`]. Newer firmware appends more
/// fields, which are ignored.
pub fn parse_firmware(payload: &[u8]) -> Result<Firmware, String> {
    let mut reader = Reader::new(payload, COMM_FW_VERSION, "firmware version")?;
    Ok(Firmware {
        major: reader.u8()?,
        minor: reader.u8()?,
        hardware: reader.string()?,
        uuid: reader
            .take::<12>()?
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect(),
    })
}

/// The motor controller's state, from [`COMM_GET_VALUES`].
#[derive(Debug, Clone, PartialEq)]
pub struct Values {
    pub temp_fet_c: f64,
    pub temp_motor_c: f64,
    /// Average motor current since the previous request, in amperes.
    pub motor_current_a: f64,
    /// Average battery current since the previous request, in amperes.
    pub input_current_a: f64,
    /// Duty cycle, in `-1..=1`.
    pub duty: f64,
    /// Electrical RPM: the motor's RPM times its pole pairs, signed.
    pub erpm: f64,
    pub input_voltage_v: f64,
    /// Motor steps counted since boot, signed - six per electrical turn.
    pub tachometer: i32,
    pub fault: Fault,
    /// Whether the firmware's command timeout has tripped (it stops the
    /// motor when no command arrives in time) - `None` from firmware too
    /// old to report it.
    pub timed_out: Option<bool>,
}

/// Parses the reply to [`COMM_GET_VALUES`].
pub fn parse_values(payload: &[u8]) -> Result<Values, String> {
    let mut reader = Reader::new(payload, COMM_GET_VALUES, "values")?;
    let temp_fet_c = reader.f16(1e1)?;
    let temp_motor_c = reader.f16(1e1)?;
    let motor_current_a = reader.f32(1e2)?;
    let input_current_a = reader.f32(1e2)?;
    let _id = reader.f32(1e2)?;
    let _iq = reader.f32(1e2)?;
    let duty = reader.f16(1e3)?;
    let erpm = reader.f32(1e0)?;
    let input_voltage_v = reader.f16(1e1)?;
    let _amp_hours = reader.f32(1e4)?;
    let _amp_hours_charged = reader.f32(1e4)?;
    let _watt_hours = reader.f32(1e4)?;
    let _watt_hours_charged = reader.f32(1e4)?;
    let tachometer = reader.i32()?;
    let _tachometer_abs = reader.i32()?;
    let fault = Fault(reader.u8()?);
    // Newer firmware only: PID position, controller ID, the three MOSFET
    // temperatures, vd, vq, then the timeout status.
    let timed_out = (|| {
        reader.take::<{ 4 + 1 + 6 + 4 + 4 }>().ok()?;
        reader.u8().ok().map(|status| status & 1 == 1)
    })();
    Ok(Values {
        temp_fet_c,
        temp_motor_c,
        motor_current_a,
        input_current_a,
        duty,
        erpm,
        input_voltage_v,
        tachometer,
        fault,
        timed_out,
    })
}

/// A motor controller fault code (`mc_fault_code` in the firmware).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Fault(pub u8);

impl Fault {
    pub fn is_none(self) -> bool {
        self.0 == 0
    }

    /// The firmware's name for it, without `FAULT_CODE_`.
    pub fn name(self) -> &'static str {
        const NAMES: [&str; 34] = [
            "NONE",
            "OVER_VOLTAGE",
            "UNDER_VOLTAGE",
            "DRV",
            "ABS_OVER_CURRENT",
            "OVER_TEMP_FET",
            "OVER_TEMP_MOTOR",
            "GATE_DRIVER_OVER_VOLTAGE",
            "GATE_DRIVER_UNDER_VOLTAGE",
            "MCU_UNDER_VOLTAGE",
            "BOOTING_FROM_WATCHDOG_RESET",
            "ENCODER_SPI",
            "ENCODER_SINCOS_BELOW_MIN_AMPLITUDE",
            "ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE",
            "FLASH_CORRUPTION",
            "HIGH_OFFSET_CURRENT_SENSOR_1",
            "HIGH_OFFSET_CURRENT_SENSOR_2",
            "HIGH_OFFSET_CURRENT_SENSOR_3",
            "UNBALANCED_CURRENTS",
            "BRK",
            "RESOLVER_LOT",
            "RESOLVER_DOS",
            "RESOLVER_LOS",
            "FLASH_CORRUPTION_APP_CFG",
            "FLASH_CORRUPTION_MC_CFG",
            "ENCODER_NO_MAGNET",
            "ENCODER_MAGNET_TOO_STRONG",
            "PHASE_FILTER",
            "ENCODER_FAULT",
            "LV_OUTPUT_FAULT",
            "ENCODER_SLIP",
            "OVERSPEED",
            "UNDERSPEED",
            "ABS_OVERSPEED",
        ];
        NAMES.get(usize::from(self.0)).copied().unwrap_or("UNKNOWN")
    }
}

/// The onboard IMU's readings, from [`COMM_GET_IMU_DATA`], in the frame
/// VESC Tool's IMU rotation settings put them in.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Imu {
    /// Roll, pitch and yaw from the firmware's attitude filter, in radians.
    pub rpy_rad: [f64; 3],
    /// Accelerations along x, y, z, in g.
    pub accel_g: [f64; 3],
    /// Angular rates about x, y, z, in degrees/second.
    pub gyro_deg_s: [f64; 3],
}

/// Parses the reply to [`imu_request`].
pub fn parse_imu(payload: &[u8]) -> Result<Imu, String> {
    let mut reader = Reader::new(payload, COMM_GET_IMU_DATA, "IMU")?;
    let mask = reader.u16()?;
    if mask != IMU_MASK {
        return Err(format!("IMU reply has fields {mask:#06x}, expected {IMU_MASK:#06x}"));
    }
    let mut three = || -> Result<[f64; 3], String> {
        Ok([reader.f32_auto()?, reader.f32_auto()?, reader.f32_auto()?])
    };
    let imu = Imu {
        rpy_rad: three()?,
        accel_g: three()?,
        gyro_deg_s: three()?,
    };
    // Then the controller ID.
    reader.u8()?;
    if !reader.is_empty() {
        return Err("IMU reply is longer than its fields".into());
    }
    Ok(imu)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The firmware's `buffer_append_float32_auto`.
    fn f32_auto(value: f32) -> [u8; 4] {
        if value == 0.0 {
            return [0; 4];
        }
        let (mut exponent, mut significand) = (0i32, value.abs());
        while significand >= 1.0 {
            significand /= 2.0;
            exponent += 1;
        }
        while significand < 0.5 {
            significand *= 2.0;
            exponent -= 1;
        }
        let bits = (((exponent + 126) as u32 & 0xFF) << 23)
            | (((significand - 0.5) * 2.0 * 8_388_608.0) as u32 & 0x7F_FFFF)
            | if value < 0.0 { 1 << 31 } else { 0 };
        bits.to_be_bytes()
    }

    #[test]
    fn crc16_matches_the_ccitt_xmodem_check_value() {
        assert_eq!(crc16(b"123456789"), 0x31C3);
    }

    #[test]
    fn requests_frame_as_the_firmware_expects() {
        let crc = crc16(&[COMM_GET_VALUES]).to_be_bytes();
        assert_eq!(frame(&request(COMM_GET_VALUES)), vec![2, 1, 4, crc[0], crc[1], 3]);
        assert_eq!(imu_request(), vec![65, 0x01, 0xFF]);
        let long = frame(&[7; 300]);
        assert_eq!(&long[..3], &[3, 1, 44]);
        assert_eq!(long.len(), 3 + 300 + 3);
    }

    #[test]
    fn servo_positions_are_sent_in_thousandths_and_clamped() {
        assert_eq!(servo_request(0.5), vec![12, 0x01, 0xF4]);
        assert_eq!(servo_request(0.123), vec![12, 0x00, 123]);
        assert_eq!(servo_request(1.7), vec![12, 0x03, 0xE8]);
        assert_eq!(servo_request(-0.2), vec![12, 0, 0]);
    }

    #[test]
    fn motor_commands_carry_a_big_endian_i32() {
        assert_eq!(rpm_request(-3000), vec![8, 0xFF, 0xFF, 0xF4, 0x48]);
        assert_eq!(brake_request(2.5), vec![7, 0, 0, 0x09, 0xC4]);
        assert_eq!(release_request(), vec![6, 0, 0, 0, 0]);
    }

    #[test]
    fn the_decoder_skips_noise_and_waits_for_whole_packets() {
        let mut decoder = Decoder::default();
        let packet = frame(&[COMM_GET_VALUES, 1, 2, 3]);
        decoder.push(&[0xFF, 2, 9, 0x42]);
        decoder.push(&packet[..4]);
        assert_eq!(decoder.next_payload(), None);
        decoder.push(&packet[4..]);
        decoder.push(&frame(&[COMM_FW_VERSION]));
        assert_eq!(decoder.next_payload(), Some(vec![COMM_GET_VALUES, 1, 2, 3]));
        assert_eq!(decoder.next_payload(), Some(vec![COMM_FW_VERSION]));
        assert_eq!(decoder.next_payload(), None);
    }

    #[test]
    fn the_decoder_drops_a_corrupted_packet() {
        let mut decoder = Decoder::default();
        let mut bad = frame(&[COMM_GET_VALUES, 1, 2, 3]);
        bad[3] ^= 0x10;
        decoder.push(&bad);
        decoder.push(&frame(&[COMM_FW_VERSION]));
        assert_eq!(decoder.next_payload(), Some(vec![COMM_FW_VERSION]));
    }

    #[test]
    fn parses_a_firmware_version() {
        let mut payload = vec![COMM_FW_VERSION, 6, 5];
        payload.extend(b"60_MK5\0");
        payload.extend(0..12);
        payload.extend([0, 0, 0, 0, 0, 0, 0, 0]);
        let firmware = parse_firmware(&payload).unwrap();
        assert_eq!((firmware.major, firmware.minor), (6, 5));
        assert_eq!(firmware.hardware, "60_MK5");
        assert_eq!(firmware.uuid, "000102030405060708090a0b");
    }

    fn values_payload(timeout_status: Option<u8>) -> Vec<u8> {
        let mut payload = vec![COMM_GET_VALUES];
        payload.extend(315i16.to_be_bytes()); // 31.5 C
        payload.extend(250i16.to_be_bytes()); // 25.0 C
        payload.extend(123i32.to_be_bytes()); // 1.23 A
        payload.extend((-45i32).to_be_bytes()); // -0.45 A
        payload.extend([0; 8]); // id, iq
        payload.extend((-120i16).to_be_bytes()); // -0.12
        payload.extend((-3000i32).to_be_bytes());
        payload.extend(168i16.to_be_bytes()); // 16.8 V
        payload.extend([0; 16]); // amp and watt hours
        payload.extend(777i32.to_be_bytes());
        payload.extend(900i32.to_be_bytes());
        payload.push(2); // UNDER_VOLTAGE
        if let Some(status) = timeout_status {
            payload.extend([0; 4 + 1 + 6 + 4 + 4]);
            payload.push(status);
        }
        payload
    }

    #[test]
    fn parses_values() {
        let values = parse_values(&values_payload(Some(1))).unwrap();
        assert_eq!(values.temp_fet_c, 31.5);
        assert_eq!(values.temp_motor_c, 25.0);
        assert_eq!(values.motor_current_a, 1.23);
        assert_eq!(values.input_current_a, -0.45);
        assert_eq!(values.duty, -0.12);
        assert_eq!(values.erpm, -3000.0);
        assert_eq!(values.input_voltage_v, 16.8);
        assert_eq!(values.tachometer, 777);
        assert_eq!(values.fault.name(), "UNDER_VOLTAGE");
        assert_eq!(values.timed_out, Some(true));
    }

    #[test]
    fn older_firmware_values_have_no_timeout_status() {
        assert_eq!(parse_values(&values_payload(None)).unwrap().timed_out, None);
        assert!(parse_values(&values_payload(None)[..20]).is_err());
    }

    #[test]
    fn a_reply_to_another_command_is_rejected() {
        let err = parse_values(&[COMM_FW_VERSION, 1, 2]).unwrap_err();
        assert!(err.contains("command 0"), "{err}");
    }

    #[test]
    fn parses_imu_data() {
        let mut payload = vec![COMM_GET_IMU_DATA, 0x01, 0xFF];
        for value in [0.1, -0.2, 3.0, 0.01, -0.02, 1.0, 0.5, -1.5, 90.0] {
            payload.extend(f32_auto(value));
        }
        payload.push(0);
        let imu = parse_imu(&payload).unwrap();
        let close = |a: [f64; 3], b: [f64; 3]| a.iter().zip(b).all(|(a, b)| (a - b).abs() < 1e-6);
        assert!(close(imu.rpy_rad, [0.1, -0.2, 3.0]), "{:?}", imu.rpy_rad);
        assert!(close(imu.accel_g, [0.01, -0.02, 1.0]), "{:?}", imu.accel_g);
        assert!(close(imu.gyro_deg_s, [0.5, -1.5, 90.0]), "{:?}", imu.gyro_deg_s);
    }
}
