//! The part of Hokuyo's SCIP 2.0 protocol [`super::HokuyoLidar`] speaks -
//! building commands and decoding the sensor's replies, with no I/O, so it
//! can be tested against hand-built replies.
//!
//! Every reply is a block of `\n`-terminated lines ending with an empty one:
//! the echoed command, a status line (two status characters and their
//! checksum), then the payload. Numbers are sent as runs of 6-bit
//! characters (each one's value plus `0x30`), most significant first.

use std::f64::consts::TAU;

/// The distance and intensity command: `ME` streams both for every scan.
const ME: &[u8] = b"ME";
/// Status of a command the sensor accepted.
const STATUS_OK: &[u8] = b"00";
/// Status of a streamed scan.
const STATUS_SCAN: &[u8] = b"99";

/// SCIP's checksum of `bytes`: the low 6 bits of their sum, plus `0x30`.
pub(super) fn checksum(bytes: &[u8]) -> u8 {
    let sum = bytes.iter().fold(0u32, |sum, &byte| sum + u32::from(byte));
    (sum & 0x3F) as u8 + 0x30
}

/// The number SCIP encodes as `chars`: 6 bits per character, each one's
/// value plus `0x30`, most significant first.
pub(super) fn decode(chars: &[u8]) -> u32 {
    chars.iter().fold(0, |value, &c| {
        (value << 6) | u32::from(c.wrapping_sub(0x30) & 0x3F)
    })
}

/// The lines of a reply block, without their `\n`s and the block's closing
/// empty line.
fn lines(block: &[u8]) -> Vec<&[u8]> {
    let block = block.strip_suffix(b"\n\n").unwrap_or(block);
    block.split(|&byte| byte == b'\n').collect()
}

/// Checks a status line: `expected`, followed by its checksum.
fn check_status(line: &[u8], expected: &[u8]) -> Result<(), String> {
    let [a, b, sum] = line else {
        return Err(format!(
            "malformed status line {:?}",
            String::from_utf8_lossy(line)
        ));
    };
    let status = [*a, *b];
    if checksum(&status) != *sum {
        return Err(format!(
            "bad checksum on status {:?}",
            String::from_utf8_lossy(line)
        ));
    }
    if status != expected {
        return Err(format!(
            "sensor replied with status {:?} (expected {:?})",
            String::from_utf8_lossy(&status),
            String::from_utf8_lossy(expected)
        ));
    }
    Ok(())
}

/// A line of data: its payload, followed by the payload's checksum.
fn data(line: &[u8]) -> Result<&[u8], String> {
    let Some((&sum, payload)) = line.split_last() else {
        return Err("empty data line".into());
    };
    if checksum(payload) != sum {
        return Err(format!(
            "bad checksum on data line {:?}",
            String::from_utf8_lossy(line)
        ));
    }
    Ok(payload)
}

/// What `PP` says about the sensor: its range and how its steps are laid out.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct Parameters {
    /// The sensor's model, e.g. `UTM-30LX-EW`.
    pub model: String,
    /// Shortest distance it measures, in millimeters.
    pub dmin_mm: u32,
    /// Longest distance it measures, in millimeters.
    pub dmax_mm: u32,
    /// How many steps a full turn would have.
    pub ares: u32,
    /// Its first measured step.
    pub amin: u32,
    /// Its last measured step.
    pub amax: u32,
    /// The step pointing straight ahead.
    pub afrt: u32,
    /// How fast it spins, in rotations per minute - one scan per rotation.
    pub scan_rpm: u32,
}

impl Parameters {
    /// How many scans it sends per second.
    pub fn rate_hz(&self) -> f64 {
        f64::from(self.scan_rpm) / 60.0
    }
}

/// Parses the reply to `PP`: `KEY:VALUE;<checksum>` lines.
pub(super) fn parse_parameters(block: &[u8]) -> Result<Parameters, String> {
    let lines = lines(block);
    let [echo, status, fields @ ..] = lines.as_slice() else {
        return Err("truncated PP reply".into());
    };
    if *echo != b"PP" {
        return Err(format!(
            "expected PP's echo, got {:?}",
            String::from_utf8_lossy(echo)
        ));
    }
    check_status(status, STATUS_OK)?;

    let field = |key: &str| -> Result<String, String> {
        fields
            .iter()
            .map(|line| String::from_utf8_lossy(line))
            .find_map(|line| {
                let (name, rest) = line.split_once(':')?;
                let (value, _checksum) = rest.rsplit_once(';')?;
                (name == key).then(|| value.to_string())
            })
            .ok_or_else(|| format!("PP reply has no {key}"))
    };
    let number = |key: &str| -> Result<u32, String> {
        let value = field(key)?;
        value
            .trim()
            .parse()
            .map_err(|_| format!("PP's {key} isn't a number: {value:?}"))
    };
    Ok(Parameters {
        model: field("MODL")?,
        dmin_mm: number("DMIN")?,
        dmax_mm: number("DMAX")?,
        ares: number("ARES")?,
        amin: number("AMIN")?,
        amax: number("AMAX")?,
        afrt: number("AFRT")?,
        scan_rpm: number("SCAN")?,
    })
}

/// Which steps to ask for: a window centered on the front step, as close to
/// a wanted field of view as the sensor allows, in groups of `cluster`
/// steps (the sensor reports each group's closest reading).
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct Layout {
    pub start_step: u32,
    pub end_step: u32,
    pub cluster: u32,
    /// How many readings every scan has: one per group.
    pub num_points: usize,
    /// The angle between the first and the last reading, in radians - what
    /// [`crate::topics::LidarScan::fov`] needs for its evenly spread
    /// readings to point where the sensor measured.
    pub fov_rad: f32,
}

impl Layout {
    /// The widest window of at most `fov_rad`, centered on the front step,
    /// that fits the sensor. `cluster` must be odd, so the middle group is
    /// centered on the front step too.
    pub fn new(parameters: &Parameters, fov_rad: f32, cluster: u32) -> Result<Self, String> {
        if cluster == 0 || cluster.is_multiple_of(2) || cluster > 99 {
            return Err(format!("cluster must be odd and in 1..=99, got {cluster}"));
        }
        let Parameters {
            amin,
            amax,
            afrt,
            ares,
            ..
        } = *parameters;
        if !(amin..=amax).contains(&afrt) || ares == 0 {
            return Err(format!("inconsistent sensor parameters {parameters:?}"));
        }
        let step_rad = TAU / f64::from(ares);
        let group_rad = step_rad * f64::from(cluster);
        let side = (cluster - 1) / 2;
        // Groups on each side of the middle one: as many as `fov_rad`
        // wants, as long as they stay within the sensor's steps.
        let wanted = (f64::from(fov_rad) / 2.0 / group_rad + 1e-6)
            .floor()
            .max(0.0) as u32;
        let fits_left = (afrt - amin).saturating_sub(side) / cluster;
        let fits_right = (amax - afrt).saturating_sub(side) / cluster;
        let half = wanted.min(fits_left).min(fits_right);
        Ok(Self {
            start_step: afrt - half * cluster - side,
            end_step: afrt + half * cluster + side,
            cluster,
            num_points: 2 * half as usize + 1,
            fov_rad: (f64::from(2 * half) * group_rad) as f32,
        })
    }

    /// The `ME` command that streams these steps, with distances and
    /// intensities, for as long as the connection lasts.
    pub fn command(&self) -> String {
        format!(
            "ME{:04}{:04}{:02}000\n",
            self.start_step, self.end_step, self.cluster
        )
    }
}

/// Checks the reply to [`Layout::command`], sent before the first scan.
pub(super) fn check_stream_started(block: &[u8]) -> Result<(), String> {
    let lines = lines(block);
    let [echo, status, ..] = lines.as_slice() else {
        return Err("truncated ME reply".into());
    };
    if !echo.starts_with(ME) {
        return Err(format!(
            "expected ME's echo, got {:?}",
            String::from_utf8_lossy(echo)
        ));
    }
    check_status(status, STATUS_OK)
}

/// One scan's raw readings, one per group of steps.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct Readings {
    /// The sensor's clock when it took the scan, in milliseconds (wraps
    /// every 2^24 ms).
    pub timestamp_ms: u32,
    pub distances_mm: Vec<u32>,
    pub intensities: Vec<u32>,
}

/// Parses one streamed `ME` scan of `num_points` readings: echo, status,
/// timestamp, then 3-character distance and intensity pairs, split over
/// checksummed lines of at most 64 characters.
pub(super) fn parse_scan(block: &[u8], num_points: usize) -> Result<Readings, String> {
    let lines = lines(block);
    let [echo, status, timestamp, rest @ ..] = lines.as_slice() else {
        return Err("truncated scan".into());
    };
    if !echo.starts_with(ME) {
        return Err(format!(
            "expected ME's echo, got {:?}",
            String::from_utf8_lossy(echo)
        ));
    }
    check_status(status, STATUS_SCAN)?;
    let timestamp_ms = decode(data(timestamp)?);

    let mut payload = Vec::with_capacity(num_points * 6);
    for line in rest {
        payload.extend_from_slice(data(line)?);
    }
    if payload.len() != num_points * 6 {
        return Err(format!(
            "scan has {} characters of readings, expected {} for {num_points} points",
            payload.len(),
            num_points * 6
        ));
    }
    let (distances_mm, intensities) = payload
        .as_chunks::<6>()
        .0
        .iter()
        .map(|pair| (decode(&pair[..3]), decode(&pair[3..])))
        .unzip();
    Ok(Readings {
        timestamp_ms,
        distances_mm,
        intensities,
    })
}

#[cfg(test)]
pub(super) mod tests {
    use super::*;

    /// `value` as `len` SCIP characters.
    pub fn encode(value: u32, len: usize) -> Vec<u8> {
        (0..len)
            .rev()
            .map(|i| ((value >> (6 * i)) & 0x3F) as u8 + 0x30)
            .collect()
    }

    /// `payload` as a checksummed line.
    fn line(payload: &[u8]) -> Vec<u8> {
        let mut line = payload.to_vec();
        line.push(checksum(payload));
        line.push(b'\n');
        line
    }

    /// The block a sensor would stream for `readings`.
    pub fn scan_block(timestamp_ms: u32, readings: &[(u32, u32)]) -> Vec<u8> {
        let mut block = b"ME0000108001000\n99b\n".to_vec();
        block.extend(line(&encode(timestamp_ms, 4)));
        let payload: Vec<u8> = readings
            .iter()
            .flat_map(|&(distance, intensity)| [encode(distance, 3), encode(intensity, 3)])
            .flatten()
            .collect();
        for chunk in payload.chunks(64) {
            block.extend(line(chunk));
        }
        block.push(b'\n');
        block
    }

    /// A UTM-30LX-EW's `PP` reply.
    pub fn parameters_block() -> Vec<u8> {
        let mut block = b"PP\n00P\n".to_vec();
        for field in [
            "MODL:UTM-30LX-EW",
            "DMIN:23",
            "DMAX:60000",
            "ARES:1440",
            "AMIN:0",
            "AMAX:1080",
            "AFRT:540",
            "SCAN:2400",
        ] {
            block.extend(field.as_bytes());
            block.push(b';');
            block.push(checksum(field.as_bytes()));
            block.push(b'\n');
        }
        block.push(b'\n');
        block
    }

    pub fn utm_30lx_ew() -> Parameters {
        parse_parameters(&parameters_block()).unwrap()
    }

    #[test]
    fn decodes_the_protocols_own_examples() {
        assert_eq!(decode(b"CB"), 1234);
        assert_eq!(decode(b"1Dh"), 5432);
        assert_eq!(decode(&encode(262_143, 3)), 262_143);
    }

    #[test]
    fn status_checksums_match_the_sensors() {
        assert_eq!(checksum(b"00"), b'P');
        assert_eq!(checksum(b"99"), b'b');
    }

    #[test]
    fn parses_a_utm_30lx_ew_parameters_reply() {
        let parameters = utm_30lx_ew();
        assert_eq!(parameters.model, "UTM-30LX-EW");
        assert_eq!((parameters.dmin_mm, parameters.dmax_mm), (23, 60_000));
        assert_eq!(
            (parameters.amin, parameters.amax, parameters.afrt),
            (0, 1080, 540)
        );
        assert_eq!(parameters.ares, 1440);
        assert_eq!(parameters.rate_hz(), 40.0);
    }

    #[test]
    fn the_full_field_of_view_asks_for_every_step() {
        let layout = Layout::new(&utm_30lx_ew(), 270f32.to_radians(), 1).unwrap();
        assert_eq!((layout.start_step, layout.end_step), (0, 1080));
        assert_eq!(layout.num_points, 1081);
        assert!((layout.fov_rad - 270f32.to_radians()).abs() < 1e-6);
        assert_eq!(layout.command(), "ME0000108001000\n");
    }

    #[test]
    fn a_wider_field_of_view_than_the_sensors_is_capped_to_it() {
        let layout = Layout::new(&utm_30lx_ew(), 360f32.to_radians(), 1).unwrap();
        assert_eq!((layout.start_step, layout.end_step), (0, 1080));
    }

    #[test]
    fn a_narrower_field_of_view_stays_centered_on_the_front() {
        let layout = Layout::new(&utm_30lx_ew(), 180f32.to_radians(), 1).unwrap();
        assert_eq!((layout.start_step, layout.end_step), (180, 900));
        assert_eq!(layout.num_points, 721);
    }

    #[test]
    fn clustered_groups_stay_centered_on_the_front() {
        let layout = Layout::new(&utm_30lx_ew(), 270f32.to_radians(), 3).unwrap();
        // 179 groups either side of the one centered on step 540.
        assert_eq!((layout.start_step, layout.end_step), (2, 1078));
        assert_eq!(layout.num_points, 359);
        assert_eq!(layout.command(), "ME0002107803000\n");
        assert!((layout.fov_rad - (358.0 * 0.75f32).to_radians()).abs() < 1e-5);
        assert!(Layout::new(&utm_30lx_ew(), 1.0, 2).is_err());
    }

    #[test]
    fn a_started_stream_is_acknowledged_with_status_00() {
        assert_eq!(check_stream_started(b"ME0000108001000\n00P\n\n"), Ok(()));
        assert!(check_stream_started(b"ME0000108001000\n0Ee\n\n").is_err());
    }

    #[test]
    fn parses_a_scan_spread_over_several_lines() {
        // 30 readings: 180 characters, over three 64-character lines.
        let readings: Vec<(u32, u32)> = (0..30).map(|i| (1000 + i, 3000 - i)).collect();
        let parsed = parse_scan(&scan_block(123_456, &readings), 30).unwrap();
        assert_eq!(parsed.timestamp_ms, 123_456);
        assert_eq!(
            parsed.distances_mm,
            (0..30).map(|i| 1000 + i).collect::<Vec<_>>()
        );
        assert_eq!(
            parsed.intensities,
            (0..30).map(|i| 3000 - i).collect::<Vec<_>>()
        );
    }

    #[test]
    fn a_corrupted_or_short_scan_is_rejected() {
        let mut block = scan_block(0, &[(1000, 1); 20]);
        let position = block.len() - 10;
        block[position] ^= 1;
        assert!(parse_scan(&block, 20).unwrap_err().contains("checksum"));
        assert!(parse_scan(&scan_block(0, &[(1000, 1); 20]), 21).is_err());
    }
}
