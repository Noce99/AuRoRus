//! [`HokuyoLidar`]: the real car's LIDAR, a Hokuyo UTM-30LX-EW (or any Hokuyo
//! speaking SCIP 2.0 over Ethernet), streamed onto the same
//! [`LidarScan`] topic [`super::SimulatedLidar`] fills in simulation. Also
//! draws every hit on its own drawing topic (see [`crate::topics::Drawing`]),
//! from wherever the vehicle is, as well as anything running knows (see
//! [`VehiclePose`]).

mod scip;

use crate::localization::{VehiclePose, WorldPose};
use crate::topics::{Color, Drawing, LidarScan, Shape, VehicleTopics};
use crate::{Captain, Executor};
use scip::{Layout, Parameters, Readings};
use std::any::Any;
use std::io::{BufRead, BufReader, ErrorKind, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::time::{Duration, Instant};

/// How long a single read blocks before checking whether to stop - how
/// quickly [`HokuyoLidar`] notices it's been told to.
const POLL: Duration = Duration::from_millis(50);

/// Every tunable parameter [`HokuyoLidar`] needs - loaded from
/// `config/sensors/hokuyo_lidar.toml` (see [`Default`]) or from an
/// arbitrary path via [`crate::config::load`].
#[derive(Debug, Clone, PartialEq, serde::Deserialize)]
pub struct HokuyoLidarConfig {
    /// The sensor's `host:port`.
    pub address: String,
    /// How long to wait for the sensor to accept a connection, in seconds.
    pub connect_timeout_s: f64,
    /// How long the sensor may go without sending a whole reply before the
    /// connection is considered lost, in seconds.
    pub data_timeout_s: f64,
    /// How long to wait before reconnecting after losing the sensor, in
    /// seconds.
    pub reconnect_delay_s: f64,
    /// The field of view to publish, in radians, centered on the sensor's
    /// front: capped to what the sensor covers.
    pub fov_rad: f32,
    /// How many neighboring steps each reading groups, the sensor reporting
    /// the closest of them; odd, so the middle reading points straight
    /// ahead. `1` keeps every step.
    pub cluster: u32,
    /// Whether to publish the readings in reverse order - for a sensor
    /// mounted so it sends the car's right-most reading first, since a
    /// [`LidarScan`]'s first is the car's left (positive angles are to the
    /// right, as the GUI draws them).
    pub upside_down: bool,
    /// Readings at or below this distance, in meters, are published as
    /// "nothing hit" - see [`LidarScan::min_distance`].
    pub min_distance_m: f32,
    /// Readings at or beyond this distance, in meters, are published as
    /// "nothing hit" - see [`LidarScan::max_distance`].
    pub max_distance_m: f32,
    /// Where the sensor sits on the vehicle, in meters, forward of the
    /// vehicle's reference point - see [`LidarScan::mount_x_m`].
    pub mount_x_m: f32,
    /// Where the sensor sits on the vehicle, in meters, left of the
    /// vehicle's reference point - see [`LidarScan::mount_y_m`].
    pub mount_y_m: f32,
}

impl Default for HokuyoLidarConfig {
    fn default() -> Self {
        toml::from_str(include_str!("../../config/sensors/hokuyo_lidar.toml"))
            .expect("config/sensors/hokuyo_lidar.toml must deserialize into HokuyoLidarConfig")
    }
}

/// The real LIDAR: claims the ego vehicle's [`VehicleTopics::lidar_scan`]
/// and publishes every scan the sensor at [`HokuyoLidarConfig::address`]
/// streams, as fast as it streams them. Losing the sensor is logged and
/// retried every [`HokuyoLidarConfig::reconnect_delay_s`], publishing
/// nothing meanwhile - so readers see the scan go stale rather than a made-up
/// one.
pub struct HokuyoLidar {
    id: u16,
    name: String,
    config: HokuyoLidarConfig,
    vehicle: VehicleTopics,
}

impl HokuyoLidar {
    pub fn new(name: impl Into<String>, config: HokuyoLidarConfig) -> Self {
        Self {
            id: 0,
            name: name.into(),
            config,
            vehicle: VehicleTopics::ego(),
        }
    }

    /// One connection to the sensor, streaming until told to stop (`Ok`) or
    /// the sensor is lost (`Err`).
    fn stream(&self, captain: &Captain) -> Result<(), String> {
        let address = self
            .config
            .address
            .to_socket_addrs()
            .map_err(|err| format!("bad address {:?}: {err}", self.config.address))?
            .next()
            .ok_or_else(|| format!("{:?} resolves to no address", self.config.address))?;
        let socket = TcpStream::connect_timeout(
            &address,
            Duration::from_secs_f64(self.config.connect_timeout_s),
        )
        .map_err(|err| format!("can't connect to {address}: {err}"))?;
        let mut sensor = Sensor::new(socket, Duration::from_secs_f64(self.config.data_timeout_s))?;

        // Stop whatever a previous run left streaming, and drop what it sent.
        sensor.send("QT\n")?;
        sensor.drain()?;
        sensor.send("PP\n")?;
        let Some(parameters) = sensor.reply(captain, self.id)? else {
            return Ok(());
        };
        let parameters = scip::parse_parameters(&parameters)?;
        let layout = Layout::new(&parameters, self.config.fov_rad, self.config.cluster)?;
        sensor.send(&layout.command())?;
        let Some(started) = sensor.reply(captain, self.id)? else {
            return Ok(());
        };
        scip::check_stream_started(&started)?;
        println!(
            "{}: {} at {address} - {} readings over {:.2} deg, {:.0} Hz, {:.2}-{:.2} m",
            self.name,
            parameters.model,
            layout.num_points,
            layout.fov_rad.to_degrees(),
            parameters.rate_hz(),
            f64::from(parameters.dmin_mm) / 1000.0,
            f64::from(parameters.dmax_mm) / 1000.0,
        );

        let result = self.publish_scans(captain, &mut sensor, &parameters, &layout);
        // Best effort: the connection may already be gone.
        let _ = sensor.send("QT\n");
        result
    }

    /// Publishes every scan `sensor` streams, until told to stop or the
    /// sensor is lost.
    fn publish_scans(
        &self,
        captain: &Captain,
        sensor: &mut Sensor,
        parameters: &Parameters,
        layout: &Layout,
    ) -> Result<(), String> {
        let lidar_topic = captain.topic::<LidarScan>(&self.vehicle.lidar_scan());
        // Only for drawing: the lidar streams whether or not anything knows
        // where the vehicle is.
        let mut vehicle_pose = VehiclePose::new(captain, self.vehicle.clone());
        let drawing_topic = captain.drawing(self.id);
        // Three missed scans in a row - but never tighter than the default, so
        // a fast lidar isn't flagged stale by a viewer's own polling jitter.
        let stale_after =
            Drawing::DEFAULT_STALE_AFTER.max(Duration::from_secs_f64(3.0 / parameters.rate_hz()));

        while let Some(block) = sensor.reply(captain, self.id)? {
            let readings = scip::parse_scan(&block, layout.num_points)?;
            let scan = to_scan(&self.config, parameters, layout, &readings);

            let hits = vehicle_pose
                .current()
                .map_or_else(Vec::new, |pose| hits(&scan, pose));
            lidar_topic
                .write(self.id, scan)
                .expect("lost writer authorization for the lidar_scan topic");
            drawing_topic
                .write(
                    self.id,
                    Drawing::default()
                        .element(
                            "Hits",
                            [Shape::Points {
                                points: hits,
                                radius_px: 2.5,
                                color: Color::RED,
                            }],
                            true,
                        )
                        .stale_after(stale_after)
                        .z_index(5),
                )
                .expect("lost writer authorization for the lidar's drawing topic");
        }
        Ok(())
    }
}

impl Executor for HokuyoLidar {
    fn init(&mut self, id: u16) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        let config = self.config.clone();
        captain.claim_writer::<LidarScan>(&self.vehicle.lidar_scan(), self.id, move || {
            LidarScan::new(
                Vec::new(),
                Vec::new(),
                config.min_distance_m,
                config.max_distance_m,
                config.fov_rad,
            )
            .mounted_at(config.mount_x_m, config.mount_y_m)
        });
        captain.claim_drawing(self.id);
    }

    fn run(&mut self, captain: &Captain) {
        let reconnect_delay = Duration::from_secs_f64(self.config.reconnect_delay_s);
        while captain.is_running(self.id) {
            match self.stream(captain) {
                Ok(()) => break,
                Err(err) => eprintln!(
                    "{}: {err} - retrying in {:.1} s",
                    self.name,
                    reconnect_delay.as_secs_f64()
                ),
            }
            let retry_at = Instant::now() + reconnect_delay;
            while captain.is_running(self.id) && Instant::now() < retry_at {
                std::thread::sleep(POLL);
            }
        }
    }

    fn name(&self) -> String {
        self.name.clone()
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn fresh(&self) -> Box<dyn Executor> {
        Box::new(HokuyoLidar::new(self.name.clone(), self.config.clone()))
    }
}

/// The connection to the sensor, reading whole reply blocks.
struct Sensor {
    reader: BufReader<TcpStream>,
    writer: TcpStream,
    /// How long a reply may take to arrive whole.
    data_timeout: Duration,
}

impl Sensor {
    fn new(socket: TcpStream, data_timeout: Duration) -> Result<Self, String> {
        let io = |err: std::io::Error| format!("can't set up the connection: {err}");
        socket.set_read_timeout(Some(POLL)).map_err(io)?;
        socket.set_nodelay(true).map_err(io)?;
        Ok(Self {
            reader: BufReader::new(socket.try_clone().map_err(io)?),
            writer: socket,
            data_timeout,
        })
    }

    fn send(&mut self, command: &str) -> Result<(), String> {
        self.writer
            .write_all(command.as_bytes())
            .map_err(|err| format!("can't send {:?}: {err}", command.trim_end()))
    }

    /// Discards everything the sensor sends until it goes quiet for a poll.
    fn drain(&mut self) -> Result<(), String> {
        let deadline = Instant::now() + self.data_timeout;
        loop {
            match self.reader.fill_buf() {
                Ok([]) => return Err("the sensor closed the connection".into()),
                Ok(buffer) => {
                    let len = buffer.len();
                    self.reader.consume(len);
                }
                Err(err) if is_timeout(&err) => return Ok(()),
                Err(err) => return Err(format!("can't read from the sensor: {err}")),
            }
            if Instant::now() > deadline {
                return Err("the sensor won't stop streaming".into());
            }
        }
    }

    /// The next whole reply block, up to and including its closing empty
    /// line - or `None` once `captain` says to stop.
    fn reply(&mut self, captain: &Captain, id: u16) -> Result<Option<Vec<u8>>, String> {
        let mut block = Vec::new();
        let deadline = Instant::now() + self.data_timeout;
        loop {
            if !captain.is_running(id) {
                return Ok(None);
            }
            // A read cut short by the timeout keeps what it read in `block`.
            match self.reader.read_until(b'\n', &mut block) {
                Ok(0) => return Err("the sensor closed the connection".into()),
                // A stray empty line between replies.
                Ok(_) if block == b"\n" => block.clear(),
                Ok(_) if block.ends_with(b"\n\n") => return Ok(Some(block)),
                Ok(_) => {}
                Err(err) if is_timeout(&err) => {}
                Err(err) => return Err(format!("can't read from the sensor: {err}")),
            }
            if Instant::now() > deadline {
                return Err(format!(
                    "no reply from the sensor in {:.1} s",
                    self.data_timeout.as_secs_f64()
                ));
            }
        }
    }
}

/// Whether `err` is a read giving up after its timeout, which (depending on
/// the platform) is either kind.
fn is_timeout(err: &std::io::Error) -> bool {
    matches!(err.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut)
}

/// `readings` as a [`LidarScan`]: in meters, laid out as `layout` asked
/// for. Anything the sensor flags as an error (outside its own range) or
/// that's outside the configured one reads as "nothing hit" - the maximum
/// distance with no intensity, as [`super::SimulatedLidar`] reports a miss.
/// Reversed if [`HokuyoLidarConfig::upside_down`] - exactly a mirror image,
/// since `layout` is centered on the front.
fn to_scan(
    config: &HokuyoLidarConfig,
    parameters: &Parameters,
    layout: &Layout,
    readings: &Readings,
) -> LidarScan {
    let sensor_range = parameters.dmin_mm..=parameters.dmax_mm;
    let (mut points, mut intensities): (Vec<f32>, Vec<f32>) = readings
        .distances_mm
        .iter()
        .zip(&readings.intensities)
        .map(|(&distance_mm, &intensity)| {
            let distance_m = distance_mm as f32 / 1000.0;
            if sensor_range.contains(&distance_mm)
                && distance_m > config.min_distance_m
                && distance_m < config.max_distance_m
            {
                (distance_m, intensity as f32)
            } else {
                (config.max_distance_m, 0.0)
            }
        })
        .unzip();
    if config.upside_down {
        points.reverse();
        intensities.reverse();
    }
    LidarScan::new(
        points,
        intensities,
        config.min_distance_m,
        config.max_distance_m,
        layout.fov_rad,
    )
    .mounted_at(config.mount_x_m, config.mount_y_m)
}

/// Where every reading of `scan` that hit something is, in the world, with
/// its vehicle at `pose`.
fn hits(scan: &LidarScan, pose: WorldPose) -> Vec<[f32; 2]> {
    let [x_m, y_m, heading_rad] = pose;
    let (x_m, y_m) = scan.origin_m(x_m, y_m, heading_rad);
    scan.points
        .iter()
        .enumerate()
        .filter(|&(_, &distance_m)| distance_m < scan.max_distance)
        .map(|(i, &distance_m)| {
            let angle_rad = heading_rad + f64::from(scan.angle_rad(i));
            let distance_m = f64::from(distance_m);
            [
                (x_m + distance_m * angle_rad.cos()) as f32,
                (y_m + distance_m * angle_rad.sin()) as f32,
            ]
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use scip::tests::utm_30lx_ew;

    #[test]
    fn readings_become_meters_and_errors_read_as_nothing_hit() {
        let config = HokuyoLidarConfig {
            max_distance_m: 10.0,
            mount_x_m: 0.2,
            upside_down: false,
            ..HokuyoLidarConfig::default()
        };
        let parameters = utm_30lx_ew();
        let layout = Layout::new(&parameters, 1.0, 1).unwrap();
        let readings = Readings {
            timestamp_ms: 0,
            // Fine, a sensor error code, past the configured maximum, past
            // the sensor's.
            distances_mm: vec![1500, 3, 12_000, 65_533],
            intensities: vec![900, 5, 800, 7],
        };

        let scan = to_scan(&config, &parameters, &layout, &readings);

        assert_eq!(scan.points, vec![1.5, 10.0, 10.0, 10.0]);
        assert_eq!(scan.intensities, vec![900.0, 0.0, 0.0, 0.0]);
        assert_eq!(scan.fov, layout.fov_rad);
        assert_eq!((scan.mount_x_m, scan.mount_y_m), (0.2, config.mount_y_m));
    }

    #[test]
    fn an_upside_down_sensors_readings_are_mirrored() {
        let config = HokuyoLidarConfig {
            upside_down: true,
            ..HokuyoLidarConfig::default()
        };
        let parameters = utm_30lx_ew();
        let layout = Layout::new(&parameters, 1.0, 1).unwrap();
        let readings = Readings {
            timestamp_ms: 0,
            distances_mm: vec![1000, 2000, 3000],
            intensities: vec![10, 20, 30],
        };

        let scan = to_scan(&config, &parameters, &layout, &readings);

        assert_eq!(scan.points, vec![3.0, 2.0, 1.0]);
        assert_eq!(scan.intensities, vec![30.0, 20.0, 10.0]);
    }

    #[test]
    fn hits_start_from_the_mount_and_skip_misses() {
        // Three readings: left, ahead, right (positive angles are to the
        // right); the right one a miss.
        let scan = LidarScan::new(
            vec![1.0, 2.0, 5.0],
            vec![1.0; 3],
            0.1,
            5.0,
            std::f32::consts::PI,
        )
        .mounted_at(0.5, 0.0);
        let hits = hits(&scan, [1.0, 1.0, 0.0]);

        assert_eq!(hits.len(), 2);
        let close =
            |p: [f32; 2], q: [f32; 2]| (p[0] - q[0]).abs() < 1e-5 && (p[1] - q[1]).abs() < 1e-5;
        assert!(close(hits[0], [1.5, 0.0]), "{:?}", hits[0]);
        assert!(close(hits[1], [3.5, 1.0]), "{:?}", hits[1]);
    }

    #[test]
    fn a_streamed_block_parses_into_a_full_scan() {
        let parameters = utm_30lx_ew();
        let layout = Layout::new(&parameters, 270f32.to_radians(), 1).unwrap();
        let readings: Vec<(u32, u32)> = (0..1081).map(|i| (1000 + i, 500)).collect();
        let block = scip::tests::scan_block(42, &readings);

        let parsed = scip::parse_scan(&block, layout.num_points).unwrap();
        let scan = to_scan(&HokuyoLidarConfig::default(), &parameters, &layout, &parsed);

        assert_eq!(scan.num_lidar_points, 1081);
        assert_eq!(scan.points[540], 1.54);
        // The middle reading points straight ahead, the ends at +-135 deg.
        assert_eq!(scan.angle_rad(540), 0.0);
        assert!((scan.angle_rad(0) + 135f32.to_radians()).abs() < 1e-6);
    }
}
