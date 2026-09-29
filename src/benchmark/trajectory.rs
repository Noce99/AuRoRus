//! A run's `trajectory.csv`: the vehicle sampled at a fixed rate from the
//! "Go!" to the end of the run - enough to replay it without simulating.

use std::fs::File;
use std::path::Path;
use std::time::{Duration, Instant};

/// How often [`TrajectoryWriter`] flushes to disk, so a crash still leaves
/// all but the last moments readable.
const FLUSH_INTERVAL: Duration = Duration::from_secs(1);

/// One sample. Poses are in the map's world frame (see
/// [`crate::environment::MapInfo`]).
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct TrajectoryRow {
    /// Seconds since the "Go!".
    pub t_s: f64,
    /// The lap in progress, counted from `1` - `0` before the vehicle first
    /// crosses the start/finish line.
    pub lap: u32,
    /// The simulator's ground truth.
    pub x_m: f64,
    pub y_m: f64,
    pub heading_rad: f64,
    pub speed_mps: f64,
    /// What the algorithm commanded: steering angle, positive toward
    /// increasing heading (see [`crate::topics::VescCommand`]).
    pub steering_cmd_rad: f64,
    /// What the algorithm commanded: forward speed.
    pub speed_cmd_mps: f64,
    /// Meters along the race line from the start/finish line, where the
    /// vehicle projects onto it.
    pub s_m: Option<f64>,
    /// Signed distance from the race line, in meters - positive toward
    /// increasing heading.
    pub lateral_m: Option<f64>,
    /// Localization's estimate of the pose, when it's running.
    pub est_x_m: Option<f64>,
    pub est_y_m: Option<f64>,
    pub est_heading_rad: Option<f64>,
}

/// Appends [`TrajectoryRow`]s to a new `trajectory.csv`.
pub struct TrajectoryWriter {
    writer: csv::Writer<File>,
    last_flush: Instant,
}

impl TrajectoryWriter {
    /// Creates (or truncates) `path`.
    pub fn create(path: &Path) -> Result<Self, csv::Error> {
        Ok(Self {
            writer: csv::Writer::from_path(path)?,
            last_flush: Instant::now(),
        })
    }

    pub fn write(&mut self, row: &TrajectoryRow) -> Result<(), csv::Error> {
        self.writer.serialize(row)?;
        if self.last_flush.elapsed() >= FLUSH_INTERVAL {
            self.flush()?;
        }
        Ok(())
    }

    pub fn flush(&mut self) -> Result<(), csv::Error> {
        self.last_flush = Instant::now();
        self.writer.flush()?;
        Ok(())
    }
}

/// Reads back a `trajectory.csv` written by [`TrajectoryWriter`].
pub fn read_trajectory(path: &Path) -> Result<Vec<TrajectoryRow>, csv::Error> {
    csv::Reader::from_path(path)?.deserialize().collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rows_round_trip_through_csv() {
        let rows = [
            TrajectoryRow {
                t_s: 0.0,
                lap: 0,
                x_m: 1.0,
                y_m: -2.0,
                heading_rad: 0.5,
                speed_mps: 0.0,
                steering_cmd_rad: 0.1,
                speed_cmd_mps: 3.0,
                s_m: Some(41.8),
                lateral_m: Some(-0.05),
                est_x_m: None,
                est_y_m: None,
                est_heading_rad: None,
            },
            TrajectoryRow {
                t_s: 0.025,
                lap: 1,
                x_m: 1.1,
                y_m: -2.0,
                heading_rad: 0.5,
                speed_mps: 0.4,
                steering_cmd_rad: 0.1,
                speed_cmd_mps: 3.0,
                s_m: None,
                lateral_m: None,
                est_x_m: Some(1.09),
                est_y_m: Some(-2.01),
                est_heading_rad: Some(0.49),
            },
        ];
        let path = std::env::temp_dir().join(format!(
            "aurorus_benchmark_trajectory_{}.csv",
            std::process::id()
        ));
        let mut writer = TrajectoryWriter::create(&path).unwrap();
        for row in &rows {
            writer.write(row).unwrap();
        }
        writer.flush().unwrap();
        let read = read_trajectory(&path).unwrap();
        std::fs::remove_file(&path).ok();
        assert_eq!(read, rows);
    }
}
