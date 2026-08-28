//! Writes a closed race line to CSV: one `x,y,speed` row per point, in
//! order, without repeating the first point at the end - the last row's
//! successor is implicitly the first row.

use crate::simulator::environment::dynamics::SpeedPoint;
use std::path::Path;

/// Error returned by [`write`].
#[derive(Debug)]
pub enum RaceLineWriteError {
    Io(std::io::Error),
    Csv(csv::Error),
}

impl std::fmt::Display for RaceLineWriteError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(err) => write!(f, "failed to write race line CSV: {err}"),
            Self::Csv(err) => write!(f, "failed to write race line CSV: {err}"),
        }
    }
}

impl std::error::Error for RaceLineWriteError {}

impl From<std::io::Error> for RaceLineWriteError {
    fn from(err: std::io::Error) -> Self {
        Self::Io(err)
    }
}

impl From<csv::Error> for RaceLineWriteError {
    fn from(err: csv::Error) -> Self {
        Self::Csv(err)
    }
}

#[derive(serde::Serialize, serde::Deserialize)]
struct Row {
    x: f64,
    y: f64,
    speed: f64,
}

/// Writes `points` to `path` as a `x,y,speed` CSV, closed (not repeating
/// the first point).
pub fn write(points: &[SpeedPoint], path: &Path) -> Result<(), RaceLineWriteError> {
    let mut writer = csv::Writer::from_path(path)?;
    for point in points {
        writer.serialize(Row { x: point.x, y: point.y, speed: point.speed_mps })?;
    }
    writer.flush()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn written_csv_round_trips() {
        let points = vec![
            SpeedPoint { x: 0.0, y: 0.0, speed_mps: 1.0 },
            SpeedPoint { x: 1.0, y: 2.0, speed_mps: 3.5 },
        ];
        let path = std::env::temp_dir().join(format!("aurorus_race_line_test_{}.csv", std::process::id()));
        write(&points, &path).unwrap();

        let mut reader = csv::Reader::from_path(&path).unwrap();
        let rows: Vec<Row> = reader.deserialize().map(|r| r.unwrap()).collect();
        std::fs::remove_file(&path).ok();

        assert_eq!(rows.len(), 2);
        assert_eq!(rows[1].x, 1.0);
        assert_eq!(rows[1].y, 2.0);
        assert_eq!(rows[1].speed, 3.5);
    }
}
