//! Reads and writes a closed race line as CSV: one `x,y,speed` row per
//! point, in order, without repeating the first point at the end - the last
//! row's successor is implicitly the first row.

use std::path::Path;

/// One race-line point paired with its target speed.
#[derive(Debug, Clone, Copy)]
pub struct SpeedPoint {
    pub x: f64,
    pub y: f64,
    pub speed_mps: f64,
}

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

/// Error returned by [`read`].
#[derive(Debug)]
pub enum RaceLineReadError {
    Io(std::io::Error),
    Csv(csv::Error),
}

impl std::fmt::Display for RaceLineReadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(err) => write!(f, "failed to read race line CSV: {err}"),
            Self::Csv(err) => write!(f, "failed to read race line CSV: {err}"),
        }
    }
}

impl std::error::Error for RaceLineReadError {}

impl From<std::io::Error> for RaceLineReadError {
    fn from(err: std::io::Error) -> Self {
        Self::Io(err)
    }
}

impl From<csv::Error> for RaceLineReadError {
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

/// Reads back a race line written by [`write`].
pub fn read(path: &Path) -> Result<Vec<SpeedPoint>, RaceLineReadError> {
    let mut reader = csv::Reader::from_path(path)?;
    let mut points = Vec::new();
    for result in reader.deserialize() {
        let row: Row = result?;
        points.push(SpeedPoint { x: row.x, y: row.y, speed_mps: row.speed });
    }
    Ok(points)
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

        let rows = read(&path).unwrap();
        std::fs::remove_file(&path).ok();

        assert_eq!(rows.len(), 2);
        assert_eq!(rows[1].x, 1.0);
        assert_eq!(rows[1].y, 2.0);
        assert_eq!(rows[1].speed_mps, 3.5);
    }
}
