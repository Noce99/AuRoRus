//! Parameter sweeps: one to [`MAX_SWEEP_AXES`] parameters, each over a
//! range, and a run for every combination of their values - the rest of the
//! parameters as in the draft. This module is the pure part: the values of
//! an axis, the grid of combinations, and a sweep's results as CSV. The
//! lab queues and tracks the runs (see `Lab::queue_sweep`).

use aurorus::localization::Scores;
use aurorus::topics::{AlgorithmParameter, ParameterKind};

/// At most this many parameters are swept at once.
pub const MAX_SWEEP_AXES: usize = 3;
/// A sweep is refused past this many combinations: it bounds how long a
/// careless sweep runs, and how much memory its maps take.
pub const MAX_SWEEP_RUNS: usize = 200;
/// Values are rounded to this many significant digits, so `0.1 + 0.2`
/// reads `0.3`.
const SIGNIFICANT_DIGITS: i32 = 10;

/// One swept parameter, as asked for: `count` values evenly spread from
/// `from` to `to`, both included.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct AxisSpec {
    pub name: String,
    pub from: f64,
    pub to: f64,
    pub count: usize,
}

/// A swept parameter with its values - in range, rounded, without
/// duplicates (an integer parameter can have fewer values than asked for).
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct Axis {
    #[serde(flatten)]
    pub spec: AxisSpec,
    pub values: Vec<f64>,
}

/// The values of every axis in `specs`, checked against `parameters` (the
/// tunable ones).
pub fn axes(specs: &[AxisSpec], parameters: &[AlgorithmParameter]) -> Result<Vec<Axis>, String> {
    if specs.is_empty() {
        return Err("pick a parameter to sweep".to_string());
    }
    if specs.len() > MAX_SWEEP_AXES {
        return Err(format!(
            "at most {MAX_SWEEP_AXES} parameters are swept at once"
        ));
    }
    let mut axes: Vec<Axis> = Vec::with_capacity(specs.len());
    for spec in specs {
        if axes.iter().any(|axis| axis.spec.name == spec.name) {
            return Err(format!("{} is swept twice", spec.name));
        }
        let parameter = parameters
            .iter()
            .find(|parameter| parameter.name == spec.name)
            .ok_or_else(|| format!("SLAM has no tunable parameter {:?}", spec.name))?;
        axes.push(Axis {
            values: values(spec, &parameter.kind)?,
            spec: spec.clone(),
        });
    }
    let runs: usize = axes.iter().map(|axis| axis.values.len()).product();
    if runs > MAX_SWEEP_RUNS {
        return Err(format!(
            "that's {runs} runs - at most {MAX_SWEEP_RUNS}: sweep fewer values"
        ));
    }
    Ok(axes)
}

/// `spec`'s values for a parameter of `kind`.
fn values(spec: &AxisSpec, kind: &ParameterKind) -> Result<Vec<f64>, String> {
    if spec.count == 0 {
        return Err(format!("{} needs at least one value", spec.name));
    }
    if spec.count > MAX_SWEEP_RUNS {
        return Err(format!(
            "{} has {} values - at most {MAX_SWEEP_RUNS}",
            spec.name, spec.count
        ));
    }
    let (from, to) = match (kind.sanitize(spec.from), kind.sanitize(spec.to)) {
        (Some(from), Some(to)) => (from, to),
        _ => return Err(format!("{}'s range must be finite numbers", spec.name)),
    };
    let mut values: Vec<f64> = (0..spec.count)
        .map(|i| match spec.count {
            1 => from,
            count => from + (to - from) * i as f64 / (count - 1) as f64,
        })
        .filter_map(|value| kind.sanitize(round_significant(value)))
        .collect();
    values.dedup();
    Ok(values)
}

/// `value` rounded to [`SIGNIFICANT_DIGITS`].
fn round_significant(value: f64) -> f64 {
    if value == 0.0 {
        return 0.0;
    }
    let scale = 10f64.powi(SIGNIFICANT_DIGITS - 1 - value.abs().log10().floor() as i32);
    (value * scale).round() / scale
}

/// Every combination of `axes`' values, as an index into each axis - the
/// last axis changing fastest.
pub fn grid(axes: &[Axis]) -> Vec<Vec<usize>> {
    axes.iter().fold(vec![Vec::new()], |cells, axis| {
        cells
            .iter()
            .flat_map(|cell| {
                (0..axis.values.len()).map(move |i| {
                    let mut cell = cell.clone();
                    cell.push(i);
                    cell
                })
            })
            .collect()
    })
}

/// One row of a sweep's CSV: a combination, and its run's state and
/// scores (if it got that far).
pub struct CsvRow<'a> {
    pub run: Option<u64>,
    pub values: Vec<f64>,
    /// `done`, `failed`, `cancelled`, `skipped`, ...
    pub state: &'a str,
    pub scores: Option<&'a Scores>,
    /// Why it failed or was skipped.
    pub note: Option<&'a str>,
}

/// The CSV columns after the run, the swept values and its state.
const SCORE_COLUMNS: [&str; 14] = [
    "trajectory_rms_m",
    "trajectory_max_m",
    "trajectory_final_m",
    "heading_rms_deg",
    "odometry_rms_m",
    "walls_mean_m",
    "walls_within",
    "wall_thickness",
    "mean_response",
    "min_response",
    "poor_matches",
    "mean_process_ms",
    "max_process_ms",
    "slow_scans",
];

/// A sweep's results as CSV: a header, then one row per combination.
pub fn csv(axes: &[Axis], rows: &[CsvRow]) -> String {
    let mut header = vec!["run".to_string()];
    header.extend(axes.iter().map(|axis| axis.spec.name.clone()));
    header.push("state".to_string());
    header.extend(SCORE_COLUMNS.iter().map(|column| column.to_string()));
    header.push("note".to_string());
    let mut out = header.join(",");
    out.push('\n');
    for row in rows {
        let mut fields = vec![row.run.map(|id| id.to_string()).unwrap_or_default()];
        fields.extend(row.values.iter().map(f64::to_string));
        fields.push(row.state.to_string());
        fields.extend(score_fields(row.scores));
        fields.push(quoted(row.note.unwrap_or_default()));
        out.push_str(&fields.join(","));
        out.push('\n');
    }
    out
}

/// [`SCORE_COLUMNS`]' values in `scores` - empty where there's none.
fn score_fields(scores: Option<&Scores>) -> Vec<String> {
    let Some(s) = scores else {
        return vec![String::new(); SCORE_COLUMNS.len()];
    };
    let number = |value: Option<f64>| value.map(|v| v.to_string()).unwrap_or_default();
    let trajectory = s.trajectory.as_ref();
    vec![
        number(trajectory.map(|t| t.rms_m)),
        number(trajectory.map(|t| t.max_m)),
        number(trajectory.map(|t| t.final_m)),
        number(trajectory.map(|t| t.heading_rms_rad.to_degrees())),
        number(s.odometry.as_ref().map(|t| t.rms_m)),
        number(s.walls.as_ref().map(|w| w.mean_m)),
        number(s.walls.as_ref().map(|w| w.within)),
        number(s.wall_thickness),
        number(s.mean_response),
        number(s.min_response),
        s.poor_matches.to_string(),
        number(s.mean_process_ms),
        number(s.max_process_ms),
        s.slow_scans.to_string(),
    ]
}

/// `text` as one CSV field.
fn quoted(text: &str) -> String {
    if text.contains([',', '"', '\n']) {
        format!("\"{}\"", text.replace('"', "\"\""))
    } else {
        text.to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec(name: &str, from: f64, to: f64, count: usize) -> AxisSpec {
        AxisSpec {
            name: name.to_string(),
            from,
            to,
            count,
        }
    }

    fn parameters() -> Vec<AlgorithmParameter> {
        vec![
            AlgorithmParameter::float("smear", 0.0, 1.0, 0.001),
            AlgorithmParameter::int("count", 0, 10, 1),
        ]
    }

    #[test]
    fn a_range_is_spread_evenly_and_rounded() {
        let axes = axes(&[spec("smear", 0.02, 0.08, 7)], &parameters()).unwrap();
        assert_eq!(
            axes[0].values,
            vec![0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08]
        );
    }

    #[test]
    fn values_are_kept_in_range_and_integers_whole() {
        let axes = axes(
            &[spec("smear", -1.0, 2.0, 4), spec("count", 1.0, 3.0, 5)],
            &parameters(),
        )
        .unwrap();
        // Brought into 0..=1 first, then spread over it.
        assert_eq!(axes[0].values, vec![0.0, 0.3333333333, 0.6666666667, 1.0]);
        // 1, 1.5, 2, 2.5, 3 rounded - each once.
        assert_eq!(axes[1].values, vec![1.0, 2.0, 3.0]);
    }

    #[test]
    fn one_value_is_the_start_of_the_range() {
        let axes = axes(&[spec("smear", 0.3, 0.9, 1)], &parameters()).unwrap();
        assert_eq!(axes[0].values, vec![0.3]);
    }

    #[test]
    fn bad_sweeps_are_refused() {
        let parameters = parameters();
        assert!(axes(&[], &parameters).is_err());
        assert!(axes(&[spec("nope", 0.0, 1.0, 2)], &parameters).is_err());
        assert!(axes(&[spec("smear", 0.0, 1.0, 0)], &parameters).is_err());
        assert!(axes(&[spec("smear", f64::NAN, 1.0, 2)], &parameters).is_err());
        assert!(
            axes(
                &[spec("smear", 0.0, 1.0, 2), spec("smear", 0.0, 1.0, 2)],
                &parameters
            )
            .is_err()
        );
        // 21 x 11 combinations is past the cap.
        assert!(
            axes(
                &[spec("smear", 0.0, 1.0, 21), spec("count", 0.0, 10.0, 11)],
                &parameters
            )
            .is_err()
        );
    }

    #[test]
    fn the_grid_has_every_combination_once() {
        let axes = axes(
            &[spec("smear", 0.1, 0.2, 2), spec("count", 1.0, 3.0, 3)],
            &parameters(),
        )
        .unwrap();
        assert_eq!(
            grid(&axes),
            vec![
                vec![0, 0],
                vec![0, 1],
                vec![0, 2],
                vec![1, 0],
                vec![1, 1],
                vec![1, 2]
            ]
        );
    }

    #[test]
    fn the_csv_has_a_row_per_combination() {
        let axes = axes(&[spec("smear", 0.1, 0.2, 2)], &parameters()).unwrap();
        let rows = [
            CsvRow {
                run: Some(3),
                values: vec![0.1],
                state: "failed",
                scores: None,
                note: Some("bad, \"really\""),
            },
            CsvRow {
                run: None,
                values: vec![0.2],
                state: "skipped",
                scores: None,
                note: None,
            },
        ];
        let csv = csv(&axes, &rows);
        let lines: Vec<_> = csv.lines().collect();
        assert_eq!(lines.len(), 3);
        assert!(lines[0].starts_with("run,smear,state,trajectory_rms_m,"));
        assert!(lines[1].starts_with("3,0.1,failed,"));
        assert!(lines[1].ends_with(",\"bad, \"\"really\"\"\""));
        assert!(lines[2].starts_with(",0.2,skipped,"));
        assert_eq!(lines[0].split(',').count(), lines[2].split(',').count());
    }
}
