//! Generic TOML config-file loading, shared by every module that reads its
//! tunable parameters from `config/`. Also holds the machinery shared by
//! every *live-tunable* config - applying a wanted parameter value (see
//! [`apply_parameters`]) and saving the values in effect back into a TOML
//! file without disturbing the rest of it (see [`save_toml_values`]) - used
//! by both [`crate::autonomous_control::ParameterTuner`] (one flat file per
//! algorithm) and [`crate::actuators::SimulatedVehicle`] (one `[<kind>]`
//! table per vehicle model, in a single shared file).

use crate::topics::AlgorithmParameter;
use serde::Serialize;
use serde::de::DeserializeOwned;
use std::collections::BTreeMap;
use std::fs;
use std::path::Path;

/// Default root folder every binary looks for its `config/` tree under,
/// relative to the current working directory - overridable per binary via
/// a `--config-dir` flag.
pub const DEFAULT_CONFIG_ROOT: &str = "config";

/// Reads `path` and deserializes it as TOML into `T`.
pub fn load<T: DeserializeOwned>(path: &Path) -> Result<T, String> {
    let text =
        std::fs::read_to_string(path).map_err(|err| format!("failed to read {path:?}: {err}"))?;
    toml::from_str(&text).map_err(|err| format!("failed to parse {path:?}: {err}"))
}

/// Sets every field of `config` named in both `parameters` and `wanted` to
/// its wanted value, sanitized (see [`crate::topics::ParameterKind::sanitize`]).
/// Returns whether anything changed. Requests for a field `parameters`
/// doesn't declare are ignored.
///
/// # Panics
///
/// Panics if `config` doesn't serialize to a JSON object, or a patched
/// config no longer deserializes - a parameter declared with a kind its
/// field can't hold, e.g. a float for a `usize`.
pub fn apply_parameters<C: Serialize + DeserializeOwned>(
    config: &mut C,
    parameters: &[AlgorithmParameter],
    wanted: &BTreeMap<String, f64>,
) -> bool {
    let before = serde_json::to_value(&*config)
        .expect("a tunable config must serialize to JSON to be tunable");
    let mut after = before.clone();
    let fields = after
        .as_object_mut()
        .expect("a tunable config must be a struct");
    for parameter in parameters {
        if let Some(value) = wanted
            .get(&parameter.name)
            .and_then(|&value| parameter.kind.sanitize(value))
        {
            fields.insert(parameter.name.clone(), parameter.kind.to_json(value));
        }
    }
    if after == before {
        return false;
    }
    *config = serde_json::from_value(after).unwrap_or_else(|err| {
        panic!("a tunable parameter's value doesn't fit its config field: {err}")
    });
    true
}

/// Re-reads every one of `parameters`' `value` from the field of the same
/// name in `config`.
///
/// # Panics
///
/// Panics if `config` has no numeric field named after one of
/// `parameters` - a typo in the declaration, caught at startup.
pub fn refresh_parameter_values<C: Serialize>(parameters: &mut [AlgorithmParameter], config: &C) {
    let json =
        serde_json::to_value(config).expect("a tunable config must serialize to JSON to be tunable");
    for parameter in parameters {
        parameter.value = json
            .get(&parameter.name)
            .and_then(serde_json::Value::as_f64)
            .unwrap_or_else(|| {
                panic!(
                    "tunable parameter {:?} has no numeric field of the same name in its config",
                    parameter.name
                )
            });
    }
}

/// `parameter`'s value as TOML: an integer for a
/// [`ParameterKind::Int`](crate::topics::ParameterKind::Int), otherwise a
/// float with at most as many decimals as its step has (at least one, so it
/// stays a TOML float) - `3.3`, not the `3.299999952316284` an `f32` field
/// reads back as - unless the value is genuinely off the step's grid (e.g.
/// a hand-written `0.4189`), which is then kept as is.
pub fn parameter_toml_value(parameter: &AlgorithmParameter) -> String {
    use crate::topics::ParameterKind;
    match parameter.kind {
        ParameterKind::Int { .. } => format!("{}", parameter.value.round() as i64),
        ParameterKind::Float { step, .. } => {
            let decimals = (0..9)
                .find(|&decimals| {
                    let scaled = step * 10f64.powi(decimals);
                    (scaled - scaled.round()).abs() < 1e-6
                })
                .unwrap_or(9)
                .max(1) as usize;
            let on_step = format!("{:.*}", decimals, parameter.value);
            // Off the step's grid by more than float noise (e.g. a hand-written
            // `0.4189` with a 0.01 step): kept exactly, not rounded to it.
            let off_grid = on_step.parse::<f64>().is_ok_and(|rounded| {
                (rounded - parameter.value).abs() > 1e-6 * parameter.value.abs().max(1.0)
            });
            let formatted = if off_grid {
                format!("{:?}", parameter.value)
            } else {
                on_step
            };
            // Trailing zeros trimmed, so an untouched `1.3` doesn't become `1.30`.
            let trimmed = formatted.trim_end_matches('0');
            if trimmed.ends_with('.') {
                format!("{trimmed}0")
            } else {
                trimmed.to_string()
            }
        }
    }
}

/// Writes `values` into `path` - each `(key, value)` replacing a `key =
/// value` line, keeping its indentation, trailing comment, and line ending -
/// and rewrites the file in place (via a temporary file, renamed over it, so
/// a failure never leaves it half-written). `section` restricts which lines
/// are eligible: `None` means only a top-level line (before any `[table]`
/// header), `Some(name)` means only a line inside that `[name]` table.
/// Fails, leaving the file untouched, if a key isn't found in scope or the
/// result would no longer parse as TOML.
pub fn save_toml_values(
    path: &Path,
    section: Option<&str>,
    values: &[(&str, String)],
) -> Result<(), String> {
    let text = fs::read_to_string(path).map_err(|err| format!("failed to read {path:?}: {err}"))?;
    let updated = set_toml_values(&text, section, values).map_err(|err| format!("{path:?}: {err}"))?;
    updated
        .parse::<toml::Table>()
        .map_err(|err| format!("{path:?} would no longer parse, not saved: {err}"))?;
    // Written aside then renamed over, so a failure never leaves it half-written.
    let temporary = path.with_extension("toml.tmp");
    fs::write(&temporary, updated)
        .and_then(|()| fs::rename(&temporary, path))
        .map_err(|err| format!("failed to write {path:?}: {err}"))
}

/// The values `path` holds for `names` - each a top-level key (`section:
/// None`) or a key of the `[section]` table - e.g. to reload what
/// [`save_toml_values`] last wrote. Fails if a key is missing or isn't a
/// number.
pub fn load_toml_values(
    path: &Path,
    section: Option<&str>,
    names: &[&str],
) -> Result<BTreeMap<String, f64>, String> {
    let text = fs::read_to_string(path).map_err(|err| format!("failed to read {path:?}: {err}"))?;
    let root: toml::Table =
        toml::from_str(&text).map_err(|err| format!("failed to parse {path:?}: {err}"))?;
    let table = match section {
        None => &root,
        Some(name) => root
            .get(name)
            .and_then(toml::Value::as_table)
            .ok_or_else(|| format!("{path:?} has no `[{name}]` table"))?,
    };
    names
        .iter()
        .map(|&name| {
            let value = table.get(name).and_then(|value| {
                value.as_float().or_else(|| value.as_integer().map(|int| int as f64))
            });
            match value {
                Some(value) => Ok((name.to_string(), value)),
                None => Err(format!("{path:?} has no numeric `{name}`")),
            }
        })
        .collect()
}

/// `text` (a TOML file) with the value of each top-level (`section: None`)
/// or `[section]`-scoped `key = value` line named in `values` replaced -
/// see [`save_toml_values`]. Only flat numeric values are expected - every
/// tunable config is a flat list of them, possibly inside one table. Fails
/// if a key isn't found in scope.
fn set_toml_values(
    text: &str,
    section: Option<&str>,
    values: &[(&str, String)],
) -> Result<String, String> {
    let mut missing: Vec<&str> = values.iter().map(|&(key, _)| key).collect();
    let mut current_section: Option<String> = None;
    let mut out = String::with_capacity(text.len());
    for line in text.split_inclusive('\n') {
        let trimmed = line.trim();
        if trimmed.starts_with('[') && !trimmed.starts_with("[[") {
            current_section = trimmed
                .trim_start_matches('[')
                .split(']')
                .next()
                .map(|name| name.trim().to_string());
            out.push_str(line);
            continue;
        }
        let in_scope = current_section.as_deref() == section;
        match replace_value(line, values).filter(|_| in_scope) {
            Some((key, replaced)) => {
                missing.retain(|&missing_key| missing_key != key);
                out.push_str(&replaced);
            }
            None => out.push_str(line),
        }
    }
    if !missing.is_empty() {
        let scope = match section {
            None => "top-level".to_string(),
            Some(name) => format!("`[{name}]`"),
        };
        return Err(format!("no {scope} `{}` to set", missing.join("`, `")));
    }
    Ok(out)
}

/// `line` with its value replaced, if it's a `key = value` line for one of
/// `values` - see [`set_toml_values`].
fn replace_value<'a>(line: &str, values: &[(&'a str, String)]) -> Option<(&'a str, String)> {
    let (lhs, rhs) = line.split_once('=')?;
    let (key, value) = values.iter().find(|(key, _)| *key == lhs.trim())?;
    // A number holds no '#', so one after the '=' starts a trailing comment.
    let value_end = rhs
        .find('#')
        .unwrap_or_else(|| rhs.trim_end_matches(['\n', '\r']).len());
    let (old, rest) = rhs.split_at(value_end);
    let spacing = &old[old.trim_end().len()..];
    Some((key, format!("{lhs}= {value}{spacing}{rest}")))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    #[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    struct Config {
        rate_hz: f32,
        radius: usize,
        untouched: f32,
    }

    fn config() -> Config {
        Config {
            rate_hz: 50.0,
            radius: 10,
            untouched: 1.0,
        }
    }

    fn parameters() -> Vec<AlgorithmParameter> {
        vec![
            AlgorithmParameter::float("rate_hz", 5.0, 200.0, 1.0),
            AlgorithmParameter::int("radius", 0, 100, 1),
        ]
    }

    fn wanted(values: &[(&str, f64)]) -> BTreeMap<String, f64> {
        values
            .iter()
            .map(|&(name, value)| (name.to_string(), value))
            .collect()
    }

    #[test]
    fn wanted_values_are_applied_to_their_fields() {
        let mut config = config();
        assert!(apply_parameters(
            &mut config,
            &parameters(),
            &wanted(&[("rate_hz", 20.0), ("radius", 30.0)])
        ));
        assert_eq!(
            config,
            Config {
                rate_hz: 20.0,
                radius: 30,
                untouched: 1.0
            }
        );
    }

    #[test]
    fn wanted_values_are_sanitized() {
        let mut config = config();
        assert!(apply_parameters(
            &mut config,
            &parameters(),
            &wanted(&[("rate_hz", 0.0), ("radius", 12.6)])
        ));
        assert_eq!(
            config,
            Config {
                rate_hz: 5.0,
                radius: 13,
                untouched: 1.0
            }
        );
        assert!(apply_parameters(
            &mut config,
            &parameters(),
            &wanted(&[("radius", -4.0)])
        ));
        assert_eq!(config.radius, 0);
        assert!(!apply_parameters(
            &mut config,
            &parameters(),
            &wanted(&[("rate_hz", f64::NAN)])
        ));
    }

    #[test]
    fn undeclared_or_unchanged_values_change_nothing() {
        let mut config = config();
        assert!(!apply_parameters(
            &mut config,
            &parameters(),
            &wanted(&[("untouched", 7.0), ("nope", 1.0)])
        ));
        assert!(!apply_parameters(
            &mut config,
            &parameters(),
            &wanted(&[("rate_hz", 50.0), ("radius", 10.0)])
        ));
        assert_eq!(config, self::config());
    }

    #[test]
    fn saved_values_keep_the_rest_of_the_file() {
        let text =
            "# Header\n\n# Rate, in Hz.\nrate_hz = 50.0\nradius=100  # points\r\nother = 1\n";
        let values = [
            ("rate_hz", "20.0".to_string()),
            ("radius", "30".to_string()),
        ];
        assert_eq!(
            set_toml_values(text, None, &values).unwrap(),
            "# Header\n\n# Rate, in Hz.\nrate_hz = 20.0\nradius= 30  # points\r\nother = 1\n"
        );
    }

    #[test]
    fn saving_fails_on_a_missing_or_nested_key() {
        let text = "rate_hz = 50.0\n[table]\nradius = 3\n";
        let values = [("rate_hz", "1.0".to_string()), ("radius", "4".to_string())];
        assert_eq!(
            set_toml_values(text, None, &values),
            Err("no top-level `radius` to set".to_string())
        );
    }

    #[test]
    fn only_the_target_section_is_replaced() {
        let text = "[bicycle]\nlf_m = 0.16\nlr_m = 0.16\n\n[two_track]\nlf_m = 0.20\nlr_m = 0.20\n";
        let values = [("lf_m", "0.30".to_string())];
        assert_eq!(
            set_toml_values(text, Some("bicycle"), &values).unwrap(),
            "[bicycle]\nlf_m = 0.30\nlr_m = 0.16\n\n[two_track]\nlf_m = 0.20\nlr_m = 0.20\n"
        );
        assert_eq!(
            set_toml_values(text, Some("two_track"), &values).unwrap(),
            "[bicycle]\nlf_m = 0.16\nlr_m = 0.16\n\n[two_track]\nlf_m = 0.30\nlr_m = 0.20\n"
        );
        assert_eq!(
            set_toml_values(text, Some("nonlinear_bicycle"), &values),
            Err("no `[nonlinear_bicycle]` `lf_m` to set".to_string())
        );
    }

    #[test]
    fn saved_values_load_back() {
        let path = std::env::temp_dir().join(format!("aurorus_load_{}.toml", std::process::id()));
        fs::write(&path, "rate_hz = 50.0\nradius = 3\n[bicycle]\nlf_m = 0.16\n").unwrap();
        let top = load_toml_values(&path, None, &["rate_hz", "radius"]).unwrap();
        assert_eq!(top, wanted(&[("rate_hz", 50.0), ("radius", 3.0)]));
        let table = load_toml_values(&path, Some("bicycle"), &["lf_m"]).unwrap();
        assert_eq!(table, wanted(&[("lf_m", 0.16)]));
        assert!(load_toml_values(&path, None, &["lf_m"]).is_err());
        assert!(load_toml_values(&path, Some("two_track"), &["lf_m"]).is_err());
        fs::remove_file(&path).unwrap();
    }

    #[test]
    fn saved_values_are_formatted_by_kind() {
        let value =
            |parameter: AlgorithmParameter, value| AlgorithmParameter { value, ..parameter };
        let float = AlgorithmParameter::float("t_m", 0.5, 12.0, 0.1);
        assert_eq!(parameter_toml_value(&value(float, 3.299999952316284)), "3.3");
        let whole = AlgorithmParameter::float("rate_hz", 5.0, 200.0, 1.0);
        assert_eq!(parameter_toml_value(&value(whole, 50.0)), "50.0");
        let fine = AlgorithmParameter::float("k", 0.0, 1.0, 0.025);
        assert_eq!(parameter_toml_value(&value(fine, 0.125)), "0.125");
        let hundredths = AlgorithmParameter::float("c", 0.5, 3.0, 0.05);
        assert_eq!(parameter_toml_value(&value(hundredths.clone(), 1.3)), "1.3");
        assert_eq!(parameter_toml_value(&value(hundredths.clone(), 1.25)), "1.25");
        assert_eq!(parameter_toml_value(&value(hundredths.clone(), 2.0)), "2.0");
        assert_eq!(parameter_toml_value(&value(hundredths, 0.4189)), "0.4189");
        let int = AlgorithmParameter::int("radius", 0, 100, 1);
        assert_eq!(parameter_toml_value(&value(int, 43.0)), "43");
    }
}
