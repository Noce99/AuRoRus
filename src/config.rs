//! Generic TOML config-file loading, shared by every module that reads its
//! tunable parameters from `config/`.

use serde::de::DeserializeOwned;
use std::path::Path;

/// Default root folder every binary looks for its `config/` tree under,
/// relative to the current working directory - overridable per binary via
/// a `--config-dir` flag.
pub const DEFAULT_CONFIG_ROOT: &str = "config";

/// Reads `path` and deserializes it as TOML into `T`.
pub fn load<T: DeserializeOwned>(path: &Path) -> Result<T, String> {
    let text = std::fs::read_to_string(path).map_err(|err| format!("failed to read {path:?}: {err}"))?;
    toml::from_str(&text).map_err(|err| format!("failed to parse {path:?}: {err}"))
}
