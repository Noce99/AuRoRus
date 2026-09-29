//! SHA-256 of the files a benchmark ran on, so two runs can be told to have
//! driven exactly the same map and race line whatever their names.

use sha2::{Digest, Sha256};
use std::io::Read;
use std::path::Path;

/// The SHA-256 of `path`'s contents, as 64 lowercase hex digits.
pub fn sha256_file(path: &Path) -> std::io::Result<String> {
    let mut file = std::fs::File::open(path)?;
    let mut hasher = Sha256::new();
    let mut buffer = [0u8; 64 * 1024];
    loop {
        let read = file.read(&mut buffer)?;
        if read == 0 {
            break;
        }
        hasher.update(&buffer[..read]);
    }
    Ok(hasher
        .finalize()
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hashes_match_known_digests() {
        let path =
            std::env::temp_dir().join(format!("aurorus_benchmark_hash_{}", std::process::id()));
        std::fs::write(&path, b"abc").unwrap();
        let hash = sha256_file(&path).unwrap();
        std::fs::remove_file(&path).ok();
        assert_eq!(
            hash,
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
    }
}
