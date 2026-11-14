//! Header-based CSV interchange, not a translation of upstream decoding code.
//! Files are loaded only on explicit request; no live file tailing or device I/O.
use crate::{Gaze, Validity};
use std::{io::Read, path::Path};
use thiserror::Error;
pub const MAX_CAPTURE_BYTES: usize = 32 * 1024 * 1024;
pub const MAX_RECORDS: usize = 100_000;
#[derive(Clone, Debug)]
pub struct Record {
    pub device_us: u64,
    pub gaze: Gaze,
    pub validity: Validity,
    /// Optional explicitly canonical interchange values. Never inferred from eyes.
    pub canonical_head: Option<[f32; 6]>,
}
#[derive(Debug, Error)]
#[error("capture rejected: {0}")]
pub struct ReplayError(pub String);
pub fn read_bounded(path: &Path, max: usize) -> Result<Vec<u8>, ReplayError> {
    let file = std::fs::File::open(path).map_err(|e| ReplayError(e.to_string()))?;
    if !file
        .metadata()
        .map_err(|e| ReplayError(e.to_string()))?
        .is_file()
    {
        return Err(ReplayError("requires a regular file".into()));
    }
    let mut bytes = Vec::new();
    file.take(max as u64 + 1)
        .read_to_end(&mut bytes)
        .map_err(|e| ReplayError(e.to_string()))?;
    if bytes.len() > max {
        return Err(ReplayError("file size limit exceeded".into()));
    }
    Ok(bytes)
}
pub fn load(path: &Path) -> Result<Vec<Record>, ReplayError> {
    parse(&read_bounded(path, MAX_CAPTURE_BYTES)?)
}
pub fn parse(data: &[u8]) -> Result<Vec<Record>, ReplayError> {
    if data.len() > MAX_CAPTURE_BYTES {
        return Err(ReplayError("file size limit exceeded".into()));
    }
    let mut reader = csv::ReaderBuilder::new().from_reader(data);
    let headers = reader
        .headers()
        .map_err(|e| ReplayError(e.to_string()))?
        .clone();
    let mut names = std::collections::HashSet::new();
    if headers.iter().any(|x| !names.insert(x)) {
        return Err(ReplayError("duplicate column name".into()));
    }
    let index = |name: &str| headers.iter().position(|x| x == name);
    let required = |name: &str| index(name).ok_or_else(|| ReplayError(format!("missing {name}")));
    let ts = required("timestamp_us")?;
    let valid = required("gaze_valid")?;
    let x = required("gaze_x_norm")?;
    let y = required("gaze_y_norm")?;
    let eyes = [index("eye_present_l"), index("eye_present_r")];
    let heads = [
        "head_yaw_deg",
        "head_pitch_deg",
        "head_roll_deg",
        "head_x_mm",
        "head_y_mm",
        "head_z_mm",
    ]
    .map(index);
    if heads.iter().any(Option::is_some) && heads.iter().any(Option::is_none) {
        return Err(ReplayError("incomplete canonical head columns".into()));
    }
    let mut result: Vec<Record> = Vec::new();
    for row in reader.records() {
        let row = row.map_err(|e| ReplayError(e.to_string()))?;
        let bad = || ReplayError(format!("malformed/incomplete record {}", result.len() + 1));
        let field = |i: usize| row.get(i).ok_or_else(bad);
        let number = |i: usize| -> Result<f64, ReplayError> {
            let n: f64 = field(i)?.parse().map_err(|_| bad())?;
            if !n.is_finite() {
                return Err(bad());
            }
            Ok(n)
        };
        let flag = |i: usize| -> Result<bool, ReplayError> {
            match field(i)? {
                "0" => Ok(false),
                "1" => Ok(true),
                _ => Err(bad()),
            }
        };
        let device_us: u64 = field(ts)?.parse().map_err(|_| bad())?;
        if result.last().is_some_and(|r| device_us <= r.device_us) {
            return Err(ReplayError(
                "timestamps must strictly increase within one capture session".into(),
            ));
        }
        let normalized = [number(x)?, number(y)?];
        let validity = if flag(valid)? {
            Validity::Valid
        } else {
            Validity::Invalid
        };
        let mut eyes_present = [None; 2];
        for (i, col) in eyes.iter().enumerate() {
            if let Some(col) = col {
                eyes_present[i] = Some(flag(*col)?);
            }
        }
        let mut canonical_head = None;
        if heads[0].is_some() {
            let mut head = [0f32; 6];
            for (i, col) in heads.iter().enumerate() {
                head[i] = number(col.unwrap())? as f32;
                if !head[i].is_finite() {
                    return Err(bad());
                }
            }
            canonical_head = Some(head);
        }
        result.push(Record {
            device_us,
            gaze: Gaze {
                normalized,
                eyes_present,
            },
            validity,
            canonical_head,
        });
        if result.len() > MAX_RECORDS {
            return Err(ReplayError("record count limit exceeded".into()));
        }
    }
    if result.is_empty() {
        return Err(ReplayError("capture has no records".into()));
    }
    Ok(result)
}
#[cfg(test)]
mod tests {
    use super::*;
    const HEADER: &str = "timestamp_us,gaze_valid,gaze_x_norm,gaze_y_norm\n";
    #[test]
    fn missing_and_truncated_records_are_not_decoded() {
        for s in [
            "1,1,0.2\n",
            "1,2,0.2,0.3\n",
            "1,1,NaN,0.3\n",
            "1,1,0.2,0.3\n1,1,0.4,0.5\n",
        ] {
            assert!(parse(format!("{HEADER}{s}").as_bytes()).is_err());
        }
    }
    #[test]
    fn invalid_clears_valid_and_outside_is_distinct() {
        let r = parse(format!("{HEADER}1,1,1.2,0.5\n2,0,-1,-1\n").as_bytes()).unwrap();
        assert_eq!(r[0].validity, Validity::Valid);
        assert_eq!(r[1].validity, Validity::Invalid);
    }
}
