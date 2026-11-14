//! Independent length-prefixed LAB fixture transport. Not TTP. A future licensed
//! wire codec must implement its own envelope/response matching on this contract.
use thiserror::Error;
pub const MAX_FRAME: usize = 1_048_576;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TransferEnd {
    Success,
    Timeout,
    Disconnected,
    Cancelled,
}
#[derive(Debug, PartialEq, Eq)]
pub enum Outcome {
    Progress,
    Complete(Vec<Vec<u8>>),
    Timeout,
    Closed(TransferEnd),
}
#[derive(Debug, Error, PartialEq, Eq)]
pub enum DecodeError {
    #[error("frame exceeds bounded lab transport limit")]
    Oversize,
    #[error("stream was closed; start a new session")]
    Closed,
}
#[derive(Default)]
pub struct Framer {
    bytes: Vec<u8>,
    closed: bool,
}
impl Framer {
    pub fn feed(&mut self, data: &[u8], end: TransferEnd) -> Result<Outcome, DecodeError> {
        if self.closed {
            return Err(DecodeError::Closed);
        }
        if matches!(end, TransferEnd::Disconnected | TransferEnd::Cancelled) {
            self.bytes.clear();
            self.closed = true;
            return Ok(Outcome::Closed(end));
        }
        // Bound one read as well as retained memory; do not allocate from a wire length.
        if data.len() > MAX_FRAME + 4 || self.bytes.len() + data.len() > 2 * (MAX_FRAME + 4) {
            self.closed = true;
            self.bytes.clear();
            return Err(DecodeError::Oversize);
        }
        self.bytes.extend_from_slice(data); // includes bytes delivered with timeout
        let mut frames = Vec::new();
        let mut used = 0;
        while self.bytes.len() - used >= 4 {
            let n = u32::from_le_bytes(self.bytes[used..used + 4].try_into().unwrap()) as usize;
            if n > MAX_FRAME {
                self.bytes.clear();
                self.closed = true;
                return Err(DecodeError::Oversize);
            }
            if self.bytes.len() - used < 4 + n {
                break;
            }
            frames.push(self.bytes[used + 4..used + 4 + n].to_vec());
            used += 4 + n;
            // Each transfer has a bounded number of records, including empty ones.
            if frames.len() > 4096 {
                self.bytes.clear();
                self.closed = true;
                return Err(DecodeError::Oversize);
            }
        }
        self.bytes.drain(..used);
        if !frames.is_empty() {
            Ok(Outcome::Complete(frames))
        } else if !data.is_empty() {
            Ok(Outcome::Progress)
        } else if end == TransferEnd::Timeout {
            Ok(Outcome::Timeout)
        } else {
            Ok(Outcome::Progress)
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    fn packet(s: &[u8]) -> Vec<u8> {
        [(s.len() as u32).to_le_bytes().as_slice(), s].concat()
    }
    #[test]
    fn every_split_and_timeout_bytes() {
        let p = packet(b"real partial data");
        for split in 1..p.len() {
            let mut f = Framer::default();
            assert_eq!(
                f.feed(&p[..split], TransferEnd::Success),
                Ok(Outcome::Progress)
            );
            assert_eq!(
                f.feed(&p[split..], TransferEnd::Timeout),
                Ok(Outcome::Complete(vec![b"real partial data".to_vec()]))
            );
        }
    }
    #[test]
    fn coalesced_and_empty_timeouts() {
        let mut f = Framer::default();
        assert_eq!(f.feed(&[], TransferEnd::Timeout), Ok(Outcome::Timeout));
        assert_eq!(
            f.feed(&[packet(b"a"), packet(b"b")].concat(), TransferEnd::Success),
            Ok(Outcome::Complete(vec![b"a".to_vec(), b"b".to_vec()]))
        );
    }
    #[test]
    fn oversize_and_teardown() {
        let mut f = Framer::default();
        assert_eq!(
            f.feed(&u32::MAX.to_le_bytes(), TransferEnd::Success),
            Err(DecodeError::Oversize)
        );
        let mut f = Framer::default();
        f.feed(&[3, 0], TransferEnd::Timeout).unwrap();
        assert_eq!(
            f.feed(&[], TransferEnd::Cancelled),
            Ok(Outcome::Closed(TransferEnd::Cancelled))
        );
        assert_eq!(
            f.feed(&[0, 0, 1, 2, 3], TransferEnd::Success),
            Err(DecodeError::Closed)
        );
    }
    #[test]
    fn deterministic_fuzz_smoke() {
        let mut seed = 7u64;
        for len in 0..2048 {
            let mut data = vec![0; len];
            for x in &mut data {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
                *x = (seed >> 32) as u8;
            }
            let _ = Framer::default().feed(&data, TransferEnd::Timeout);
        }
    }
}
