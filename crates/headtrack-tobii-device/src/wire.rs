//! Independently constructed codec for the realm-0 wire dialect observed on the
//! user's ET5 on 2026-09-06. See docs/WIRE_SPEC.md for facts and attribution.
//! No authentication, captured initialization blob, or upstream parser is used.
use crate::{image::GrayImage, Gaze, Validity};
use thiserror::Error;
pub const MAX_WIRE_FRAME: usize = 1024 * 1024;
#[derive(Debug, Error, Clone)]
#[error("unsupported or malformed Tobii packet: {0}")]
pub struct WireError(pub &'static str);
#[derive(Debug, Clone)]
pub struct Frame {
    pub kind: u32,
    pub sequence: u32,
    pub direction: u32,
    pub operation: u32,
    pub status: u32,
    pub payload: Vec<u8>,
}
fn be32(b: &[u8]) -> u32 {
    u32::from_be_bytes(b[..4].try_into().unwrap())
}
#[derive(Default)]
pub struct Decoder {
    pending: Vec<u8>,
    poisoned: bool,
}
impl Decoder {
    /// Appends all returned bytes, including bytes from a canceled/timeout read.
    /// A successful incomplete read returns zero frames, not a USB timeout.
    pub fn feed(&mut self, bytes: &[u8]) -> Result<Vec<Frame>, WireError> {
        if self.poisoned {
            return Err(WireError("reset required after framing error"));
        }
        let result = self.append(bytes);
        if result.is_err() {
            self.pending.clear();
            self.poisoned = true;
        }
        result
    }
    fn append(&mut self, bytes: &[u8]) -> Result<Vec<Frame>, WireError> {
        if bytes.len() > 65536 || self.pending.len() + bytes.len() > MAX_WIRE_FRAME + 65536 {
            return Err(WireError("buffer limit"));
        }
        self.pending.extend_from_slice(bytes);
        let mut used = 0;
        let mut frames = Vec::new();
        while self.pending.len() - used >= 8 {
            let b = &self.pending[used..];
            let envelope = u32::from_le_bytes(b[..4].try_into().unwrap());
            let total = u32::from_le_bytes(b[4..8].try_into().unwrap()) as usize;
            if envelope != 1 || !(32..=MAX_WIRE_FRAME).contains(&total) {
                return Err(WireError("unobserved envelope or excessive length"));
            }
            if b.len() < 32 {
                break;
            }
            let header = &b[8..32];
            let payload = be32(&header[20..]) as usize;
            if payload.checked_add(32) != Some(total) {
                return Err(WireError("envelope/payload length mismatch"));
            }
            let kind = be32(header);
            if kind != 0x52 && kind != 0x53 {
                return Err(WireError("unexpected inbound message kind"));
            }
            if b.len() < total {
                break;
            }
            frames.push(Frame {
                kind,
                sequence: be32(&header[4..]),
                direction: be32(&header[8..]),
                operation: be32(&header[12..]),
                status: be32(&header[16..]),
                payload: b[32..total].to_vec(),
            });
            used += total;
            if frames.len() > 2048 {
                return Err(WireError("record limit"));
            }
        }
        self.pending.drain(..used);
        Ok(frames)
    }
}
/// Header encoding from observed wire fields; payloads are built semantically.
pub fn request(sequence: u32, operation: u32, payload: &[u8]) -> Result<Vec<u8>, WireError> {
    if payload.len() > 4096 {
        return Err(WireError("request limit"));
    }
    let mut bytes = Vec::with_capacity(32 + payload.len());
    bytes.extend_from_slice(&0u32.to_le_bytes());
    // Outbound length excludes the envelope; inbound length includes it.
    bytes.extend_from_slice(&(24u32 + payload.len() as u32).to_le_bytes());
    for value in [0x51, sequence, 0, operation, 0, payload.len() as u32] {
        bytes.extend_from_slice(&value.to_be_bytes());
    }
    bytes.extend_from_slice(payload);
    Ok(bytes)
}
pub fn atom(tag: u8, value: &[u8]) -> Vec<u8> {
    let mut b = vec![tag];
    b.extend_from_slice(&(value.len() as u32).to_be_bytes());
    b.extend_from_slice(value);
    b
}
pub fn hello() -> Vec<u8> {
    // Offer only protocol 1.8, which the physical device selected in observation.
    let mut v = 1u32.to_be_bytes().to_vec();
    v.extend_from_slice(&0x00010008u32.to_be_bytes());
    [vec![0, 0], atom(23, &v)].concat()
}
pub fn subscription(stream: u32) -> Vec<u8> {
    [
        vec![0, 0],
        atom(2, &stream.to_be_bytes()),
        atom(23, &0u32.to_be_bytes()),
    ]
    .concat()
}
#[derive(Debug)]
struct Node<'a> {
    tag: u8,
    body: &'a [u8],
    schema: Option<u16>,
    children: Vec<Node<'a>>,
}
fn node<'a>(
    b: &'a [u8],
    cursor: &mut usize,
    depth: usize,
    budget: &mut usize,
) -> Result<Node<'a>, WireError> {
    if depth > 8 || *budget == 0 {
        return Err(WireError("TLV nesting/field limit"));
    }
    *budget -= 1;
    let h = b
        .get(*cursor..cursor.checked_add(5).ok_or(WireError("offset overflow"))?)
        .ok_or(WireError("incomplete TLV header"))?;
    let tag = h[0];
    let length = be32(&h[1..]) as usize;
    *cursor += 5;
    let end = cursor
        .checked_add(length)
        .ok_or(WireError("TLV size overflow"))?;
    let body = b
        .get(*cursor..end)
        .ok_or(WireError("incomplete TLV body"))?;
    *cursor = end;
    let mut children = Vec::new();
    let mut schema = None;
    if tag == 5 {
        if body.len() != 4 {
            return Err(WireError("invalid structure header"));
        }
        let count = u16::from_be_bytes(body[..2].try_into().unwrap()) as usize;
        if count > *budget {
            return Err(WireError("structure element limit"));
        }
        schema = Some(u16::from_be_bytes(body[2..].try_into().unwrap()));
        for _ in 0..count {
            children.push(node(b, cursor, depth + 1, budget)?);
        }
    }
    Ok(Node {
        tag,
        body,
        schema,
        children,
    })
}
fn root(payload: &[u8]) -> Result<Node<'_>, WireError> {
    if payload.len() > MAX_WIRE_FRAME || !payload.starts_with(&[0, 0]) {
        return Err(WireError("unsupported payload prefix"));
    }
    let mut cursor = 2;
    let root = node(payload, &mut cursor, 0, &mut 512)?;
    if cursor != payload.len() {
        return Err(WireError("trailing/incomplete record"));
    }
    Ok(root)
}
fn scalar(n: &Node<'_>) -> Result<u32, WireError> {
    if n.tag != 2 || n.body.len() != 4 {
        return Err(WireError("expected uint32"));
    }
    Ok(be32(n.body))
}
fn boolean(n: &Node<'_>) -> Result<bool, WireError> {
    match scalar(n)? {
        0 => Ok(false),
        1 => Ok(true),
        _ => Err(WireError("unknown validity value")),
    }
}
fn timestamp(n: &Node<'_>) -> Result<u64, WireError> {
    if n.tag != 6 || n.body.len() != 8 {
        return Err(WireError("expected device timestamp"));
    }
    Ok(u64::from_be_bytes(n.body.try_into().unwrap()))
}
fn fields<'a, 'b>(root: &'b Node<'a>) -> Result<Vec<(u32, &'b Node<'a>)>, WireError> {
    if root.schema != Some(3000) {
        return Err(WireError("unknown record schema"));
    }
    let mut result = Vec::new();
    for field in &root.children {
        if field.schema != Some(3001) || field.children.len() != 2 {
            return Err(WireError("incomplete field record"));
        }
        let id = scalar(&field.children[0])?;
        if result.iter().any(|(key, _)| *key == id) {
            return Err(WireError("duplicate field"));
        }
        result.push((id, &field.children[1]));
    }
    Ok(result)
}
fn required<'a, 'b>(fields: &[(u32, &'b Node<'a>)], id: u32) -> Result<&'b Node<'a>, WireError> {
    fields
        .iter()
        .find(|(key, _)| *key == id)
        .map(|(_, v)| *v)
        .ok_or(WireError("required measurement field missing"))
}
fn vector<const N: usize>(node: &Node<'_>) -> Result<[f64; N], WireError> {
    if node.schema != Some(if N == 2 { 8000 } else { 8001 }) || node.children.len() != N {
        return Err(WireError("invalid coordinate vector"));
    }
    let mut out = [0.; N];
    for (i, n) in node.children.iter().enumerate() {
        if n.tag != 4 || n.body.len() != 8 {
            return Err(WireError("expected fixed-point coordinate"));
        }
        out[i] = i64::from_be_bytes(n.body.try_into().unwrap()) as f64 / 4_398_046_511_104.0;
    }
    Ok(out)
}
#[derive(Clone, Debug)]
pub struct NativeGaze {
    pub device_us: u64,
    pub counter: u32,
    pub gaze: Gaze,
    pub validity: Validity,
    pub eye_origins: [Option<[f64; 3]>; 2],
}
pub fn gaze(payload: &[u8]) -> Result<NativeGaze, WireError> {
    let root = root(payload)?;
    let f = fields(&root)?;
    let device_us = timestamp(required(&f, 1)?)?;
    let counter = scalar(required(&f, 20)?)?;
    let valid = boolean(required(&f, 27)?)?;
    let normalized = vector::<2>(required(&f, 28)?)?;
    let mut eyes = [None; 2];
    let mut origins = [None; 2];
    for (i, (presence, origin)) in [(22, 2), (21, 8)].iter().enumerate() {
        if let Some((_, n)) = f.iter().find(|(id, _)| id == presence) {
            eyes[i] = Some(boolean(n)?);
        }
        if eyes[i] == Some(true) {
            if let Some((_, n)) = f.iter().find(|(id, _)| id == origin) {
                origins[i] = Some(vector::<3>(n)?);
            }
        }
    }
    Ok(NativeGaze {
        device_us,
        counter,
        gaze: Gaze {
            normalized,
            eyes_present: eyes,
        },
        validity: if valid {
            Validity::Valid
        } else {
            Validity::Invalid
        },
        eye_origins: origins,
    })
}
pub fn image(payload: &[u8]) -> Result<(u64, GrayImage), WireError> {
    let root = root(payload)?;
    let f = fields(&root)?;
    let ts = timestamp(required(&f, 1)?)?;
    let bpp = scalar(required(&f, 2)?)?;
    let width = scalar(required(&f, 3)?)? as usize;
    let height = scalar(required(&f, 4)?)? as usize;
    let stride = scalar(required(&f, 5)?)? as usize;
    let blob = required(&f, 6)?;
    if bpp != 8 || blob.tag != 21 || blob.body.len() < 4 {
        return Err(WireError("unsupported image format"));
    }
    let n = be32(blob.body) as usize;
    if n != blob.body.len() - 4 {
        return Err(WireError("image byte count mismatch"));
    }
    Ok((
        ts,
        GrayImage::new(width, height, stride, &blob.body[4..])
            .map_err(|_| WireError("invalid image dimensions/raster"))?,
    ))
}
/// Query-realm response observed as three scalar fields, first = realm type.
pub fn realm(payload: &[u8]) -> Result<u32, WireError> {
    if !payload.starts_with(&[0, 0]) {
        return Err(WireError("realm prefix"));
    }
    let mut p = 2;
    let mut values = Vec::new();
    let mut budget = 16;
    while p < payload.len() {
        values.push(scalar(&node(payload, &mut p, 0, &mut budget)?)?);
    }
    if values.len() != 3 {
        return Err(WireError("unknown realm response"));
    }
    Ok(values[0])
}
pub fn negotiated_version(payload: &[u8]) -> Result<u32, WireError> {
    scalar(&root(payload)?)
}
#[cfg(test)]
mod tests {
    use super::*;
    fn response(payload: &[u8]) -> Vec<u8> {
        let mut b = Vec::new();
        b.extend_from_slice(&1u32.to_le_bytes());
        b.extend_from_slice(&(32u32 + payload.len() as u32).to_le_bytes());
        for n in [0x52, 1, 1, 0x3e8, 0, payload.len() as u32] {
            b.extend_from_slice(&n.to_be_bytes());
        }
        b.extend_from_slice(payload);
        b
    }
    fn structure(schema: u16, children: Vec<Vec<u8>>) -> Vec<u8> {
        let mut v = (children.len() as u16).to_be_bytes().to_vec();
        v.extend_from_slice(&schema.to_be_bytes());
        [atom(5, &v), children.concat()].concat()
    }
    fn field(id: u32, value: Vec<u8>) -> Vec<u8> {
        structure(3001, vec![atom(2, &id.to_be_bytes()), value])
    }
    fn gaze_payload() -> Vec<u8> {
        [
            vec![0, 0],
            structure(
                3000,
                vec![
                    field(1, atom(6, &100u64.to_be_bytes())),
                    field(20, atom(2, &7u32.to_be_bytes())),
                    field(27, atom(2, &1u32.to_be_bytes())),
                    field(
                        28,
                        structure(
                            8000,
                            vec![
                                atom(4, &(1i64 << 41).to_be_bytes()),
                                atom(4, &(1i64 << 41).to_be_bytes()),
                            ],
                        ),
                    ),
                ],
            ),
        ]
        .concat()
    }
    #[test]
    fn every_fragment_boundary_and_coalescing() {
        let p = response(&gaze_payload());
        for i in 1..p.len() {
            let mut d = Decoder::default();
            assert!(d.feed(&p[..i]).unwrap().is_empty());
            assert_eq!(d.feed(&p[i..]).unwrap().len(), 1);
        }
        assert_eq!(
            Decoder::default()
                .feed(&[p.clone(), p].concat())
                .unwrap()
                .len(),
            2
        );
    }
    #[test]
    fn rejects_truncated_records_and_unknown_validity() {
        let p = gaze_payload();
        assert_eq!(gaze(&p).unwrap().gaze.normalized, [0.5; 2]);
        for i in 0..p.len() {
            assert!(gaze(&p[..i]).is_err(), "{i}");
        }
        let mut bad = p;
        bad.extend_from_slice(&[0]);
        assert!(gaze(&bad).is_err());
    }
    #[test]
    fn rejects_oversize_before_allocation() {
        let mut p = response(&[]);
        p[4..8].copy_from_slice(&u32::MAX.to_le_bytes());
        assert!(Decoder::default().feed(&p).is_err());
    }
    #[test]
    fn declared_empty_column_is_not_decoded() {
        let p = [vec![0, 0], atom(5, &[0, 1, 0x0b, 0xb8])].concat();
        assert!(gaze(&p).is_err());
    }
    #[test]
    fn overflow_image_is_rejected() {
        let p = [
            vec![0, 0],
            structure(
                3000,
                vec![
                    field(1, atom(6, &1u64.to_be_bytes())),
                    field(2, atom(2, &8u32.to_be_bytes())),
                    field(3, atom(2, &65536u32.to_be_bytes())),
                    field(4, atom(2, &65536u32.to_be_bytes())),
                    field(5, atom(2, &65536u32.to_be_bytes())),
                    field(6, atom(21, &0u32.to_be_bytes())),
                ],
            ),
        ]
        .concat();
        assert!(image(&p).is_err());
    }
}
