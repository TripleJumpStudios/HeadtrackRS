//! SimConnect TCP wire protocol — packet headers, message IDs, and structs.
//!
//! All integers are little-endian.  Packet framing:
//!   u32 size  |  u32 version  |  u32 msg_id  |  payload bytes
//!
//! Verified against MSFS 2024 (protocol version 0x05).

/// Protocol version sent in every outgoing packet header.
/// There have been exactly 5 SimConnect TCP protocol versions; MSFS 2020/2024 = v5.
pub const SC_VERSION: u32 = 0x05;

// ---------------------------------------------------------------------------
// Client → Sim message IDs
// ---------------------------------------------------------------------------

/// Handshake: identify the client, request RECV_OPEN response.
pub const MSG_OPEN: u32 = 0x01;
/// Subscribe to a named system event (e.g. "Frame").
pub const MSG_SUBSCRIBE_TO_SYSTEM_EVENT: u32 = 0x17;
/// Acquire exclusive control of a camera referential.
pub const MSG_CAMERA_ACQUIRE: u32 = 0x31;
/// Release the camera referential (sent on clean shutdown).
pub const MSG_CAMERA_RELEASE: u32 = 0x32;
/// Set the camera position + orientation.
pub const MSG_CAMERA_SET: u32 = 0x30;

// ---------------------------------------------------------------------------
// Sim → Client message IDs
// ---------------------------------------------------------------------------

/// Response to MSG_OPEN — contains sim version fields.
/// SIMCONNECT_RECV_ID_OPEN = 2 in the SDK enum (0=NULL, 1=EXCEPTION, 2=OPEN).
pub const RECV_OPEN: u32 = 0x02;
/// Notification of a system event (e.g. Frame tick is returned as EVENT_FRAME which is 0x07).
pub const RECV_EVENT: u32 = 0x07;
/// Response to MSG_CAMERA_ACQUIRE — indicates success/failure.
pub const RECV_CAMERA_6DOF: u32 = 0x29;

// ---------------------------------------------------------------------------
// Camera referentials
// ---------------------------------------------------------------------------

/// EYEPOINT referential — purpose-built for head tracking (MSFS 2024 SU5+).
pub const REFERENTIAL_EYEPOINT: u32 = 2;

// ---------------------------------------------------------------------------
// Packet building helpers
// ---------------------------------------------------------------------------

/// Build a complete SimConnect packet: size-prefixed header + payload.
pub fn build_packet(msg_id: u32, payload: &[u8]) -> Vec<u8> {
    // Total size includes the 16-byte header (size + version + msg_id + send_id).
    let total = 16 + payload.len() as u32;
    let mut pkt = Vec::with_capacity(total as usize);
    pkt.extend_from_slice(&total.to_le_bytes());
    pkt.extend_from_slice(&SC_VERSION.to_le_bytes());
    let packet_type = 0xf0000000 | msg_id;
    pkt.extend_from_slice(&packet_type.to_le_bytes());
    pkt.extend_from_slice(&0u32.to_le_bytes()); // Send ID / internal packet ID = 0
    pkt.extend_from_slice(payload);
    pkt
}

/// Build MSG_OPEN payload.
///
/// Layout (from Node-SimConnect reference):
///   [0..256]   application name (null-padded ASCII)
///   [256..260] zero (u32)
///   [260..261] zero (byte)
///   [261..264] "HK\0" protocol alias
///   [264..268] major version (u32, 11)
///   [268..272] minor version (u32, 0)
///   [272..276] build major (u32, 62651)
///   [276..280] build minor (u32, 3)
pub fn build_open(app_name: &str) -> Vec<u8> {
    let mut payload = vec![0u8; 280];
    let name_bytes = app_name.as_bytes();
    let copy_len = name_bytes.len().min(255);
    payload[..copy_len].copy_from_slice(&name_bytes[..copy_len]);
    
    // protocol alias for KittyHawk/MSFS
    payload[261..264].copy_from_slice(b"HK\0");
    // major version
    payload[264..268].copy_from_slice(&11u32.to_le_bytes());
    // minor version
    payload[268..272].copy_from_slice(&0u32.to_le_bytes());
    // build major
    payload[272..276].copy_from_slice(&62651u32.to_le_bytes());
    // build minor
    payload[276..280].copy_from_slice(&3u32.to_le_bytes());
    
    payload
}

/// Build MSG_SUBSCRIBE_TO_SYSTEM_EVENT payload.
///
/// Layout: u32 client_event_id + null-padded event name (256 bytes).
pub fn build_subscribe_event(client_event_id: u32, event_name: &str) -> Vec<u8> {
    let mut payload = vec![0u8; 260];
    payload[..4].copy_from_slice(&client_event_id.to_le_bytes());
    let name_bytes = event_name.as_bytes();
    let copy_len = name_bytes.len().min(255);
    payload[4..4 + copy_len].copy_from_slice(&name_bytes[..copy_len]);
    payload
}

/// Build MSG_CAMERA_ACQUIRE payload.
///
/// Layout: u32 request_id + u32 referential + null-padded client name (256 bytes).
pub fn build_camera_acquire(request_id: u32, referential: u32, client_name: &str) -> Vec<u8> {
    let mut payload = vec![0u8; 264];
    payload[..4].copy_from_slice(&request_id.to_le_bytes());
    payload[4..8].copy_from_slice(&referential.to_le_bytes());
    let name_bytes = client_name.as_bytes();
    let copy_len = name_bytes.len().min(255);
    payload[8..8 + copy_len].copy_from_slice(&name_bytes[..copy_len]);
    payload
}

/// Build MSG_CAMERA_SET payload (SimConnect_CameraSetRelative6DOF).
///
/// Layout: f32[6] deltaX, deltaY, deltaZ, pitchDeg, bankDeg, headingDeg.
pub fn build_camera_set(
    data: [f32; 6]
) -> Vec<u8> {
    let mut payload = vec![0u8; 24]; // 6 * 4 = 24 bytes
    for (i, &v) in data.iter().enumerate() {
        payload[i * 4..(i + 1) * 4].copy_from_slice(&v.to_le_bytes());
    }
    payload
}

/// Build MSG_CAMERA_RELEASE payload.
///
/// Layout: u32 referential.
pub fn build_camera_release(referential: u32) -> Vec<u8> {
    referential.to_le_bytes().to_vec()
}

// ---------------------------------------------------------------------------
// RECV_OPEN version extraction
// ---------------------------------------------------------------------------

/// Minimum sim version required for the Camera API (MSFS 2024 SU5+).
pub const MIN_VERSION_MAJOR: u32 = 1;
pub const MIN_VERSION_MINOR: u32 = 7;
pub const MIN_VERSION_BUILD: u32 = 9;

/// Parse the application version from a RECV_OPEN payload.
///
/// RECV_OPEN layout (after the 16-byte header is stripped):
///   [0..256]   sim name (null-padded)
///   [256..260] app version major
///   [260..264] app version minor
///   [264..268] app build major
///   [268..272] app build minor
///   (followed by more fields we don't need)
pub fn parse_recv_open_version(payload: &[u8]) -> Option<(u32, u32, u32)> {
    if payload.len() < 272 {
        return None;
    }
    let major = u32::from_le_bytes(payload[256..260].try_into().ok()?);
    let minor = u32::from_le_bytes(payload[260..264].try_into().ok()?);
    let build = u32::from_le_bytes(payload[264..268].try_into().ok()?);
    Some((major, minor, build))
}

/// Returns true if the version from RECV_OPEN is sufficient for the Camera API.
pub fn version_ok(major: u32, minor: u32, build: u32) -> bool {
    if major > MIN_VERSION_MAJOR { return true; }
    if major < MIN_VERSION_MAJOR { return false; }
    if minor > MIN_VERSION_MINOR { return true; }
    if minor < MIN_VERSION_MINOR { return false; }
    build >= MIN_VERSION_BUILD
}
