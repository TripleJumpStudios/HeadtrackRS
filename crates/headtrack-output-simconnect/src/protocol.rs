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
/// Legacy `SimConnect_CameraSetRelative6DOF` — deprecated. Sending it on MSFS
/// 2024 SU5 hijacks the menu UI because it bypasses the new acquire/release
/// model. Replaced by `MSG_CAMERA_SET` (`0x62`) below. Kept here only so older
/// SDKs and the deleted code path are documented.
#[allow(dead_code)]
pub const MSG_CAMERA_SET_RELATIVE_6DOF_LEGACY: u32 = 0x30;

// --- SU5 Camera API (wire IDs reverse-engineered from SimConnect_static.lib v1.6.9) ---

/// `SimConnect_CameraAcquire(ClientId)` — must succeed before any `CameraSet`.
pub const MSG_CAMERA_ACQUIRE: u32 = 0x5f;
/// `SimConnect_CameraRelease(CameraDefName)` — release on shutdown / pause.
pub const MSG_CAMERA_RELEASE: u32 = 0x60;
/// `SimConnect_CameraSet(SIMCONNECT_DATA_CAMERA, DataMask)` — the SU5 setter.
pub const MSG_CAMERA_SET: u32 = 0x62;
/// `SimConnect_CameraGet(Referential)` — query the addon camera's current
/// position in the chosen frame.  Used after bootstrap to read the pilot
/// eyepoint coordinates so we can stream pose offsets in absolute body-frame
/// coords without overriding the bootstrap.  Response: `RECV_CAMERA_DATA`.
pub const MSG_CAMERA_GET: u32 = 0x63;
/// `SimConnect_SubscribeToCameraStatusUpdate()` — pushes RECV_CAMERA_STATUS.
pub const MSG_SUBSCRIBE_TO_CAMERA_STATUS_UPDATE: u32 = 0x66;
/// `SimConnect_UnsubscribeToCameraStatusUpdate()` — call on shutdown.
#[allow(dead_code)]
pub const MSG_UNSUBSCRIBE_TO_CAMERA_STATUS_UPDATE: u32 = 0x67;
/// `SimConnect_EnumerateCameraDefinitions()` — request the list of camera
/// definitions defined by the current aircraft.  Response arrives as
/// `RECV_CAMERA_DEFINITION_LIST`.  Empty payload (16-byte packet header only).
pub const MSG_ENUMERATE_CAMERA_DEFINITIONS: u32 = 0x68;
/// `SimConnect_CameraSetUsingCameraDefinition(name)` — seat the addon camera
/// at one of the predefined camera positions returned by enumerate.  Used at
/// connect time to bootstrap from the pilot/cockpit camera, after which we
/// drive head-pose deltas via EYEPOINT-relative `CameraSet`.
pub const MSG_CAMERA_SET_USING_CAMERA_DEFINITION: u32 = 0x69;

// ---------------------------------------------------------------------------
// Sim → Client message IDs
// ---------------------------------------------------------------------------

/// SimConnect exception — returned when a request is invalid or a SimVar is not writable.
/// SIMCONNECT_RECV_ID_EXCEPTION = 1.
pub const RECV_EXCEPTION: u32 = 0x01;
/// Response to MSG_OPEN — contains sim version fields.
/// SIMCONNECT_RECV_ID_OPEN = 2 in the SDK enum (0=NULL, 1=EXCEPTION, 2=OPEN).
pub const RECV_OPEN: u32 = 0x02;
/// Notification of a system event (e.g. Frame tick is returned as EVENT_FRAME which is 0x07).
pub const RECV_EVENT: u32 = 0x07;
/// Notification of a *plain* system event — `SIMCONNECT_RECV_ID_EVENT = 4`.
///
/// Frame ticks arrive as `EVENT_FRAME` (0x07) because they carry the extra
/// `fFrameRate`/`fSimSpeed` floats; every other system event we subscribe to
/// ("Sim", "SimStart", "SimStop") comes back under this ID with the base
/// `SIMCONNECT_RECV_EVENT` layout. Both parse with [`parse_recv_event`].
pub const RECV_EVENT_PLAIN: u32 = 0x04;
/// Response to `MSG_CAMERA_GET` — the addon camera's current state.
/// Position 40 in the SimConnect_RECV_ID enum.  Payload: `SIMCONNECT_DATA_CAMERA`
/// (84-byte struct, same layout as the one we send in `CameraSet`).
pub const RECV_CAMERA_DATA: u32 = 0x28;
/// SU5 Camera API push notification — sent after we
/// `SUBSCRIBE_TO_CAMERA_STATUS_UPDATE` and on every camera-state transition.
/// Position 41 in the SimConnect_RECV_ID enum (counted from `SimConnect.h`).
pub const RECV_CAMERA_STATUS: u32 = 0x29;
/// Response to `MSG_ENUMERATE_CAMERA_DEFINITIONS`.  Position 42 in the
/// SimConnect_RECV_ID enum.  Payload is `SIMCONNECT_RECV_LIST_TEMPLATE` (16-byte
/// header: dwRequestID, dwArraySize, dwEntryNumber, dwOutOf) followed by
/// `dwArraySize × 256-byte name strings` — see `parse_recv_camera_definition_list`.
pub const RECV_CAMERA_DEFINITION_LIST: u32 = 0x2a;

// ---------------------------------------------------------------------------
// SimConnect constants
// ---------------------------------------------------------------------------

// --- SU5 Camera API enums and flags ---

/// `SIMCONNECT_POSITION_REFERENTIAL_*` enum values.
#[allow(dead_code)]
pub const POSITION_REFERENTIAL_SIMOBJECT: u32 = 1;
pub const POSITION_REFERENTIAL_EYEPOINT: u32 = 3;
/// Aircraft-body datum referential.  Per-frame `CameraSet` does not use this —
/// it has a Get/Set asymmetry (sending the value back from `CameraGet` lands
/// the camera in the nose, not at pilot eye, verified 2026-05-06).  Kept here
/// as a documented protocol constant for reference; EYEPOINT is symmetric.
#[allow(dead_code)]
pub const POSITION_REFERENTIAL_SIMOBJECT_DATUM: u32 = 4;

/// `SIMCONNECT_CAMERA_DATA_MASK` flags.
pub const CAMERA_DATA_MASK_POSITION: u32 = 0x01;
pub const CAMERA_DATA_MASK_ROTATION: u32 = 0x02;

/// `SIMCONNECT_CAMERA_AVAILABILITY` enum values.
pub const CAMERA_NOT_ACQUIRED: u32 = 0;
pub const CAMERA_ACQUIRED: u32 = 1;
#[allow(dead_code)]
pub const CAMERA_ACQUIRED_BY_OTHER: u32 = 2;
#[allow(dead_code)]
pub const CAMERA_USER_DISABLED: u32 = 3;

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

/// Parse a RECV_EXCEPTION payload and return the exception code.
///
/// RECV_EXCEPTION layout after recv_packet strips the 12-byte wire header:
///   [0..4]  dwException (u32) — SIMCONNECT_EXCEPTION enum value
///   [4..8]  dwSendID    (u32) — send ID of the packet that caused the exception
///   [8..12] dwIndex     (u32) — parameter index within that packet
pub fn parse_recv_exception(payload: &[u8]) -> Option<u32> {
    if payload.len() < 4 { return None; }
    Some(u32::from_le_bytes(payload[0..4].try_into().ok()?))
}

/// Parse a RECV_EVENT payload and return `(event_id, dw_data)`.
///
/// RECV_EVENT layout after recv_packet strips the 12-byte wire header:
///   [0..4]  dwGroupID
///   [4..8]  uEventID  — the client_event_id we passed in build_subscribe_event
///   [8..12] dwData    — event-specific payload (e.g. 1/0 for the "Sim" event)
pub fn parse_recv_event(payload: &[u8]) -> Option<(u32, u32)> {
    if payload.len() < 12 { return None; }
    let event_id = u32::from_le_bytes(payload[4..8].try_into().ok()?);
    let dw_data  = u32::from_le_bytes(payload[8..12].try_into().ok()?);
    Some((event_id, dw_data))
}

/// Parse the `SIMCONNECT_RECV_EVENT_FRAME` extension of a RECV_EVENT payload.
///
/// Our `RECV_EVENT = 0x07` is `SIMCONNECT_RECV_ID_EVENT_FRAME` in the SDK enum
/// (plain `RECV_ID_EVENT` is 4) — the sim delivers Frame ticks with two extra
/// floats appended after the base event struct:
///   [12..16] fFrameRate — the sim's current visual frame rate (fps)
///   [16..20] fSimSpeed  — simulation rate multiplier (1.0 = real time)
///
/// Returns `None` when the payload is too short to carry them (defensive:
/// non-Frame events routed through this ID stay parseable via
/// [`parse_recv_event`]).
pub fn parse_recv_event_frame_rate(payload: &[u8]) -> Option<(f32, f32)> {
    if payload.len() < 20 { return None; }
    let frame_rate = f32::from_le_bytes(payload[12..16].try_into().ok()?);
    let sim_speed  = f32::from_le_bytes(payload[16..20].try_into().ok()?);
    Some((frame_rate, sim_speed))
}

/// Build a fixed-size 2048-byte null-padded ASCII string payload.
///
/// Used by `MSG_CAMERA_ACQUIRE` and `MSG_CAMERA_RELEASE`. Verified from the
/// SimConnect_CameraAcquire disassembly: total packet size = `0x810` (2064
/// bytes), header = 0x10, payload = 2048 bytes copied with `strncpy_s`-style
/// max length `0x800`.
fn build_string_payload_2048(s: &str) -> Vec<u8> {
    let mut payload = vec![0u8; 2048];
    let bytes = s.as_bytes();
    let n = bytes.len().min(2047);
    payload[..n].copy_from_slice(&bytes[..n]);
    payload
}

/// `SimConnect_CameraAcquire(ClientId)` payload.
pub fn build_camera_acquire(client_id: &str) -> Vec<u8> {
    build_string_payload_2048(client_id)
}

/// `SimConnect_CameraRelease(CameraDefName)` payload. Name is allowed to be empty.
pub fn build_camera_release(camera_def_name: &str) -> Vec<u8> {
    build_string_payload_2048(camera_def_name)
}

/// `SimConnect_SubscribeToCameraStatusUpdate()` — no parameters.
/// Empty payload; the wire ID alone is the request.
pub fn build_subscribe_camera_status() -> Vec<u8> {
    Vec::new()
}

/// `SimConnect_CameraGet(Referential)` payload — single 4-byte DWORD with the
/// referential we want the camera's current position reported in.
pub fn build_camera_get(referential: u32) -> Vec<u8> {
    referential.to_le_bytes().to_vec()
}

/// Parse a `RECV_CAMERA_DATA` payload (post-12-byte RECV header strip).
///
/// Returns the position (3 × f64, metres) and PBH (3 × f32, degrees) extracted
/// from the embedded 84-byte `SIMCONNECT_DATA_CAMERA` struct.
pub fn parse_recv_camera_data(payload: &[u8]) -> Option<([f64; 3], [f32; 3])> {
    if payload.len() < 84 { return None; }
    let mut pos = [0f64; 3];
    for (i, slot) in pos.iter_mut().enumerate() {
        let o = i * 8;
        *slot = f64::from_le_bytes(payload[o..o + 8].try_into().ok()?);
    }
    let mut pbh = [0f32; 3];
    for (i, slot) in pbh.iter_mut().enumerate() {
        let o = 56 + i * 4;
        *slot = f32::from_le_bytes(payload[o..o + 4].try_into().ok()?);
    }
    Some((pos, pbh))
}

/// `SimConnect_EnumerateCameraDefinitions()` — no parameters.
/// Empty payload (verified: total packet size = 0x10 = 16 bytes = header only).
/// Response arrives as `RECV_CAMERA_DEFINITION_LIST`.
pub fn build_enumerate_camera_definitions() -> Vec<u8> {
    Vec::new()
}

/// `SimConnect_CameraSetUsingCameraDefinition(name)` payload.
/// Same shape as Acquire/Release: 2048-byte null-padded ASCII string.
pub fn build_camera_set_using_camera_definition(name: &str) -> Vec<u8> {
    build_string_payload_2048(name)
}

/// Parse a `RECV_CAMERA_DEFINITION_LIST` payload (post-12-byte RECV header strip).
///
/// Layout (`SIMCONNECT_RECV_LIST_TEMPLATE` + `SIMCONNECT_CAMERA_DEFINITION_ITEM[]`):
/// ```text
/// [0..4]   dwRequestID
/// [4..8]   dwArraySize     — number of items in this packet
/// [8..12]  dwEntryNumber   — packet index when the list is split (0-based)
/// [12..16] dwOutOf         — total number of packets in the split list
/// [16..]   rgData[]        — dwArraySize × 256-byte null-padded ASCII strings
/// ```
///
/// Returns the item names from this packet; the caller is responsible for
/// concatenating across multi-packet responses if `dwOutOf > 1`.
pub fn parse_recv_camera_definition_list(payload: &[u8]) -> Option<Vec<String>> {
    if payload.len() < 16 { return None; }
    let array_size = u32::from_le_bytes(payload[4..8].try_into().ok()?) as usize;
    let mut names = Vec::with_capacity(array_size);
    for i in 0..array_size {
        let start = 16 + i * 256;
        if start + 256 > payload.len() { break; }
        let raw = &payload[start..start + 256];
        let nul = raw.iter().position(|&b| b == 0).unwrap_or(256);
        if let Ok(s) = std::str::from_utf8(&raw[..nul]) {
            names.push(s.to_string());
        }
    }
    Some(names)
}

/// `SimConnect_CameraSet(SIMCONNECT_DATA_CAMERA, DataMask)` payload.
///
/// Layout (88 bytes — verified from CameraSet disassembly: 84-byte struct copy
/// followed by 4-byte DataMask):
/// ```text
/// 0..24   Position           (3 × f64 — XYZ in meters)
/// 24..28  PositionReferential (DWORD)
/// 28..32  PositionReferentialObjectId (DWORD; 0 = user aircraft)
/// 32..56  TargetedPos        (3 × f64 — unused for head tracking, set to 0)
/// 56..68  Pbh                (3 × f32 — Pitch, Bank, Heading in degrees)
/// 68..72  RotationReferential (DWORD)
/// 72..76  RotationReferentialObjectId (DWORD)
/// 76..84  Fov                (f64 — set to SIMCONNECT_CAMERA_IGNORE_FIELD = FLT_MAX)
/// 84..88  DataMask           (DWORD)
/// ```
#[allow(clippy::too_many_arguments)]
pub fn build_camera_set(
    position_xyz: [f64; 3],
    position_ref: u32,
    position_object_id: u32,
    pbh: [f32; 3],
    rotation_ref: u32,
    rotation_object_id: u32,
    fov_or_ignore: f64,
    data_mask: u32,
) -> Vec<u8> {
    let mut payload = vec![0u8; 88];

    // Position (3 × f64)
    for (i, v) in position_xyz.iter().enumerate() {
        let o = i * 8;
        payload[o..o + 8].copy_from_slice(&v.to_le_bytes());
    }
    payload[24..28].copy_from_slice(&position_ref.to_le_bytes());
    payload[28..32].copy_from_slice(&position_object_id.to_le_bytes());

    // TargetedPos stays zeroed (32..56).

    // PBH (3 × f32) starting at 56.
    for (i, v) in pbh.iter().enumerate() {
        let o = 56 + i * 4;
        payload[o..o + 4].copy_from_slice(&v.to_le_bytes());
    }
    payload[68..72].copy_from_slice(&rotation_ref.to_le_bytes());
    payload[72..76].copy_from_slice(&rotation_object_id.to_le_bytes());

    payload[76..84].copy_from_slice(&fov_or_ignore.to_le_bytes());
    payload[84..88].copy_from_slice(&data_mask.to_le_bytes());

    payload
}

/// Parse a `RECV_CAMERA_STATUS` payload (post-12-byte header strip).
///
/// Returns `(acquired_state, game_controlled)`.  `game_controlled` is true when
/// the simulation has temporarily reclaimed the camera (menu, pause, RTC); we
/// must not send `CameraSet` while it is true.
///
/// Layout: `[0..4] DWORD acquiredState`, `[4..8] BOOL bGameControlled`.
pub fn parse_recv_camera_status(payload: &[u8]) -> Option<(u32, bool)> {
    if payload.len() < 8 { return None; }
    let acquired = u32::from_le_bytes(payload[0..4].try_into().ok()?);
    let game_ctl = u32::from_le_bytes(payload[4..8].try_into().ok()?) != 0;
    Some((acquired, game_ctl))
}

// ---------------------------------------------------------------------------
// SimVar read path (aircraft identity for per-aircraft seat offsets)
// ---------------------------------------------------------------------------
// Ported byte-identical from MSFS_Linux_Manager/crates/msfs-simconnect (which
// was itself adapted from this file's framing); layouts verified empirically
// against MSFS 2024 12.2 build 282174.

/// `SimConnect_AddToDataDefinition` — register one SimVar under a define ID.
/// One SimVar per define_id: MSFS 2024 silently breaks a whole bundled group
/// if any one entry is rejected.
pub const MSG_ADD_TO_DATA_DEFINITION: u32 = 0x0C;
/// `SimConnect_RequestDataOnSimObject` — request periodic SimVar delivery.
pub const MSG_REQUEST_DATA_ON_SIM_OBJECT: u32 = 0x0E;
/// Periodic SimVar data (`SIMCONNECT_RECV_ID_SIMOBJECT_DATA`).
pub const RECV_SIMOBJECT_DATA: u32 = 0x08;
/// `SIMCONNECT_PERIOD_VISUAL_FRAME` — deliver every rendered frame (the
/// empirically-verified period; callers dedupe on change).
pub const PERIOD_VISUAL_FRAME: u32 = 4;
/// `SIMCONNECT_DATATYPE_STRING256` — fixed 256-byte null-padded buffer.
pub const DATATYPE_STRING256: u32 = 9;

/// `MSG_ADD_TO_DATA_DEFINITION` payload (528 bytes):
/// ```text
/// [0..4]     define_id
/// [4..260]   datum name  (null-padded ASCII, e.g. "TITLE")
/// [260..516] units name  (null-padded ASCII; "" for string vars)
/// [516..520] data type   (DATATYPE_*)
/// [520..528] epsilon (f32) + datum_id (u32), both zero
/// ```
pub fn build_add_to_data_definition(
    define_id: u32,
    datum_name: &str,
    units_name: &str,
    data_type: u32,
) -> Vec<u8> {
    let mut payload = vec![0u8; 528];
    payload[..4].copy_from_slice(&define_id.to_le_bytes());
    let n = datum_name.as_bytes();
    payload[4..4 + n.len().min(255)].copy_from_slice(&n[..n.len().min(255)]);
    let u = units_name.as_bytes();
    payload[260..260 + u.len().min(255)].copy_from_slice(&u[..u.len().min(255)]);
    payload[516..520].copy_from_slice(&data_type.to_le_bytes());
    payload
}

/// `MSG_REQUEST_DATA_ON_SIM_OBJECT` payload (32 bytes):
/// ```text
/// [0..4]   request_id
/// [4..8]   define_id
/// [8..12]  object_id = 0 (user aircraft)
/// [12..16] period (PERIOD_*)
/// [16..32] flags / origin / interval / limit — zero
/// ```
pub fn build_request_data_on_sim_object(request_id: u32, define_id: u32, period: u32) -> Vec<u8> {
    let mut payload = vec![0u8; 32];
    payload[0..4].copy_from_slice(&request_id.to_le_bytes());
    payload[4..8].copy_from_slice(&define_id.to_le_bytes());
    payload[12..16].copy_from_slice(&period.to_le_bytes());
    payload
}

/// `RECV_SIMOBJECT_DATA` layout after the 12-byte wire header strip:
/// ```text
/// [0..4]   dwRequestID   [4..8]  flags        [8..12]  dwDefineID
/// [12..16] dwObjectID    [16..20] entrynumber [20..24] outof
/// [24..28] dwDefineCount [28..]   data        ← data at 28, NOT 24
/// ```
pub fn parse_simobject_data_request_id(payload: &[u8]) -> Option<u32> {
    if payload.len() < 4 { return None; }
    Some(u32::from_le_bytes(payload[0..4].try_into().ok()?))
}

/// Extract a STRING256 datum: 256-byte null-padded buffer at offset 28..284.
pub fn parse_simobject_data_string256(payload: &[u8]) -> Option<String> {
    if payload.len() < 28 + 256 { return None; }
    let raw = &payload[28..28 + 256];
    let end = raw.iter().position(|&b| b == 0).unwrap_or(raw.len());
    Some(String::from_utf8_lossy(&raw[..end]).trim().to_string())
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

#[cfg(test)]
mod tests {
    use super::*;

    /// These builders write raw bytes into fixed offsets of a struct defined by
    /// a closed-source SDK. Nothing in the type system checks them, and MSFS's
    /// failure mode for a wrong layout is not an error — it is a camera in the
    /// engine bay, or a silently ignored packet. The offsets were established
    /// by reverse-engineering the SDK installer and confirmed in-cockpit; these
    /// tests are what stops an innocent-looking edit from undoing that.
    #[test]
    fn camera_set_payload_layout() {
        let pos = [1.5_f64, -2.25, 3.75];
        let pbh = [10.0_f32, -20.0, 30.0];
        let p = build_camera_set(
            pos,
            POSITION_REFERENTIAL_EYEPOINT,
            0,
            pbh,
            POSITION_REFERENTIAL_EYEPOINT,
            0,
            f64::MAX,
            CAMERA_DATA_MASK_POSITION | CAMERA_DATA_MASK_ROTATION,
        );

        assert_eq!(p.len(), 88, "SIMCONNECT_DATA_CAMERA is 88 bytes");

        for (i, want) in pos.iter().enumerate() {
            let o = i * 8;
            let got = f64::from_le_bytes(p[o..o + 8].try_into().unwrap());
            assert_eq!(got, *want, "position[{i}] at offset {o}");
        }
        assert_eq!(u32::from_le_bytes(p[24..28].try_into().unwrap()), POSITION_REFERENTIAL_EYEPOINT);
        assert_eq!(u32::from_le_bytes(p[28..32].try_into().unwrap()), 0);

        // TargetedPos is unused and must stay zeroed — a stray value here
        // silently retargets the camera.
        assert!(p[32..56].iter().all(|&b| b == 0), "TargetedPos must be zero");

        for (i, want) in pbh.iter().enumerate() {
            let o = 56 + i * 4;
            let got = f32::from_le_bytes(p[o..o + 4].try_into().unwrap());
            assert_eq!(got, *want, "pbh[{i}] at offset {o}");
        }
        assert_eq!(u32::from_le_bytes(p[68..72].try_into().unwrap()), POSITION_REFERENTIAL_EYEPOINT);
        assert_eq!(u32::from_le_bytes(p[72..76].try_into().unwrap()), 0);
        assert_eq!(f64::from_le_bytes(p[76..84].try_into().unwrap()), f64::MAX, "FOV ignore field");
        assert_eq!(u32::from_le_bytes(p[84..88].try_into().unwrap()), 0x03, "POSITION | ROTATION");
    }

    /// Position **and** rotation is the mask that made 6DOF work; the
    /// rotation-only value was a workaround for the SIMOBJECT_DATUM asymmetry
    /// and must not creep back as a "fix".
    #[test]
    fn data_mask_bits_are_stable() {
        assert_eq!(CAMERA_DATA_MASK_POSITION, 0x01);
        assert_eq!(CAMERA_DATA_MASK_ROTATION, 0x02);
        assert_eq!(CAMERA_DATA_MASK_POSITION | CAMERA_DATA_MASK_ROTATION, 0x03);
        assert_eq!(POSITION_REFERENTIAL_EYEPOINT, 3, "EYEPOINT is Get/Set-symmetric");
        assert_eq!(POSITION_REFERENTIAL_SIMOBJECT_DATUM, 4, "the asymmetric one — do not send");
    }

    #[test]
    fn add_to_data_definition_payload_layout() {
        let p = build_add_to_data_definition(21, "TITLE", "", DATATYPE_STRING256);
        assert_eq!(p.len(), 528);
        assert_eq!(u32::from_le_bytes(p[0..4].try_into().unwrap()), 21);
        assert_eq!(&p[4..9], b"TITLE");
        assert_eq!(p[9], 0, "datum name must be null-terminated");
        assert!(p[260..516].iter().all(|&b| b == 0), "empty units must be all-zero");
        assert_eq!(u32::from_le_bytes(p[516..520].try_into().unwrap()), DATATYPE_STRING256);
        assert!(p[520..528].iter().all(|&b| b == 0), "epsilon + datum_id must be zero");
    }

    /// An over-long name must be truncated, not panic on a slice out of range —
    /// the datum name can come from a caller, and a panic here takes the
    /// SimConnect task down mid-flight.
    #[test]
    fn add_to_data_definition_truncates_long_names() {
        let long = "A".repeat(400);
        let p = build_add_to_data_definition(1, &long, &long, DATATYPE_STRING256);
        assert_eq!(p.len(), 528);
        assert_eq!(p[259], 0, "datum name must not run into the units field");
        assert_eq!(p[515], 0, "units name must not run into the data type");
    }

    #[test]
    fn request_data_on_sim_object_payload_layout() {
        let p = build_request_data_on_sim_object(7, 21, 1);
        assert_eq!(p.len(), 32);
        assert_eq!(u32::from_le_bytes(p[0..4].try_into().unwrap()), 7);
        assert_eq!(u32::from_le_bytes(p[4..8].try_into().unwrap()), 21);
        assert_eq!(u32::from_le_bytes(p[8..12].try_into().unwrap()), 0, "object id 0 = user aircraft");
        assert_eq!(u32::from_le_bytes(p[12..16].try_into().unwrap()), 1);
        assert!(p[16..32].iter().all(|&b| b == 0));
    }

    #[test]
    fn camera_status_parses_and_rejects_short_payloads() {
        let mut buf = vec![0u8; 8];
        buf[0..4].copy_from_slice(&1u32.to_le_bytes()); // ACQUIRED
        buf[4..8].copy_from_slice(&1u32.to_le_bytes()); // bGameControlled
        assert_eq!(parse_recv_camera_status(&buf), Some((1, true)));

        buf[4..8].copy_from_slice(&0u32.to_le_bytes());
        assert_eq!(parse_recv_camera_status(&buf), Some((1, false)));

        // A truncated packet must return None rather than indexing past the end.
        assert_eq!(parse_recv_camera_status(&buf[..7]), None);
        assert_eq!(parse_recv_camera_status(&[]), None);
    }

    /// Every packet is `[len][version][0xf0000000 | msg_id][send_id][payload]`.
    /// A wrong length prefix desynchronises the entire stream, not just one
    /// message, and the `0xf0000000` packet-type tag is easy to drop when
    /// adding a new message by copy-paste.
    #[test]
    fn packet_framing_prefixes_length_and_ids() {
        let payload = [0xAAu8, 0xBB, 0xCC, 0xDD];
        let pkt = build_packet(MSG_CAMERA_SET, &payload);
        assert_eq!(pkt.len(), 16 + payload.len());
        assert_eq!(u32::from_le_bytes(pkt[0..4].try_into().unwrap()), pkt.len() as u32);
        assert_eq!(u32::from_le_bytes(pkt[4..8].try_into().unwrap()), SC_VERSION);
        assert_eq!(
            u32::from_le_bytes(pkt[8..12].try_into().unwrap()),
            0xf000_0000 | MSG_CAMERA_SET,
            "msg id must carry the 0xf0000000 packet-type tag"
        );
        assert_eq!(u32::from_le_bytes(pkt[12..16].try_into().unwrap()), 0, "send id");
        assert_eq!(&pkt[16..], &payload);
    }
}
