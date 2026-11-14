//! Exclusive native nusb session. No kernel-driver detach, device reset, firmware,
//! authentication key, or display/calibration write. Only observed realm 0 / 1.8.
use crate::{
    image::GrayImage,
    wire::{self, Decoder, Frame, NativeGaze},
};
use nusb::{
    transfer::{Buffer, Bulk, ControlOut, ControlType, In, Out, Recipient, TransferError},
    Endpoint, Interface, MaybeFuture,
};
use std::{
    sync::atomic::{AtomicBool, Ordering},
    time::{Duration, Instant},
};
use thiserror::Error;
#[derive(Debug, Error)]
pub enum UsbError {
    #[error("Tobii Eye Tracker 5 is not connected")]
    Absent,
    #[error("more than one tracker; select a serial explicitly")]
    Selection,
    #[error("USB {0}: {1}")]
    Access(AccessKind, String),
    #[error("USB transfer: {0}")]
    Transfer(#[from] TransferError),
    #[error(transparent)]
    Wire(#[from] wire::WireError),
    #[error("operation cancelled")]
    Cancelled,
    #[error("USB/protocol response deadline exceeded")]
    Timeout,
    #[error("unsupported device response: {0}")]
    Unsupported(String),
}
/// Why the OS refused or lost the device. Typed so callers (the Tobii tab's
/// state, P05's mapping table) never have to match on message text.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AccessKind {
    /// Another process has claimed interface 0.
    Busy,
    /// The device node is not readable by this user (udev rule missing).
    PermissionDenied,
    /// The device went away or was never enumerable at open time.
    Disconnected,
    Other,
}
impl std::fmt::Display for AccessKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Busy => "busy (another reader owns interface 0)",
            Self::PermissionDenied => "permission denied (udev rule missing)",
            Self::Disconnected => "disconnected",
            Self::Other => "error",
        })
    }
}
/// Coarse classification of a [`UsbError`] for status display.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UsbErrorKind {
    /// No 2104:0313 enumerated.
    NoDevice,
    PermissionDenied,
    Busy,
    /// Was there, went away mid-session (unplug) or a transfer failed.
    Disconnected,
    /// Protocol, timeout, selection, or anything else.
    Other,
}
impl UsbError {
    pub fn retryable(&self) -> bool {
        matches!(
            self,
            Self::Absent | Self::Access(..) | Self::Transfer(_) | Self::Timeout | Self::Wire(_)
        )
    }
    pub fn kind(&self) -> UsbErrorKind {
        match self {
            Self::Absent => UsbErrorKind::NoDevice,
            Self::Access(AccessKind::Busy, _) => UsbErrorKind::Busy,
            Self::Access(AccessKind::PermissionDenied, _) => UsbErrorKind::PermissionDenied,
            Self::Access(AccessKind::Disconnected, _) => UsbErrorKind::Disconnected,
            Self::Transfer(TransferError::Disconnected) => UsbErrorKind::Disconnected,
            _ => UsbErrorKind::Other,
        }
    }
}
fn access_kind(kind: nusb::ErrorKind) -> AccessKind {
    use nusb::ErrorKind::*;
    match kind {
        Busy => AccessKind::Busy,
        PermissionDenied => AccessKind::PermissionDenied,
        Disconnected | NotFound => AccessKind::Disconnected,
        _ => AccessKind::Other,
    }
}
fn access(e: nusb::Error) -> UsbError {
    UsbError::Access(access_kind(e.kind()), e.to_string())
}
#[derive(Default, Clone, Debug)]
pub struct Stats {
    pub transfers: u64,
    pub startup_discarded_bytes: u64,
    pub bytes: u64,
    pub partial_reads: u64,
    pub wait_timeouts: u64,
    pub frames: u64,
    pub malformed: u64,
    pub gaze: u64,
    pub valid_gaze: u64,
    pub images: u64,
}
#[derive(Debug)]
pub enum Event {
    Gaze(NativeGaze),
    Image { device_us: u64, image: GrayImage },
}
pub struct Session {
    input: Endpoint<Bulk, In>,
    output: Endpoint<Bulk, Out>,
    interface: Interface,
    opened: bool,
    decoder: Decoder,
    next_seq: u32,
    pub serial: Option<String>,
    pub bcd_device: u16,
    pub stats: Stats,
    last_gaze: Option<(u64, u32)>,
    last_image: Option<u64>,
    last_progress: Instant,
}
impl Session {
    pub fn open(serial: Option<&str>, cancel: &AtomicBool) -> Result<Self, UsbError> {
        if cancel.load(Ordering::Acquire) {
            return Err(UsbError::Cancelled);
        }
        let mut candidates = nusb::list_devices().wait().map_err(access)?.filter(|d| {
            d.vendor_id() == 0x2104
                && d.product_id() == 0x0313
                && serial.is_none_or(|s| d.serial_number() == Some(s))
        });
        let info = candidates.next().ok_or(UsbError::Absent)?;
        if candidates.next().is_some() {
            return Err(UsbError::Selection);
        }
        let device = info.open().wait().map_err(access)?;
        let interface = device.claim_interface(0).wait().map_err(access)?;
        let input = interface.endpoint::<Bulk, In>(0x83).map_err(access)?;
        let output = interface.endpoint::<Bulk, Out>(0x05).map_err(access)?;
        let mut s = Self {
            input,
            output,
            interface,
            opened: false,
            decoder: Decoder::default(),
            next_seq: 1,
            serial: info.serial_number().map(str::to_owned),
            bcd_device: info.device_version(),
            stats: Stats::default(),
            last_gaze: None,
            last_image: None,
            last_progress: Instant::now(),
        };
        // A prior reader can stop midway through an IR frame. Close the old
        // device session and drain its endpoint before creating the new decoder
        // session. These bytes are deliberately old-session data, not timeouts
        // or active-session parser input.
        s.control(0x42)?;
        s.stats.startup_discarded_bytes = s.drain_old_session(cancel)?;
        s.control(0x41)?;
        s.opened = true;
        let hello = s.exchange(0x3e8, &wire::hello(), cancel)?;
        if wire::negotiated_version(&hello.payload)? != 0x00010008 {
            return Err(UsbError::Unsupported("protocol other than 1.8".into()));
        }
        let realm = s.exchange(0x640, &[0, 0], cancel)?;
        if wire::realm(&realm.payload)? != 0 {
            return Err(UsbError::Unsupported(
                "nonzero realm; authentication is intentionally unavailable".into(),
            ));
        }
        // Realm 0 is anonymous. Attempt subscriptions directly, requiring successful
        // acknowledgments rather than interpreting a failed open-realm as success.
        for stream in [0x500, 0x50e] {
            s.exchange(0x4c4, &wire::subscription(stream), cancel)?;
        }
        s.last_progress = Instant::now();
        Ok(s)
    }
    fn drain_old_session(&mut self, cancel: &AtomicBool) -> Result<u64, UsbError> {
        let deadline = Instant::now() + Duration::from_millis(250);
        let mut discarded = 0;
        loop {
            if cancel.load(Ordering::Acquire) {
                return Err(UsbError::Cancelled);
            }
            if Instant::now() >= deadline || discarded > 2 * 1024 * 1024 {
                return Err(UsbError::Timeout);
            }
            self.input.submit(Buffer::new(16384));
            if let Some(c) = self.input.wait_next_complete(Duration::from_millis(20)) {
                discarded += c.actual_len as u64;
                c.status?;
            } else {
                self.input.cancel_all();
                // Consume completion ownership before the first new-session read.
                let before_cancel = discarded;
                while self.input.pending() > 0 {
                    if Instant::now() >= deadline {
                        return Err(UsbError::Timeout);
                    }
                    if let Some(c) = self.input.wait_next_complete(Duration::from_millis(5)) {
                        discarded += c.actual_len as u64;
                        if !matches!(c.status, Ok(()) | Err(TransferError::Cancelled)) {
                            c.status?;
                        }
                    }
                }
                if discarded > before_cancel {
                    continue;
                }
                return Ok(discarded);
            }
        }
    }
    fn control(&self, request: u8) -> Result<(), UsbError> {
        self.interface
            .control_out(
                ControlOut {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Interface,
                    request,
                    value: 0,
                    index: 0,
                    data: &[],
                },
                Duration::from_millis(100),
            )
            .wait()?;
        Ok(())
    }
    fn send(&mut self, packet: Vec<u8>, cancel: &AtomicBool) -> Result<(), UsbError> {
        let expected = packet.len();
        self.output.submit(Buffer::from(packet));
        let deadline = Instant::now() + Duration::from_millis(250);
        loop {
            if cancel.load(Ordering::Acquire) {
                return Err(UsbError::Cancelled);
            }
            if let Some(completed) = self.output.wait_next_complete(Duration::from_millis(10)) {
                completed.status?;
                if completed.actual_len != expected {
                    return Err(UsbError::Unsupported(
                        "short USB write; session abandoned".into(),
                    ));
                }
                return Ok(());
            }
            if Instant::now() >= deadline {
                return Err(UsbError::Timeout);
            }
        }
    }
    fn receive(&mut self, cancel: &AtomicBool) -> Result<Vec<Frame>, UsbError> {
        if cancel.load(Ordering::Acquire) {
            return Err(UsbError::Cancelled);
        }
        if self.input.pending() == 0 {
            self.input.submit(Buffer::new(16384));
        }
        let Some(completed) = self.input.wait_next_complete(Duration::from_millis(20)) else {
            self.stats.wait_timeouts += 1;
            return Ok(Vec::new());
        };
        self.stats.transfers += 1;
        self.stats.bytes += completed.actual_len as u64;
        // Completion keeps both status and actual_len. Do not call into_result(),
        // which would discard partial bytes on an error.
        let frames = self
            .decoder
            .feed(&completed.buffer[..completed.actual_len])?;
        if completed.actual_len > 0 {
            self.last_progress = Instant::now();
            if frames.is_empty() {
                self.stats.partial_reads += 1;
            }
        }
        self.stats.frames += frames.len() as u64;
        match completed.status {
            Ok(()) | Err(TransferError::Cancelled) if !cancel.load(Ordering::Acquire) => Ok(frames),
            Ok(()) => Err(UsbError::Cancelled),
            Err(e) => Err(e.into()),
        }
    }
    fn exchange(
        &mut self,
        op: u32,
        payload: &[u8],
        cancel: &AtomicBool,
    ) -> Result<Frame, UsbError> {
        let seq = self.next_seq;
        self.next_seq += 1;
        self.send(wire::request(seq, op, payload)?, cancel)?;
        let deadline = Instant::now() + Duration::from_secs(1);
        while Instant::now() < deadline {
            for frame in self.receive(cancel)? {
                if frame.kind == 0x52 && frame.sequence == seq {
                    if frame.operation != op || frame.direction != 1 {
                        return Err(UsbError::Unsupported(
                            "response opcode/direction mismatch".into(),
                        ));
                    }
                    if frame.status != 0 {
                        return Err(UsbError::Unsupported(format!(
                            "operation {op:#06x} status {:#010x}",
                            frame.status
                        )));
                    }
                    return Ok(frame);
                }
                // Notifications arriving before subscription acknowledgment belong
                // to startup, never to a newly published measurement.
            }
        }
        Err(UsbError::Timeout)
    }
    pub fn poll(&mut self, cancel: &AtomicBool) -> Result<Vec<Event>, UsbError> {
        let frames = self.receive(cancel)?;
        let mut events = Vec::new();
        if self.last_progress.elapsed() > Duration::from_millis(500) {
            return Err(UsbError::Timeout);
        }
        for frame in frames {
            if frame.kind != 0x53 {
                continue;
            }
            if frame.status != 0 || frame.direction != 0 {
                self.stats.malformed += 1;
                continue;
            }
            match frame.operation {
                0x500 => match wire::gaze(&frame.payload) {
                    Ok(g) => {
                        if self
                            .last_gaze
                            .is_some_and(|(t, c)| g.device_us <= t || g.counter <= c)
                        {
                            return Err(UsbError::Wire(wire::WireError(
                                "device gaze clock/counter regressed",
                            )));
                        }
                        self.last_gaze = Some((g.device_us, g.counter));
                        self.stats.gaze += 1;
                        if g.validity == crate::Validity::Valid {
                            self.stats.valid_gaze += 1;
                        }
                        events.push(Event::Gaze(g));
                    }
                    Err(_) => self.stats.malformed += 1,
                },
                0x50e => match wire::image(&frame.payload) {
                    Ok((ts, image)) => {
                        if self.last_image.is_some_and(|t| ts <= t) {
                            return Err(UsbError::Wire(wire::WireError(
                                "device image clock regressed",
                            )));
                        }
                        self.last_image = Some(ts);
                        self.stats.images += 1;
                        events.push(Event::Image {
                            device_us: ts,
                            image,
                        });
                    }
                    Err(_) => self.stats.malformed += 1,
                },
                _ => {}
            }
        }
        Ok(events)
    }
}
impl Drop for Session {
    fn drop(&mut self) {
        self.input.cancel_all();
        self.output.cancel_all();
        let deadline = Instant::now() + Duration::from_millis(100);
        while self.input.pending() > 0 && Instant::now() < deadline {
            let _ = self.input.wait_next_complete(Duration::from_millis(5));
        }
        while self.output.pending() > 0 && Instant::now() < deadline {
            let _ = self.output.wait_next_complete(Duration::from_millis(5));
        }
        if self.opened {
            let _ = self.control(0x42);
            self.opened = false;
        }
        // nusb endpoint/interface ownership drops after this, releasing the claim.
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn os_errors_map_to_typed_kinds() {
        let cases = [
            (nusb::ErrorKind::Busy, UsbErrorKind::Busy),
            (nusb::ErrorKind::PermissionDenied, UsbErrorKind::PermissionDenied),
            (nusb::ErrorKind::Disconnected, UsbErrorKind::Disconnected),
            (nusb::ErrorKind::NotFound, UsbErrorKind::Disconnected),
            (nusb::ErrorKind::Other, UsbErrorKind::Other),
        ];
        for (os, want) in cases {
            let e = UsbError::Access(access_kind(os), "x".into());
            assert_eq!(e.kind(), want, "{os:?}");
            assert!(e.retryable());
        }
        assert_eq!(UsbError::Absent.kind(), UsbErrorKind::NoDevice);
        assert_eq!(
            UsbError::Transfer(TransferError::Disconnected).kind(),
            UsbErrorKind::Disconnected
        );
        assert_eq!(UsbError::Timeout.kind(), UsbErrorKind::Other);
        assert_eq!(UsbError::Selection.kind(), UsbErrorKind::Other);
    }
    #[test]
    fn access_messages_keep_their_wording() {
        let e = UsbError::Access(AccessKind::Busy, "Device or resource busy".into());
        assert_eq!(
            e.to_string(),
            "USB busy (another reader owns interface 0): Device or resource busy"
        );
    }
}
