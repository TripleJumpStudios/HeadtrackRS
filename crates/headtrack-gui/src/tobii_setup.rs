//! One-click USB access for the Tobii Eye Tracker 5.
//!
//! Linux gives a freshly plugged USB device to root only, so the tracker
//! reports `PermissionDenied` until a udev rule tags it `uaccess`. This module
//! installs that one rule, and only when the user clicks *Set up USB access*:
//! `pkexec` asks for the password once, a short `sh` script writes
//! [`RULE_PATH`], reloads udev and re-triggers the tracker. Nothing here runs
//! as root otherwise, and tracking never does.
//!
//! The rule text is compiled in from `packaging/udev/`, the same file the
//! AppImage ships, and reaches the root shell on stdin. Root cannot read an
//! AppImage's FUSE mount, and a temp file would be a path someone else could
//! swap, so neither is used. The destination is passed as `$1`, never spliced
//! into the script.

use std::io::Write;
use std::path::Path;
use std::process::{Command, Stdio};
use std::sync::mpsc;

/// The rule, byte for byte as shipped in `packaging/udev/`.
pub const RULE_TEXT: &str = include_str!("../../../packaging/udev/70-headtrackrs-tobii.rules");

/// Where the click installs it. Removing this file undoes the setup.
pub const RULE_PATH: &str = "/etc/udev/rules.d/70-headtrackrs-tobii.rules";

/// Runs as root under pkexec, with the rule on stdin and the destination in
/// `$1`. Write-then-rename so udev never reads a half-written rule.
const INSTALL_SCRIPT: &str = "set -e
umask 022
dst=\"$1\"
tmp=\"$dst.headtrackrs-new\"
cat > \"$tmp\"
chmod 0644 \"$tmp\"
mv -f \"$tmp\" \"$dst\"
udevadm control --reload-rules
udevadm trigger --action=change --subsystem-match=usb --attr-match=idVendor=2104 --attr-match=idProduct=0313
";

/// `$0` of the root shell; shows up in pkexec's prompt and in `ps`.
const SCRIPT_NAME: &str = "headtrackrs-usb-setup";

/// A fully built command: what to run, its arguments, and what to feed it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Invocation {
    pub program: &'static str,
    pub args: Vec<String>,
    pub stdin: &'static str,
}

/// The one command the click runs.
pub fn install_invocation() -> Invocation {
    Invocation {
        program: "pkexec",
        args: vec![
            "/bin/sh".into(),
            "-c".into(),
            INSTALL_SCRIPT.into(),
            SCRIPT_NAME.into(),
            RULE_PATH.into(),
        ],
        stdin: RULE_TEXT,
    }
}

/// Whether the rule file is already in place (a plain stat, no privileges).
pub fn rule_installed() -> bool {
    Path::new(RULE_PATH).exists()
}

/// Where the setup is, as the Tobii tab shows it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SetupState {
    Idle,
    /// pkexec is up, most likely waiting on the password dialog.
    Running,
    Done,
    Failed(String),
}

/// Runs the install on a background thread so the password dialog never
/// stalls the UI; [`TobiiSetup::poll`] picks up the result.
pub struct TobiiSetup {
    state: SetupState,
    rx: Option<mpsc::Receiver<Result<(), String>>>,
}

impl Default for TobiiSetup {
    fn default() -> Self {
        Self {
            state: SetupState::Idle,
            rx: None,
        }
    }
}

impl TobiiSetup {
    pub fn state(&self) -> &SetupState {
        &self.state
    }

    /// Start the install. Ignored while one is already running.
    pub fn start(&mut self) {
        if self.state == SetupState::Running {
            return;
        }
        let (tx, rx) = mpsc::channel();
        let inv = install_invocation();
        let spawned = std::thread::Builder::new()
            .name("tobii-usb-setup".into())
            .spawn(move || {
                let _ = tx.send(run(&inv));
            });
        match spawned {
            Ok(_) => {
                self.state = SetupState::Running;
                self.rx = Some(rx);
            }
            Err(e) => self.state = SetupState::Failed(format!("could not start setup: {e}")),
        }
    }

    /// Collect a finished install, if any.
    pub fn poll(&mut self) {
        let Some(rx) = &self.rx else { return };
        match rx.try_recv() {
            Ok(result) => {
                self.state = match result {
                    Ok(()) => SetupState::Done,
                    Err(e) => SetupState::Failed(e),
                };
                self.rx = None;
            }
            Err(mpsc::TryRecvError::Empty) => {}
            Err(mpsc::TryRecvError::Disconnected) => {
                self.state = SetupState::Failed("setup thread ended without a result".into());
                self.rx = None;
            }
        }
    }
}

fn run(inv: &Invocation) -> Result<(), String> {
    let mut child = Command::new(inv.program)
        .args(&inv.args)
        .stdin(Stdio::piped())
        .stdout(Stdio::null())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|e| match e.kind() {
            std::io::ErrorKind::NotFound => format!(
                "pkexec was not found. Install polkit, or copy the rule to {RULE_PATH} by hand."
            ),
            _ => format!("could not run pkexec: {e}"),
        })?;
    if let Some(mut stdin) = child.stdin.take() {
        // A write error here means pkexec exited early (dialog dismissed);
        // its exit status below says why.
        let _ = stdin.write_all(inv.stdin.as_bytes());
    }
    let out = child
        .wait_with_output()
        .map_err(|e| format!("pkexec did not finish: {e}"))?;
    exit_result(out.status.code(), &String::from_utf8_lossy(&out.stderr))
}

/// Map pkexec's exit status to a result. 126: the dialog was dismissed;
/// 127: not authorised (or the program was missing).
fn exit_result(code: Option<i32>, stderr: &str) -> Result<(), String> {
    match code {
        Some(0) => Ok(()),
        Some(126) => Err("Cancelled — the password dialog was closed.".into()),
        Some(127) => Err("Not authorised to install the rule.".into()),
        Some(n) => {
            let detail = stderr.lines().last().unwrap_or("").trim();
            Err(if detail.is_empty() {
                format!("Setup failed (exit {n}).")
            } else {
                format!("Setup failed (exit {n}): {detail}")
            })
        }
        None => Err("Setup was interrupted.".into()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// USB vendor and product of the Tobii Eye Tracker 5.
    const VENDOR_ID: &str = "2104";
    const PRODUCT_ID: &str = "0313";

    fn rule_lines() -> Vec<&'static str> {
        RULE_TEXT
            .lines()
            .map(str::trim)
            .filter(|l| !l.is_empty() && !l.starts_with('#'))
            .collect()
    }

    #[test]
    fn rule_text_is_one_uaccess_match_for_the_tracker() {
        let lines = rule_lines();
        assert_eq!(lines.len(), 1, "exactly one active rule: {lines:?}");
        assert_eq!(
            lines[0],
            "SUBSYSTEM==\"usb\", ENV{DEVTYPE}==\"usb_device\", ATTR{idVendor}==\"2104\", \
             ATTR{idProduct}==\"0313\", TAG+=\"uaccess\""
        );
        // No broad grants: no MODE, GROUP, OWNER or RUN.
        for forbidden in ["MODE", "GROUP", "OWNER", "RUN"] {
            assert!(
                !lines[0].contains(forbidden),
                "rule must not set {forbidden}"
            );
        }
        assert!(RULE_TEXT.ends_with('\n'));
    }

    #[test]
    fn rule_path_matches_the_shipped_file_and_sorts_before_seat_late() {
        let name = Path::new(RULE_PATH).file_name().unwrap().to_str().unwrap();
        assert_eq!(name, "70-headtrackrs-tobii.rules");
        assert!(
            name < "73-seat-late.rules",
            "uaccess is applied by 73-seat-late"
        );
        assert!(RULE_PATH.starts_with("/etc/udev/rules.d/"));
    }

    #[test]
    fn install_command_is_pkexec_sh_with_the_path_as_an_argument() {
        let inv = install_invocation();
        assert_eq!(inv.program, "pkexec");
        assert_eq!(inv.args.len(), 5);
        assert_eq!(inv.args[0], "/bin/sh");
        assert_eq!(inv.args[1], "-c");
        assert_eq!(inv.args[3], SCRIPT_NAME);
        assert_eq!(inv.args[4], RULE_PATH);
        assert_eq!(inv.stdin, RULE_TEXT);

        let script = &inv.args[2];
        assert!(
            !script.contains(RULE_PATH),
            "path goes in $1, not the script"
        );
        assert!(script.starts_with("set -e\n"));
        let steps: Vec<&str> = script.lines().collect();
        let pos = |s: &str| {
            steps
                .iter()
                .position(|l| l.starts_with(s))
                .unwrap_or_else(|| panic!("missing {s}"))
        };
        assert!(pos("cat > \"$tmp\"") < pos("mv -f \"$tmp\" \"$dst\""));
        assert!(pos("mv -f") < pos("udevadm control --reload-rules"));
        assert!(pos("udevadm control") < pos("udevadm trigger"));
    }

    #[test]
    fn trigger_targets_the_same_device_as_the_rule() {
        let script = &install_invocation().args[2];
        let trigger = script
            .lines()
            .find(|l| l.starts_with("udevadm trigger"))
            .unwrap();
        assert!(trigger.contains(&format!("--attr-match=idVendor={VENDOR_ID}")));
        assert!(trigger.contains(&format!("--attr-match=idProduct={PRODUCT_ID}")));
        assert!(trigger.contains("--subsystem-match=usb"));
        assert!(rule_lines()[0].contains(&format!("ATTR{{idVendor}}==\"{VENDOR_ID}\"")));
        assert!(rule_lines()[0].contains(&format!("ATTR{{idProduct}}==\"{PRODUCT_ID}\"")));
    }

    #[test]
    fn exit_codes_map_to_plain_messages() {
        assert_eq!(exit_result(Some(0), ""), Ok(()));
        assert!(exit_result(Some(126), "")
            .unwrap_err()
            .contains("Cancelled"));
        assert!(exit_result(Some(127), "")
            .unwrap_err()
            .contains("Not authorised"));
        assert_eq!(
            exit_result(Some(1), "x\nmv: cannot move\n").unwrap_err(),
            "Setup failed (exit 1): mv: cannot move"
        );
        assert!(exit_result(None, "").is_err());
    }
}
