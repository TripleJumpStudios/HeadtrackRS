#!/usr/bin/env bash
# soak.sh — unattended resource soak for the release candidate (RELEASE_DEVELOPMENT_PLAN L2).
#
# Launches HeadtrackRS, then once per interval records the GUI process's resident memory
# (VmRSS), open file descriptors and thread count to a CSV. At the end (or on Ctrl-C) it
# stops the app and prints a summary. PASS means all three are flat: no steady climb
# across the run.
#
# Usage:
#   scripts/soak.sh [--minutes N] [--interval S] [--bin PATH] [--out DIR] [--proc-name NAME]
#
#   --minutes N     run length (default 120)
#   --interval S    seconds between samples (default 60)
#   --bin PATH      what to launch (default /tmp/htrs/headtrack-rs-0.2.0-x86_64.AppImage,
#                   the copy docs/FINAL_LIVE_PROCEDURE.md makes; a plain binary works too)
#   --out DIR       output directory (default ~/headtrack-dataset/soak-<date>-<time>)
#   --proc-name N   process to sample (default headtrack-gui). An AppImage starts a runtime
#                   that spawns the real binary, so the sampled PID is the newest descendant
#                   with this name, falling back to the launched PID itself.
#
# Runs with whatever head source was last selected (camera.toml). No face is needed: the
# point is that a long-running engine does not leak. Run it with no sim running. It writes
# nothing outside --out. The app's own stdout and stderr go to --out/app.log.
set -euo pipefail

MINUTES=120
INTERVAL=60
BIN=/tmp/htrs/headtrack-rs-0.2.0-x86_64.AppImage
OUT="$HOME/headtrack-dataset/soak-$(date +%F-%H%M%S)"
PROC=headtrack-gui

while [[ $# -gt 0 ]]; do
    case "$1" in
        --minutes)   MINUTES="$2"; shift 2 ;;
        --interval)  INTERVAL="$2"; shift 2 ;;
        --bin)       BIN="$2"; shift 2 ;;
        --out)       OUT="$2"; shift 2 ;;
        --proc-name) PROC="$2"; shift 2 ;;
        -h|--help)   sed -n '2,24p' "$0"; exit 0 ;;
        *) echo "unknown option: $1 (see --help)" >&2; exit 2 ;;
    esac
done

[[ -x "$BIN" ]] || { echo "not executable: $BIN" >&2; exit 1; }
[[ "$MINUTES" =~ ^[0-9]+$ && "$INTERVAL" =~ ^[0-9]+$ && "$INTERVAL" -gt 0 ]] \
    || { echo "--minutes and --interval must be whole numbers (interval > 0)" >&2; exit 2; }
mkdir -p "$OUT"
CSV="$OUT/soak.csv"

# Newest descendant of $1 (any depth) whose command name is $PROC.
find_target() {
    local root=$1 pid kids queue=("$1") found=""
    while [[ ${#queue[@]} -gt 0 ]]; do
        pid=${queue[0]}; queue=("${queue[@]:1}")
        [[ "$(cat /proc/"$pid"/comm 2>/dev/null)" == "$PROC" ]] && found=$pid
        kids=$(cat /proc/"$pid"/task/*/children 2>/dev/null || true)
        for k in $kids; do queue+=("$k"); done
    done
    echo "${found:-}"
}

echo "soak: launching $BIN for $MINUTES min, sampling every $INTERVAL s -> $OUT"
RUST_LOG="${RUST_LOG:-warn}" "$BIN" >"$OUT/app.log" 2>&1 &
LAUNCHED=$!

TARGET=""
for _ in $(seq 1 30); do
    TARGET=$(find_target "$LAUNCHED")
    [[ -n "$TARGET" ]] && break
    kill -0 "$LAUNCHED" 2>/dev/null || { echo "app exited during startup; see $OUT/app.log" >&2; exit 1; }
    sleep 1
done
if [[ -z "$TARGET" ]]; then
    TARGET=$LAUNCHED
    echo "soak: no '$PROC' process found under PID $LAUNCHED; sampling the launched PID"
fi
echo "soak: sampling PID $TARGET ($(cat /proc/"$TARGET"/comm))"

summarise() {
    python3 -I - "$CSV" <<'PY'
import csv, sys
allrows = list(csv.DictReader(open(sys.argv[1])))
rows = [r for r in allrows if r["rss_kb"].isdigit()]
exited = [r for r in allrows if r["rss_kb"] == "EXITED"]
if len(rows) < 2:
    print("soak summary: fewer than 2 samples; nothing to judge"
          + (f" (process exited at {exited[0]['elapsed_s']} s)" if exited else "")); sys.exit()
half = len(rows) // 2
print(f"soak summary: {len(rows)} samples over {rows[-1]['elapsed_s']} s")
# Judge the second half only: the first minutes include model load and allocator warm-up.
verdict = "FLAT"
for key, unit, limit in (("rss_kb", "kB", 0.05), ("fds", "", 0), ("threads", "", 0)):
    v = [int(r[key]) for r in rows]
    late = v[half:]
    growth = late[-1] - late[0]
    rel = growth / late[0] if late[0] else 0
    flag = (rel > limit) if key == "rss_kb" else (growth > limit)
    if flag: verdict = "GROWING"
    print(f"  {key:8s} first {v[0]:>8} last {v[-1]:>8} max {max(v):>8} {unit:2s} | "
          f"second half {late[0]} -> {late[-1]} ({growth:+d}){'  <- grew' if flag else ''}")
if exited:
    verdict = f"FAILED: the process exited at {exited[0]['elapsed_s']} s (see app.log)"
print(f"verdict: {verdict}  (second half: RSS growth <= 5 %, FDs and threads no net growth)")
PY
}

stopped=0
finish() {
    [[ $stopped -eq 1 ]] && return
    stopped=1
    kill "$TARGET" "$LAUNCHED" 2>/dev/null || true
    wait "$LAUNCHED" 2>/dev/null || true
    summarise
}
trap finish EXIT
trap 'exit 130' INT TERM

echo "timestamp,elapsed_s,rss_kb,fds,threads" > "$CSV"
START=$(date +%s)
END=$(( START + MINUTES * 60 ))
while :; do
    now=$(date +%s)
    if ! kill -0 "$TARGET" 2>/dev/null; then
        echo "soak: process $TARGET exited at $(( now - START )) s; see $OUT/app.log" >&2
        echo "$(date -Is),$(( now - START )),EXITED,," >> "$CSV"
        exit 1
    fi
    rss=$(awk '/^VmRSS:/{print $2}' /proc/"$TARGET"/status)
    thr=$(awk '/^Threads:/{print $2}' /proc/"$TARGET"/status)
    fds=$(find /proc/"$TARGET"/fd -mindepth 1 -maxdepth 1 2>/dev/null | wc -l)
    echo "$(date -Is),$(( now - START )),$rss,$fds,$thr" >> "$CSV"
    [[ $now -ge $END ]] && break
    sleep "$INTERVAL"
done
