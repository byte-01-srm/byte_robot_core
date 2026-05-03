#!/usr/bin/env python3
"""
keyboard_controller.py
════════════════════════════════════════════════════════════════════════
BYTE-01  Keyboard State Controller  (headless-compatible)

Uses the `keyboard` lib — works over SSH on a headless Pi with no X.
Requires: pip install keyboard requests
On Linux : sudo python3 keyboard_controller.py   (keyboard lib needs root)

Only fires a POST request to the Pi's FastAPI state server when the
state actually changes — no spamming the same state on every loop tick.

Keys:
    W / A / S / D   → FORWARD / LEFT / BACKWARD / RIGHT
    Ctrl (L or R)   → SIT
    Space           → JUMP
    (nothing held)  → STAND
    ESC             → quit

State server:   http://<PI_IP>:8000
                POST /state  { "state": "FORWARD" }
"""

import sys
import time
import threading
import ctypes
import requests
import keyboard     # pip install keyboard  (needs sudo on Linux)


def _enable_ansi_stdout() -> None:
    """Enable ANSI cursor/colors on Windows Conhost (reduces scroll-spam redraw)."""
    if sys.platform != "win32":
        return
    try:
        import ctypes

        kernel32 = ctypes.windll.kernel32
        STD_OUTPUT_HANDLE = -11
        ENABLE_VIRTUAL_TERMINAL_PROCESSING = 0x0004
        h = kernel32.GetStdHandle(STD_OUTPUT_HANDLE)
        mode = ctypes.c_uint32()
        if kernel32.GetConsoleMode(h, ctypes.byref(mode)) == 0:
            return
        kernel32.SetConsoleMode(h, mode.value | ENABLE_VIRTUAL_TERMINAL_PROCESSING)
    except Exception:
        pass


def _configure_stdout_utf8() -> None:
    if hasattr(sys.stdout, "reconfigure"):
        try:
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass

# ─────────────────────────────────────────────────────────────────────────────
# CONFIG
# ─────────────────────────────────────────────────────────────────────────────
PI_HOST       = "http://192.168.1.100:8000"   # ← change to your Pi's IP
STATE_URL     = f"{PI_HOST}/state"
POST_TIMEOUT  = 0.5     # seconds per request before giving up
POLL_HZ       = 30      # UI + state-check refresh rate

# ─────────────────────────────────────────────────────────────────────────────
# KEY QUERY  —  keyboard.is_pressed() is non-blocking, no X display needed
# ─────────────────────────────────────────────────────────────────────────────
def is_w()     -> bool: return keyboard.is_pressed('w')
def is_a()     -> bool: return keyboard.is_pressed('a')
def is_s()     -> bool: return keyboard.is_pressed('s')
def is_d()     -> bool: return keyboard.is_pressed('d')
def is_ctrl()  -> bool: return keyboard.is_pressed('ctrl')
def is_space() -> bool: return keyboard.is_pressed('space')
def is_esc()   -> bool: return keyboard.is_pressed('esc')

# ─────────────────────────────────────────────────────────────────────────────
# STATE RESOLVER
# ─────────────────────────────────────────────────────────────────────────────
def get_state() -> str:
    """
    Resolves currently held keys → robot state string.
    Priority: SIT > JUMP > movement > STAND
    """
    if is_ctrl():  return "SIT"
    if is_space(): return "JUMP"
    if is_w():     return "FORWARD"
    if is_s():     return "BACKWARD"
    if is_a():     return "LEFT"
    if is_d():     return "RIGHT"
    return "STAND"

# ─────────────────────────────────────────────────────────────────────────────
# API  —  fire-and-forget in a daemon thread so the UI never blocks
# ─────────────────────────────────────────────────────────────────────────────
_last_sent: str = ""
_api_lock       = threading.Lock()
_log_lock       = threading.Lock()
_net_log: list[str] = []      # rolling list of last 4 POST results (oldest → newest)


def _post(state: str) -> None:
    """Runs in a background thread. Never called directly."""
    try:
        r = requests.post(STATE_URL, json={"state": state}, timeout=POST_TIMEOUT)
        tag = "✓" if r.status_code == 200 else f"✗ {r.status_code}"
    except requests.exceptions.ConnectionError:
        tag = "✗ no connection"
    except requests.exceptions.Timeout:
        tag = "✗ timeout"
    except Exception as e:
        tag = f"✗ {e}"

    row = f"{state:<10}  {tag}"
    with _log_lock:
        _net_log.append(row)
        if len(_net_log) > 4:
            _net_log.pop(0)


def maybe_post(state: str) -> None:
    """
    Fires a POST only when state differs from the last one sent.
    The actual request runs in a daemon thread — main loop is never stalled.
    """
    global _last_sent
    with _api_lock:
        if state == _last_sent:
            return
        _last_sent = state

    threading.Thread(target=_post, args=(state,), daemon=True).start()


def _snapshot_log_rows() -> list[str]:
    """Last up to 4 log lines, padded with blanks; order = oldest → newest (newest last)."""
    with _log_lock:
        tail = list(_net_log[-4:])
    pad = [""] * (4 - len(tail))
    return pad + tail


def _enable_windows_virtual_terminal() -> None:
    """So cursor-up redraw (ESC[nA) works in PowerShell / classic console."""
    if sys.platform != "win32":
        return
    try:
        kernel32 = ctypes.windll.kernel32
        h = kernel32.GetStdHandle(-11)
        mode = ctypes.c_uint32()
        if kernel32.GetConsoleMode(h, ctypes.byref(mode)) == 0:
            return
        kernel32.SetConsoleMode(h, mode.value | 0x0004)  # ENABLE_VIRTUAL_TERMINAL_PROCESSING
    except Exception:
        pass


# ─────────────────────────────────────────────────────────────────────────────
# TERMINAL UI
# ─────────────────────────────────────────────────────────────────────────────
_R    = "\033[0m"
_ON   = "\033[1;92m"    # bright green  — key held
_OFF  = "\033[2;37m"    # dim grey      — key not held
_BOLD = "\033[1m"
_DIM  = "\033[2m"
_LOG  = "\033[97m"    # bright white — POST lines (not dimmed)

_STATE_CLR = {
    "FORWARD":  "\033[1;96m",
    "BACKWARD": "\033[1;96m",
    "LEFT":     "\033[1;96m",
    "RIGHT":    "\033[1;96m",
    "SIT":      "\033[1;93m",
    "JUMP":     "\033[1;95m",
    "STAND":    "\033[0;90m",
}

_STATE_ICON = {
    "FORWARD":  "▲  FORWARD",
    "BACKWARD": "▼  BACKWARD",
    "LEFT":     "◀  LEFT",
    "RIGHT":    "▶  RIGHT",
    "SIT":      "⬇  SIT",
    "JUMP":     "★  JUMP",
    "STAND":    "■  STAND",
}

_S1 = "  " + "─" * 36
_S2 = "  " + "═" * 36
_UI_HEIGHT = 21


def _k(label: str, active: bool) -> str:
    c = _ON if active else _OFF
    return f"{c}[{label}]{_R}"


_first_draw = True


def draw_ui(state: str) -> None:
    global _first_draw

    w     = _k("W",                  is_w())
    a     = _k("A",                  is_a())
    s     = _k("S",                  is_s())
    d     = _k("D",                  is_d())
    ctrl  = _k(" CTRL ",             is_ctrl())
    space = _k("      SPACE      ",  is_space())

    sc   = _STATE_CLR.get(state, _R)
    icon = _STATE_ICON.get(state, state)

    log = _snapshot_log_rows()

    lines = [
        _S2,
        f"  {_BOLD}  BYTE-01  CONTROLLER{_R}",
        f"  {_DIM}  → {PI_HOST}{_R}",
        _S2,
        "",
        f"                 {w}",
        f"              {a}  {s}  {d}",
        "",
        f"       {ctrl}     {space}",
        "",
        _S1,
        f"    State  →  {sc}{_BOLD}{icon}{_R}",
        _S1,
        f"  {_DIM}POST log  (last 4, newest at bottom){_R}",
        f"  {_LOG}{log[0] or '  —'}{_R}",
        f"  {_LOG}{log[1] or '  —'}{_R}",
        f"  {_LOG}{log[2] or '  —'}{_R}",
        f"  {_LOG}{log[3] or '  —'}{_R}",
        _S2,
        f"  {_DIM}ESC → quit{_R}",
        "",
    ]

    assert len(lines) == _UI_HEIGHT, f"_UI_HEIGHT is {_UI_HEIGHT} but lines={len(lines)}"

    out = []
    if not _first_draw:
        out.append(f"\033[{_UI_HEIGHT}A")

    for line in lines:
        out.append(f"\r\033[K{line}")

    sys.stdout.write("\n".join(out))
    sys.stdout.flush()
    _first_draw = False


# ─────────────────────────────────────────────────────────────────────────────
# MAIN LOOP
# ─────────────────────────────────────────────────────────────────────────────
def main() -> None:
    dt = 1.0 / POLL_HZ

    _enable_windows_virtual_terminal()
    sys.stdout.write("\033[?25l")   # hide cursor
    sys.stdout.flush()

    try:
        while not is_esc():
            state = get_state()
            maybe_post(state)       # no-op if state hasn't changed
            draw_ui(state)
            time.sleep(dt)

    except KeyboardInterrupt:
        pass

    finally:
        sys.stdout.write("\033[?25h\n")
        sys.stdout.flush()
        # Safety: always land the robot on exit
        _post("STAND")
        with _log_lock:
            tail = list(_net_log[-4:])
        for row in tail:
            if row.strip():
                print(f"  POST  {row}")
        print("  Controller stopped — sent STAND to robot.")


if __name__ == "__main__":
    main()