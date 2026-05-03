#!/usr/bin/env python3
"""
keyboard_controller.py
════════════════════════════════════════════════════════════════════════
BYTE-01  Keyboard State Controller

Uses pynput to track which keys are currently held (not typed).
Exposes get_state() — call this at every gait cycle boundary.
Redraws a fixed terminal UI block — zero scroll spam.

Importable:
    from keyboard_controller import get_state, is_quit

    state = get_state()
    # → "FORWARD" | "BACKWARD" | "LEFT" | "RIGHT"
    #    "SIT"    | "JUMP"     | "STAND"

Run standalone to test the UI:
    python3 keyboard_controller.py

Keys:
    W / A / S / D   → FORWARD / LEFT / BACKWARD / RIGHT
    Ctrl            → SIT
    Space           → JUMP
    (nothing held)  → STAND
    ESC             → quit

Note: pynput on Linux needs either:
  - An X display  (desktop / VNC)
  - Root + evdev  (sudo python3 keyboard_controller.py)
  If running headless over SSH with no X, use the `keyboard` lib variant instead.
"""

import sys
import time
from pynput import keyboard as kb

# ─────────────────────────────────────────────────────────────────────────────
# KEY STATE  —  non-blocking, event-driven into a held set
# ─────────────────────────────────────────────────────────────────────────────
_held: set = set()
_quit: bool = False


def _on_press(key):
    _held.add(key)


def _on_release(key):
    global _quit
    _held.discard(key)
    if key == kb.Key.esc:
        _quit = True


_listener = kb.Listener(on_press=_on_press, on_release=_on_release, suppress=False)
_listener.daemon = True
_listener.start()


# ─────────────────────────────────────────────────────────────────────────────
# KEY QUERY HELPERS
# ─────────────────────────────────────────────────────────────────────────────
def _char(c: str) -> bool:
    """True if character key c is currently held."""
    for k in _held:
        try:
            if k.char and k.char.lower() == c:
                return True
        except AttributeError:
            pass
    return False


def _special(key: kb.Key) -> bool:
    return key in _held


def is_w()     -> bool: return _char('w')
def is_a()     -> bool: return _char('a')
def is_s()     -> bool: return _char('s')
def is_d()     -> bool: return _char('d')
def is_ctrl()  -> bool: return _special(kb.Key.ctrl_l) or _special(kb.Key.ctrl_r)
def is_space() -> bool: return _special(kb.Key.space)
def is_quit()  -> bool: return _quit


# ─────────────────────────────────────────────────────────────────────────────
# STATE RESOLVER
# ─────────────────────────────────────────────────────────────────────────────
def get_state() -> str:
    """
    Returns the current robot command state as a string.

    Priority (high → low):
        SIT  >  JUMP  >  FORWARD / BACKWARD / LEFT / RIGHT  >  STAND

    This is designed to be called at gait cycle boundaries, NOT every
    tick of your polling loop. The gait runner calls this once per cycle
    to decide whether to keep walking or return to stand.
    """
    if is_ctrl():  return "SIT"
    if is_space(): return "JUMP"
    if is_w():     return "FORWARD"
    if is_s():     return "BACKWARD"
    if is_a():     return "LEFT"
    if is_d():     return "RIGHT"
    return "STAND"


# ─────────────────────────────────────────────────────────────────────────────
# TERMINAL UI
# ─────────────────────────────────────────────────────────────────────────────
_R    = "\033[0m"
_ON   = "\033[1;92m"    # bright green  — key is held
_OFF  = "\033[2;37m"    # dim grey      — key is not held
_BOLD = "\033[1m"
_DIM  = "\033[2m"

_STATE_CLR = {
    "FORWARD":  "\033[1;96m",    # bright cyan
    "BACKWARD": "\033[1;96m",
    "LEFT":     "\033[1;96m",
    "RIGHT":    "\033[1;96m",
    "SIT":      "\033[1;93m",    # bright yellow
    "JUMP":     "\033[1;95m",    # bright magenta
    "STAND":    "\033[0;90m",    # dark grey  (idle)
}

_STATE_ICON = {
    "FORWARD":  "FORWARD",
    "BACKWARD": "BACKWARD",
    "LEFT":     "LEFT",
    "RIGHT":    "RIGHT",
    "SIT":      "SIT",
    "JUMP":     "JUMP",
    "STAND":    "STAND",
}

_SEP  = "  " + "─" * 32
_SEP2 = "  " + "═" * 32
_UI_HEIGHT = 14   # lines printed per frame — must match lines list length exactly


def _k(label: str, active: bool) -> str:
    """Render a single key label with on/off colour."""
    c = _ON if active else _OFF
    return f"{c}[{label}]{_R}"


_first_draw = True


def draw_ui(state: str) -> None:
    """
    Redraws the controller UI in-place.
    First call prints it; subsequent calls jump cursor up and overwrite.
    """
    global _first_draw

    w     = _k("W", is_w())
    a     = _k("A", is_a())
    s     = _k("S", is_s())
    d     = _k("D", is_d())
    ctrl  = _k("CTRL ", is_ctrl())
    space = _k("      SPACE      ", is_space())

    sc    = _STATE_CLR.get(state, _R)
    icon  = _STATE_ICON.get(state, state)

    lines = [
        _SEP2,
        f"  {_BOLD}  BYTE-01  CONTROLLER{_R}",
        _SEP2,
        "",
        f"              {w}",
        f"           {a}  {s}  {d}",
        "",
        f"     {ctrl}    {space}",
        "",
        _SEP,
        f"    State  →  {sc}{_BOLD}{icon}{_R}",
        _SEP2,
        f"  {_DIM}ESC → quit{_R}",
        "",
    ]

    assert len(lines) == _UI_HEIGHT, f"UI_HEIGHT mismatch: {len(lines)}"

    out = []
    if not _first_draw:
        out.append(f"\033[{_UI_HEIGHT}A")   # jump cursor up

    for line in lines:
        out.append(f"\r\033[K{line}")       # \033[K clears to EOL before writing

    sys.stdout.write("\n".join(out))
    sys.stdout.flush()
    _first_draw = False


# ─────────────────────────────────────────────────────────────────────────────
# STANDALONE LOOP  —  run this file directly to test the UI
# ─────────────────────────────────────────────────────────────────────────────
def main() -> None:
    POLL_HZ = 30        # UI refresh rate — purely cosmetic, not gait rate
    dt      = 1.0 / POLL_HZ

    sys.stdout.write("\033[?25l")   # hide cursor
    sys.stdout.flush()

    try:
        while not _quit:
            draw_ui(get_state())
            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write("\033[?25h\n")   # restore cursor
        sys.stdout.flush()
        _listener.stop()
        print("  Controller stopped.")


if __name__ == "__main__":
    main()
