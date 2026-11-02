"""
Real-time decision monitor for the ego vehicle agent.
Runs in a daemon background thread; all calls are thread-safe via a Queue.

Layout
──────
  ┌──────────────────────────────────────────────────────────────┐
  │  ⚡  EGO VEHICLE — DECISION MONITOR                          │
  ├──────────────────────────────────────────────────────────────┤
  │  Speed: 32.4 km/h │ State: DRIVING                 ● CLEAR  │
  ├══════════════════════════════════════════════════════════════╡
  │  NOW  ↪  Construction bypass — lateral offset -1.82 m       │
  ├──────────────────────────────────────────────────────────────┤
  │  EVENT LOG (transitions only)                                │
  │  [14:23:01] ⛔ Vehicle obstacle ahead — emergency stop       │
  │  [14:23:05] ✔  Path clear — resuming normal driving         │
  └──────────────────────────────────────────────────────────────┘

update_current()  →  rewrites the NOW line every step  (smooth, no spam)
log()             →  appends to the event log only on state transitions
"""
import threading
import queue
from datetime import datetime
from typing import Optional

try:
    import tkinter as tk
    from tkinter import font as tkfont
    _TK_OK = True
except ImportError:
    _TK_OK = False


class DecisionDisplay:
    """Dedicated dark-themed window that shows ego-vehicle decisions in real time."""

    _COLORS = {
        'bg':        '#0b0b1e',
        'hdr':       '#0d0d28',
        'panel':     '#101028',
        'now_bg':    '#0c0c24',
        'border':    '#1c3a6e',
        'hazard':    '#ff4f4f',
        'warning':   '#ffb020',
        'clear':     '#3dd870',
        'info':      '#40a8ff',
        'action':    '#cc80ff',
        'text':      '#c0c0d8',
        'ts':        '#4c4c72',
        'dim':       '#181834',
        'scrollbar': '#282860',
    }

    _ICONS = {
        'hazard':  '⛔',
        'warning': '⚠ ',
        'clear':   '✔ ',
        'info':    '→ ',
        'action':  '↪ ',
    }

    _LOG_BG = {
        'hazard':  '#2a0606',
        'warning': '#281c04',
        'clear':   '#061a0c',
        'action':  '#160826',
        'info':    None,
    }

    def __init__(self, title: str = "Ego Vehicle Decision Monitor",
                 width: int = 800, height: int = 580):
        self._title  = title
        self._width  = width
        self._height = height
        self._q: queue.Queue = queue.Queue()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._root: Optional[tk.Tk] = None

    # ── public API ────────────────────────────────────────────────────────────

    def start(self) -> None:
        """Open the window in a daemon background thread."""
        if not _TK_OK:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run,
                                        name="DecisionDisplay", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        self._q.put(('quit',))

    def update_current(self, message: str, level: str = 'info') -> None:
        """
        Rewrite the NOW line in-place every step.
        Call this every run_step() regardless of state changes.
        level: 'hazard' | 'warning' | 'clear' | 'action' | 'info'
        """
        self._q.put(('current', level, message))

    def log(self, message: str, level: str = 'info') -> None:
        """
        Append a timestamped entry to the event log.
        Call only on genuine state transitions (not every step).
        """
        ts = datetime.now().strftime('%H:%M:%S')
        self._q.put(('log', ts, level, message))
        icon = self._ICONS.get(level, '  ').strip()
        print(f"[{ts}] [{level.upper():7s}] {icon} {message}")

    def update_status(self, speed: Optional[float] = None,
                      state: Optional[str] = None,
                      hazard: Optional[bool] = None) -> None:
        """Refresh the top status bar (non-blocking)."""
        self._q.put(('status', speed, state, hazard))

    # ── window lifecycle ──────────────────────────────────────────────────────

    def _run(self) -> None:
        try:
            self._root = tk.Tk()
            self._root.title(self._title)
            self._root.configure(bg=self._COLORS['bg'])
            self._root.geometry(f"{self._width}x{self._height}")
            self._root.resizable(True, True)
            self._root.protocol("WM_DELETE_WINDOW", self._on_close)
            self._build_ui()
            self._poll()
            self._root.mainloop()
        except Exception as exc:
            print(f"[DecisionDisplay] window error: {exc}")

    def _on_close(self) -> None:
        self._running = False
        if self._root:
            self._root.quit()

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        C = self._COLORS
        root = self._root

        # fonts ---------------------------------------------------------------
        def mk(size, bold=False):
            for fam in ('JetBrains Mono', 'Fira Code', 'Consolas', 'Courier New', 'Courier'):
                try:
                    f = tkfont.Font(family=fam, size=size,
                                    weight='bold' if bold else 'normal')
                    if f.actual('family').lower() not in ('helvetica', 'times new roman'):
                        return f
                except Exception:
                    pass
            return tkfont.Font(family='Courier', size=size,
                               weight='bold' if bold else 'normal')

        f_title  = mk(13, bold=True)
        f_status = mk(11, bold=True)
        f_now    = mk(12, bold=True)
        f_log    = mk(11)
        f_ts     = mk(10)

        # ── header ───────────────────────────────────────────────────────────
        hdr = tk.Frame(root, bg=C['hdr'], pady=11)
        hdr.pack(fill=tk.X)
        tk.Label(hdr,
                 text="⚡   E G O   V E H I C L E   —   D E C I S I O N   M O N I T O R",
                 bg=C['hdr'], fg='#5ac8ff', font=f_title
                 ).pack(side=tk.LEFT, padx=16)
        tk.Frame(root, bg=C['border'], height=2).pack(fill=tk.X)

        # ── status bar ───────────────────────────────────────────────────────
        sb = tk.Frame(root, bg=C['panel'], pady=8)
        sb.pack(fill=tk.X)

        self._speed_var  = tk.StringVar(value="Speed:   0.0 km/h")
        self._state_var  = tk.StringVar(value="State:   STARTING")
        self._hazard_var = tk.StringVar(value="●  CLEAR")

        tk.Label(sb, textvariable=self._speed_var,
                 bg=C['panel'], fg=C['text'], font=f_status
                 ).pack(side=tk.LEFT, padx=16)

        self._state_lbl = tk.Label(sb, textvariable=self._state_var,
                                   bg=C['panel'], fg=C['clear'], font=f_status)
        self._state_lbl.pack(side=tk.LEFT, padx=16)

        self._hazard_lbl = tk.Label(sb, textvariable=self._hazard_var,
                                    bg=C['panel'], fg=C['clear'], font=f_status)
        self._hazard_lbl.pack(side=tk.RIGHT, padx=16)

        # ── NOW panel (always updating, in-place) ────────────────────────────
        tk.Frame(root, bg='#252560', height=2).pack(fill=tk.X)

        now_row = tk.Frame(root, bg=C['now_bg'], pady=10)
        now_row.pack(fill=tk.X)

        # coloured left accent bar
        self._now_accent = tk.Frame(now_row, bg=C['info'], width=5)
        self._now_accent.pack(side=tk.LEFT, fill=tk.Y, padx=(8, 0))

        tk.Label(now_row, text="  NOW",
                 bg=C['now_bg'], fg=C['ts'], font=f_status
                 ).pack(side=tk.LEFT, padx=(6, 12))

        self._now_var = tk.StringVar(value="→   Initializing…")
        self._now_lbl = tk.Label(now_row, textvariable=self._now_var,
                                  bg=C['now_bg'], fg=C['info'], font=f_now)
        self._now_lbl.pack(side=tk.LEFT)

        tk.Frame(root, bg=C['border'], height=2).pack(fill=tk.X)

        # ── event log section header ─────────────────────────────────────────
        log_hdr = tk.Frame(root, bg=C['dim'], pady=3)
        log_hdr.pack(fill=tk.X, padx=6)
        tk.Label(log_hdr, text="  EVENT LOG",
                 bg=C['dim'], fg=C['ts'], font=f_ts).pack(side=tk.LEFT)

        # ── scrolling log ────────────────────────────────────────────────────
        outer = tk.Frame(root, bg=C['bg'])
        outer.pack(fill=tk.BOTH, expand=True, padx=6, pady=(2, 6))

        self._text = tk.Text(
            outer,
            bg=C['bg'], fg=C['text'],
            font=f_log,
            relief=tk.FLAT, bd=0,
            state=tk.DISABLED,
            wrap=tk.CHAR,
            cursor='arrow',
            selectbackground=C['dim'],
            padx=8, pady=2,
        )

        vsb = tk.Scrollbar(outer, orient=tk.VERTICAL, command=self._text.yview,
                           bg=C['dim'], troughcolor=C['bg'],
                           activebackground=C['scrollbar'])
        self._text.configure(yscrollcommand=vsb.set)
        vsb.pack(side=tk.RIGHT, fill=tk.Y)
        self._text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # text tags
        self._text.tag_configure('ts', foreground=C['ts'], font=f_ts)
        for lvl in ('hazard', 'warning', 'clear', 'info', 'action'):
            kw = {'foreground': C[lvl]}
            if self._LOG_BG.get(lvl):
                kw['background'] = self._LOG_BG[lvl]
            self._text.tag_configure(lvl, **kw)

    # ── queue processing ─────────────────────────────────────────────────────

    def _poll(self) -> None:
        try:
            while True:
                item = self._q.get_nowait()
                if item[0] == 'current':
                    _, lvl, msg = item
                    self._refresh_now(lvl, msg)
                elif item[0] == 'log':
                    _, ts, lvl, msg = item
                    self._write(ts, lvl, msg)
                elif item[0] == 'status':
                    _, spd, state, hazard = item
                    self._refresh_status(spd, state, hazard)
                elif item[0] == 'quit':
                    if self._root:
                        self._root.quit()
                    return
        except queue.Empty:
            pass
        if self._running and self._root:
            self._root.after(40, self._poll)

    def _refresh_now(self, level: str, message: str) -> None:
        icon  = self._ICONS.get(level, '  ')
        color = self._COLORS.get(level, self._COLORS['text'])
        self._now_var.set(f"{icon}   {message}")
        self._now_lbl.configure(fg=color)
        self._now_accent.configure(bg=color)

    def _write(self, ts: str, level: str, message: str) -> None:
        icon = self._ICONS.get(level, '  ')
        tag  = level if level in self._ICONS else 'info'
        self._text.configure(state=tk.NORMAL)
        self._text.insert(tk.END, f" [{ts}]  ", 'ts')
        self._text.insert(tk.END, f"{icon} {message}\n", tag)
        self._text.configure(state=tk.DISABLED)
        self._text.see(tk.END)

    def _refresh_status(self, speed, state, hazard) -> None:
        C = self._COLORS
        if speed is not None:
            self._speed_var.set(f"Speed:  {speed:5.1f} km/h")
        if state is not None:
            self._state_var.set(f"State:  {state}")
        if hazard is not None:
            if hazard:
                self._hazard_var.set("●  HAZARD")
                self._hazard_lbl.configure(fg=C['hazard'])
                self._state_lbl.configure(fg=C['hazard'])
            else:
                self._hazard_var.set("●  CLEAR ")
                self._hazard_lbl.configure(fg=C['clear'])
                self._state_lbl.configure(fg=C['clear'])


# ── module-level singleton ────────────────────────────────────────────────────

_display = None  # type: Optional[DecisionDisplay]


def get_display() -> DecisionDisplay:
    """Return the shared display instance, creating and starting it on first call."""
    global _display
    if _display is None:
        _display = DecisionDisplay()
        _display.start()
    return _display
