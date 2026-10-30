# Note from Nikolai: This code is vibecoded so please be careful with it's use.
"""
FSM State Transition Visualizer
Parses state_initilizer.cpp and renders a state diagram using graphviz,
displayed in an interactive tkinter window with zoom, pan, and toggles.

Everything is derived from the C++ source — no hardcoded state names.

Dependencies:
    pip install -r requirements.txt
    + graphviz system binary:
        macOS:   brew install graphviz
        Ubuntu:  sudo apt install graphviz
        Windows: choco install graphviz   (or winget install graphviz)

Usage:
    python fsm_visualizer.py                  # interactive window
    python fsm_visualizer.py --show-boot      # start with BOOT edges visible
    python fsm_visualizer.py --export svg     # export to output/fsm.svg (no GUI)
"""

from __future__ import annotations

import re
import sys
import shutil
import argparse
from pathlib import Path
from typing import Optional

import graphviz

try:
    import tkinter as tk
    import tkinter.ttk as ttk
    from PIL import Image, ImageTk
except ImportError:
    tk = None
    ttk = None
    Image = None
    ImageTk = None

_TkBase = tk.Tk if tk is not None else object

# ── Paths ───────────────────────────────────────────────────────────────────
SCRIPT_DIR = Path(__file__).resolve().parent
SOURCE_PATH = SCRIPT_DIR / ".." / ".." / "App" / "Src" / "Threads" / "Fsm" / "state_initilizer.cpp"
OUTPUT_DIR = SCRIPT_DIR / "output"

# ── Style ───────────────────────────────────────────────────────────────────
NODE_PALETTE = [
    "#4a90d9", "#f5a623", "#e8832a", "#d4712a", "#c06020", "#50b87a",
    "#d0021b", "#c44040", "#7ed321", "#5cb85c", "#50c878", "#9b9b9b",
    "#8e44ad", "#2c3e50", "#1abc9c", "#e74c3c",
]


# ── Parser ──────────────────────────────────────────────────────────────────
def parse_source(path: Path) -> dict[str, list[tuple[str, str]]]:
    """
    Parse state_initilizer.cpp -> { source_state: [(event, target_state), ...] }
    Works for any states.emplace(STATE_NAME::X, ...) block.
    """
    text = path.read_text()
    transitions: dict[str, list[tuple[str, str]]] = {}

    emplace_re = re.compile(r'states\.emplace\s*\(\s*STATE_NAME::(\w+)\s*,', re.DOTALL)
    trans_re = re.compile(r'\{\s*EVENT::(\w+)\s*,\s*STATE_NAME::(\w+)\s*\}')

    for m in emplace_re.finditer(text):
        state = m.group(1)
        depth, end = 0, m.start()
        for i in range(m.start(), len(text)):
            if text[i] == '(':
                depth += 1
            elif text[i] == ')':
                depth -= 1
                if depth == 0:
                    end = i
                    break
        block = text[m.start():end + 1]
        transitions[state] = trans_re.findall(block)

    return transitions


# ── Graph builder ───────────────────────────────────────────────────────────
def build_graph(
    transitions: dict[str, list[tuple[str, str]]],
    show_boot: bool = False,
    fmt: str = "png",
) -> graphviz.Digraph:
    dot = graphviz.Digraph(
        "FSM",
        format=fmt,
        engine="dot",
        graph_attr={
            "rankdir": "TB",
            "bgcolor": "#1e1e1e",
            "fontcolor": "white",
            "fontname": "Helvetica",
            "pad": "0.5",
            "nodesep": "0.8",
            "ranksep": "1.0",
            "label": "Flight Computer FSM — State Transitions",
            "labelloc": "t",
            "fontsize": "20",
        },
        node_attr={
            "shape": "circle",
            "style": "filled",
            "fontname": "Helvetica",
            "fontsize": "10",
            "fixedsize": "true",
            "width": "1.3",
            "height": "1.3",
            "fontcolor": "white",
            "penwidth": "2",
            "color": "#444444",
        },
        edge_attr={
            "fontname": "Helvetica",
            "fontsize": "9",
            "color": "#999999",
            "fontcolor": "#cccccc",
            "penwidth": "1.5",
            "arrowsize": "0.8",
        },
    )

    ordered = list(transitions.keys())
    all_targets = {tgt for trans in transitions.values() for _, tgt in trans}
    for t in all_targets:
        if t not in ordered:
            ordered.append(t)

    for i, state in enumerate(ordered):
        color = NODE_PALETTE[i % len(NODE_PALETTE)]
        dot.node(state, label=state.replace("_", "\\n"), fillcolor=color)

    for src, trans_list in transitions.items():
        for event, tgt in trans_list:
            is_boot = event == "BOOT"
            if is_boot and not show_boot:
                continue
            dot.edge(
                src, tgt,
                label=f"  {event.replace('_', ' ')}  ",
                color="#555555" if is_boot else "#999999",
                fontcolor="#777777" if is_boot else "#cccccc",
                style="dashed" if is_boot else "solid",
                penwidth="1.0" if is_boot else "1.5",
            )

    return dot


def render_to_png(transitions: dict, show_boot: bool) -> str:
    """Render graph and return path to the PNG file."""
    OUTPUT_DIR.mkdir(exist_ok=True)
    dot = build_graph(transitions, show_boot=show_boot, fmt="png")
    out = str(OUTPUT_DIR / "fsm")
    dot.render(out, cleanup=True)
    return out + ".png"


# ── Interactive tkinter viewer ──────────────────────────────────────────────
class FsmViewer(_TkBase):
    def __init__(self, transitions: dict, initial_boot: bool = False):
        super().__init__()
        self.title("FSM Visualizer — Propulse Flight Computer")
        self.configure(bg="#1e1e1e")
        self.geometry("900x750")

        self.transitions = transitions
        self.show_boot = tk.BooleanVar(value=initial_boot)
        self._pil_image: Optional[Image.Image] = None
        self._tk_image: Optional[ImageTk.PhotoImage] = None
        self._scale = 1.0
        self._img_id: Optional[int] = None
        self._drag_data = {"x": 0, "y": 0}

        self._build_toolbar()
        self._build_canvas()
        self._render()

    def _build_toolbar(self):
        toolbar = tk.Frame(self, bg="#2d2d2d", height=40)
        toolbar.pack(side=tk.TOP, fill=tk.X)
        toolbar.pack_propagate(False)

        boot_cb = tk.Checkbutton(
            toolbar,
            text="Show BOOT → IDLE",
            variable=self.show_boot,
            command=self._render,
            bg="#2d2d2d", fg="white", selectcolor="#444444",
            activebackground="#2d2d2d", activeforeground="white",
            font=("Helvetica", 12),
        )
        boot_cb.pack(side=tk.LEFT, padx=12, pady=4)

        # Use ttk buttons with a dark style for macOS compatibility
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure(
            "Dark.TButton",
            background="#3d3d3d",
            foreground="white",
            borderwidth=0,
            font=("Helvetica", 12),
            padding=(10, 4),
        )
        style.map(
            "Dark.TButton",
            background=[("active", "#555555")],
            foreground=[("active", "white")],
        )

        export_btn = ttk.Button(
            toolbar, text="Export PNG",
            command=self._export,
            style="Dark.TButton",
        )
        export_btn.pack(side=tk.RIGHT, padx=12, pady=4)

        reset_btn = ttk.Button(
            toolbar, text="Reset View",
            command=self._reset_view,
            style="Dark.TButton",
        )
        reset_btn.pack(side=tk.RIGHT, padx=4, pady=4)

        zoom_out_btn = ttk.Button(
            toolbar, text="\u2212",
            command=lambda: self._zoom_center(1 / 1.3),
            style="Dark.TButton",
        )
        zoom_out_btn.pack(side=tk.RIGHT, padx=2, pady=4)

        zoom_in_btn = ttk.Button(
            toolbar, text="+",
            command=lambda: self._zoom_center(1.3),
            style="Dark.TButton",
        )
        zoom_in_btn.pack(side=tk.RIGHT, padx=2, pady=4)

    def _build_canvas(self):
        self.canvas = tk.Canvas(self, bg="#1e1e1e", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Focus canvas on enter so it receives key/scroll events
        self.canvas.bind("<Enter>", lambda e: self.canvas.focus_set())

        # Scroll zoom — try every known binding for cross-platform coverage
        for target in (self.canvas, self):
            target.bind("<MouseWheel>", self._on_scroll)
        self.bind_all("<MouseWheel>", self._on_scroll)
        self.canvas.bind("<Button-4>", self._on_scroll_linux)   # Linux up
        self.canvas.bind("<Button-5>", self._on_scroll_linux)   # Linux down

        # Keyboard zoom: Cmd+=/- on macOS, Ctrl+=/- elsewhere, and plain +/-
        self.bind_all("<Command-equal>", lambda e: self._zoom_center(1.3))
        self.bind_all("<Command-minus>", lambda e: self._zoom_center(1 / 1.3))
        self.bind_all("<Control-equal>", lambda e: self._zoom_center(1.3))
        self.bind_all("<Control-minus>", lambda e: self._zoom_center(1 / 1.3))
        self.canvas.bind("<plus>", lambda e: self._zoom_center(1.3))
        self.canvas.bind("<minus>", lambda e: self._zoom_center(1 / 1.3))
        self.canvas.bind("<equal>", lambda e: self._zoom_center(1.3))

        # Pan
        self.canvas.bind("<ButtonPress-1>", self._on_drag_start)
        self.canvas.bind("<B1-Motion>", self._on_drag)

    def _render(self):
        png_path = render_to_png(self.transitions, self.show_boot.get())
        self._pil_image = Image.open(png_path)
        self._scale = 1.0
        self._place_image()

    def _place_image(self):
        self.canvas.delete("all")
        if self._pil_image is None:
            return

        w = max(1, int(self._pil_image.width * self._scale))
        h = max(1, int(self._pil_image.height * self._scale))
        resized = self._pil_image.resize((w, h), Image.LANCZOS)
        self._tk_image = ImageTk.PhotoImage(resized)

        cx = self.canvas.winfo_width() // 2
        cy = self.canvas.winfo_height() // 2
        self._img_id = self.canvas.create_image(
            cx, cy, anchor=tk.CENTER, image=self._tk_image, tags="img",
        )

    def _reset_view(self):
        self._scale = 1.0
        self._place_image()

    def _export(self):
        png_path = render_to_png(self.transitions, self.show_boot.get())
        print(f"Exported to {png_path}")

    # ── Zoom ──
    def _zoom_center(self, factor: float):
        """Zoom around the center of the canvas (for buttons/keyboard)."""
        cx = self.canvas.winfo_width() / 2
        cy = self.canvas.winfo_height() / 2
        self._apply_zoom(factor, cx, cy)

    def _apply_zoom(self, factor: float, cx: float, cy: float):
        new_scale = self._scale * factor
        new_scale = max(0.15, min(5.0, new_scale))
        if abs(new_scale - self._scale) < 1e-6:
            return

        # Zoom centered on (cx, cy): shift image so the point under
        # the cursor stays fixed.
        nx, ny = cx, cy
        if self._img_id is not None:
            ix, iy = self.canvas.coords(self._img_id)
            ratio = new_scale / self._scale
            nx = cx + (ix - cx) * ratio
            ny = cy + (iy - cy) * ratio

        self._scale = new_scale

        if self._pil_image is None:
            return
        w = max(1, int(self._pil_image.width * self._scale))
        h = max(1, int(self._pil_image.height * self._scale))
        resized = self._pil_image.resize((w, h), Image.LANCZOS)
        self._tk_image = ImageTk.PhotoImage(resized)

        self.canvas.delete("all")
        self._img_id = self.canvas.create_image(
            nx, ny, anchor=tk.CENTER, image=self._tk_image, tags="img",
        )

    def _on_scroll(self, event):
        d = event.delta
        if d == 0:
            return
        # Normalize: macOS sends ±1..±N, Windows sends ±120*N
        if abs(d) >= 10:
            d = d / 120.0
        factor = 1.05 ** d
        # Use pointer position relative to canvas
        px = self.winfo_pointerx() - self.canvas.winfo_rootx()
        py = self.winfo_pointery() - self.canvas.winfo_rooty()
        self._apply_zoom(factor, float(px), float(py))
        return "break"  # prevent event propagation to avoid double-handling

    def _on_scroll_linux(self, event):
        factor = 1.1 if event.num == 4 else 1 / 1.1
        self._apply_zoom(factor, float(event.x), float(event.y))

    # ── Pan ──
    def _on_drag_start(self, event):
        self._drag_data["x"] = event.x
        self._drag_data["y"] = event.y

    def _on_drag(self, event):
        dx = event.x - self._drag_data["x"]
        dy = event.y - self._drag_data["y"]
        self.canvas.move("all", dx, dy)
        self._drag_data["x"] = event.x
        self._drag_data["y"] = event.y


# ── CLI ─────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Visualize FSM state transitions")
    parser.add_argument("--show-boot", action="store_true",
                        help="Include BOOT->IDLE edges (hidden by default)")
    parser.add_argument("--export", choices=["png", "pdf", "svg"],
                        help="Export to output/ and exit (no GUI)")
    args = parser.parse_args()

    if shutil.which("dot") is None:
        print("Error: 'dot' (graphviz) not found on PATH.", file=sys.stderr)
        print("Install it:", file=sys.stderr)
        print("  macOS:   brew install graphviz", file=sys.stderr)
        print("  Ubuntu:  sudo apt install graphviz", file=sys.stderr)
        print("  Windows: choco install graphviz", file=sys.stderr)
        sys.exit(1)

    source = SOURCE_PATH.resolve()
    if not source.exists():
        print(f"Error: source not found at {source}", file=sys.stderr)
        sys.exit(1)

    transitions = parse_source(source)
    print(f"Parsed {len(transitions)} states from {source.name}")

    if args.export:
        OUTPUT_DIR.mkdir(exist_ok=True)
        dot = build_graph(transitions, show_boot=args.show_boot, fmt=args.export)
        out = str(OUTPUT_DIR / "fsm")
        dot.render(out, cleanup=True)
        print(f"Exported -> {out}.{args.export}")
    else:
        if tk is None or ttk is None or Image is None or ImageTk is None:
            print("Error: tkinter/Pillow viewer dependencies are not available.", file=sys.stderr)
            print("Use --export png or --export svg for headless rendering.", file=sys.stderr)
            sys.exit(1)
        app = FsmViewer(transitions, initial_boot=args.show_boot)
        app.mainloop()


if __name__ == "__main__":
    main()
