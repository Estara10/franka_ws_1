"""Shared publication-style settings for final project figures.

The workstation does not have Microsoft YaHei installed. Use the installed
Noto Sans CJK face as the CJK fallback so Chinese captions render as text
instead of tofu boxes. The style is intentionally close to a restrained
Science/Nature figure: thin axes, no top/right spines, compact sans-serif
labels, restrained colors and vector-friendly output.
"""

from pathlib import Path

import matplotlib
from matplotlib import font_manager


FONT_CANDIDATES = [
    Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
    Path("/usr/share/fonts/truetype/wqy/wqy-microhei.ttc"),
]

NATURE = {
    "blue": "#0C5DA5",
    "red": "#C43D4B",
    "green": "#2A9D8F",
    "orange": "#E69F00",
    "purple": "#7A5195",
    "gray": "#6C757D",
    "light_gray": "#D9D9D9",
}
GROUP_COLORS = {
    "A": NATURE["blue"],
    "B": NATURE["red"],
    "C": NATURE["green"],
    "D": NATURE["purple"],
}


def install_cjk_font() -> str:
    for path in FONT_CANDIDATES:
        if path.exists():
            font_manager.fontManager.addfont(str(path))
            return font_manager.FontProperties(fname=str(path)).get_name()
    return "DejaVu Sans"


def configure_scientific_style() -> str:
    family = install_cjk_font()
    matplotlib.rcParams.update(
        {
            "font.family": family,
            "font.size": 8,
            "axes.titlesize": 9,
            "axes.labelsize": 8,
            "xtick.labelsize": 7,
            "ytick.labelsize": 7,
            "legend.fontsize": 7,
            "axes.linewidth": 0.6,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.grid": True,
            "grid.alpha": 0.22,
            "grid.linewidth": 0.45,
            "lines.linewidth": 1.0,
            "figure.dpi": 160,
            "savefig.dpi": 300,
            "savefig.bbox": "tight",
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )
    return family
