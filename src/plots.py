"""Grafici di validazione della procedura.

Legge il log prodotto da main.py e genera due figure:

    targeting_error.png   distanza punta-bersaglio nel tempo, con le fasi
                          evidenziate
    trajectory_xz.png     traiettoria della punta nel piano sagittale,
                          sovrapposta alla geometria del phantom

"""

import csv
import os

import matplotlib.pyplot as plt
import numpy as np

import config as cfg

PHASE_COLORS = {
    "positioning": "#dfe6ef",
    "alignment": "#cfe0d8",
    "insertion": "#f6d9cf",
    "dwell": "#efe0b8",
    "retraction": "#d9d3e8",
    "homing": "#e8e8e8",
}

PHASE_LABELS = {
    "positioning": "posizionamento",
    "alignment": "allineamento",
    "insertion": "inserimento",
    "dwell": "prelievo",
    "retraction": "retrazione",
    "homing": "ritorno",
}


def load(path):
    with open(path, newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise RuntimeError(f"{path} è vuoto: esegui prima main.py")
    return rows


def phase_spans(rows):
    """Intervalli temporali contigui di ciascuna fase."""
    spans = []
    start = float(rows[0]["t"])
    current = rows[0]["phase"]
    for row in rows[1:]:
        if row["phase"] != current:
            spans.append((current, start, float(row["t"])))
            current, start = row["phase"], float(row["t"])
    spans.append((current, start, float(rows[-1]["t"])))
    return spans


def plot_error(rows, out_dir):
    t = np.array([float(r["t"]) for r in rows])
    error = np.array([float(r["error"]) for r in rows]) * 1000.0

    fig, ax = plt.subplots(figsize=(9, 4.2))

    spans = phase_spans(rows)
    total = t[-1] - t[0]
    for phase, t0, t1 in spans:
        ax.axvspan(t0, t1, color=PHASE_COLORS.get(phase, "#eeeeee"), zorder=0)
        narrow = (t1 - t0) < 0.12 * total
        ax.annotate(
            PHASE_LABELS.get(phase, phase),
            xy=((t0 + t1) / 2, 0.97), xycoords=("data", "axes fraction"),
            ha="center", va="top", fontsize=8, rotation=90 if narrow else 0,
            color="#444444", zorder=6,
        )

    ax.plot(t, error, color="#1f3b63", linewidth=1.8, zorder=3)
    ax.axhline(0, color="#999999", linewidth=0.8, linestyle="--", zorder=1)

    ax.set_xlabel("tempo simulato [s]")
    ax.set_ylabel("distanza punta-bersaglio [mm]")
    ax.set_title("Errore di targeting durante la procedura")
    ax.set_xlim(t[0], t[-1])
    ax.margins(y=0.16)
    ax.grid(alpha=0.25, zorder=1)

    fig.tight_layout()
    path = os.path.join(out_dir, "targeting_error.png")
    fig.savefig(path, dpi=160)
    plt.close(fig)
    print(f"Salvato {path}")


def plot_trajectory(rows, out_dir):
    x = np.array([float(r["tip_x"]) for r in rows])
    z = np.array([float(r["tip_z"]) for r in rows])
    phases = [r["phase"] for r in rows]

    fig, ax = plt.subplots(figsize=(6.5, 5.5))

    # Sezione del phantom nel piano XZ
    cx, _, cz = cfg.PHANTOM_CENTER
    sx, _, sz = cfg.PHANTOM_SIZE
    ax.add_patch(plt.Rectangle((cx - sx / 2, cz - sz / 2), sx, sz,
                               facecolor="#f4ded6", edgecolor="#c9a99c",
                               zorder=0, label="phantom"))
    ax.axhline(cfg.SKIN_Z, color="#c0392b", linewidth=1.0,
               linestyle=":", zorder=1, label="cute")

    lesion = cfg.LESION_POSITION
    ax.add_patch(plt.Circle((lesion[0], lesion[2]), cfg.LESION_RADIUS,
                            facecolor="#c0392b", alpha=0.75, zorder=2,
                            label="lesione"))

    # Andata e ritorno separati, per rendere visibile che coincidono.
    forward = [i for i, p in enumerate(phases)
               if p in ("positioning", "alignment", "insertion", "dwell")]
    backward = [i for i, p in enumerate(phases)
                if p in ("retraction", "homing")]

    ax.plot(x[forward], z[forward], color="#1f3b63", linewidth=2.0,
            zorder=4, label="andata")
    ax.plot(x[backward], z[backward], color="#2e8b57", linewidth=1.4,
            linestyle="--", zorder=5, label="ritorno")

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Z [m]")
    ax.set_title("Traiettoria della punta nel piano sagittale")
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    ax.legend(loc="upper left", fontsize=8)

    fig.tight_layout()
    path = os.path.join(out_dir, "trajectory_xz.png")
    fig.savefig(path, dpi=160)
    plt.close(fig)
    print(f"Salvato {path}")


def main() -> None:
    rows = load(cfg.LOG_PATH)
    out_dir = os.path.dirname(cfg.LOG_PATH)

    plot_error(rows, out_dir)
    plot_trajectory(rows, out_dir)

    final_error = float(rows[-1]["error"])
    min_error = min(float(r["error"]) for r in rows)
    max_depth = max(float(r["depth"]) for r in rows)

    print(f"\nErrore minimo raggiunto : {min_error * 1000:.3f} mm")
    print(f"Profondità massima      : {max_depth * 1000:.1f} mm")
    print(f"Errore a fine procedura : {final_error * 1000:.1f} mm (ago estratto)")


if __name__ == "__main__":
    main()