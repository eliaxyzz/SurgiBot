"""Cinematica e controllo del robot per inserimento ago.

Il robot ha cinque gradi di libertà:

    q1, q2, q3   prismatici lungo X, Y, Z del mondo. Posizionano il punto di
                 pivot, cioè il centro di rotazione dell'ago.
    q4           rotoidale attorno all'asse Y del mondo. Inclina l'ago nel
                 piano XZ.
    q5           prismatico lungo l'asse dell'ago. È l'avanzamento vero e
                 proprio dentro il tessuto.

Le funzioni di cinematica sono pure e quindi testabili senza
avere CoppeliaSim in esecuzione.
"""

from __future__ import annotations

import numpy as np

import config as cfg

JOINT_NAMES = ["X", "Y", "Z", "Tilt", "Ins"]


# --- Cinematica in forma chiusa -------------------------------------------


def needle_axis(tilt: float) -> np.ndarray:
    """Versore dell'ago, orientato nel verso di avanzamento.

    A tilt nullo l'ago punta verticalmente verso il basso. Un tilt positivo
    lo inclina verso -X, coerentemente con la rotazione attorno a +Y.
    """
    return np.array([-np.sin(tilt), 0.0, -np.cos(tilt)])


def forward_kinematics(q) -> np.ndarray:
    """Posizione della punta dell'ago data la configurazione dei giunti."""
    q = np.asarray(q, dtype=float)
    pivot = np.array([q[0], q[1], cfg.GANTRY_Z + q[2]])
    return pivot + (cfg.NEEDLE_LENGTH + q[4]) * needle_axis(q[3])


def inverse_kinematics(target, tilt: float):
    """Configurazioni di giunto per raggiungere il target con dato tilt.

    Restituisce due configurazioni:
      q_approach  punta allineata sulla traiettoria, ferma a distanza di
                  sicurezza dalla cute (q5 = 0)
      q_target    punta esattamente sul centro della lesione

    Differiscono unicamente per il valore di q5: è questo che rende la
    procedura una singola traslazione rettilinea e rende banale la
    retrazione lungo la stessa retta.
    """
    target = np.asarray(target, dtype=float)

    # Versore che dal target risale verso la cute lungo la traiettoria.
    back = -needle_axis(tilt)

    # Ingresso cutaneo: intersezione della traiettoria con il piano z = SKIN_Z
    depth = (cfg.SKIN_Z - target[2]) / np.cos(tilt)
    entry = target + depth * back

    # Il pivot arretra rispetto all'ingresso di una lunghezza d'ago più il
    # margine di sicurezza, così a q5 = 0 la punta resta fuori dal tessuto.
    pivot = entry + (cfg.STANDOFF + cfg.NEEDLE_LENGTH) * back

    q_approach = [pivot[0], pivot[1], pivot[2] - cfg.GANTRY_Z, tilt, 0.0]

    q_target = list(q_approach)
    q_target[4] = cfg.STANDOFF + depth

    return q_approach, q_target


def check_limits(q) -> list[str]:
    """Elenca i giunti fuori corsa. Lista vuota significa configurazione valida."""
    problems = []
    for name, value in zip(JOINT_NAMES, q):
        low, span = cfg.JOINT_LIMITS[name]
        if not (low <= value <= low + span):
            problems.append(
                f"{name}: {value:.4f} fuori da [{low:.3f}, {low + span:.3f}]"
            )
    return problems


# --- Interfaccia verso la simulazione -------------------------------------


class NeedleRobot:
    """Wrapper sui giunti del robot presenti nella scena CoppeliaSim."""

    def __init__(self, sim):
        self.sim = sim
        self.joints = [sim.getObject(f"/{cfg.SCENE_ROOT}/Joint{n}")
                       for n in JOINT_NAMES]
        self.tip = sim.getObject(f"/{cfg.SCENE_ROOT}/NeedleTip")
        self.target = sim.getObject(f"/{cfg.SCENE_ROOT}/Target")

    def set_joints(self, q) -> None:
        """Impone la configurazione dei giunti (controllo cinematico diretto)."""
        for handle, value in zip(self.joints, q):
            self.sim.setJointPosition(handle, float(value))

    def get_joints(self) -> list[float]:
        return [self.sim.getJointPosition(h) for h in self.joints]

    def tip_position(self) -> np.ndarray:
        """Posizione della punta letta dalla simulazione, in coordinate mondo."""
        return np.array(self.sim.getObjectPosition(self.tip, self.sim.handle_world))

    def target_position(self) -> np.ndarray:
        return np.array(self.sim.getObjectPosition(self.target, self.sim.handle_world))

    def targeting_error(self) -> float:
        """Distanza punta-bersaglio in metri."""
        return float(np.linalg.norm(self.tip_position() - self.target_position()))


# --- Verifica offline della cinematica ------------------------------------


def _self_test() -> None:
    """Controlla che cinematica diretta e inversa siano coerenti.
    """
    target = np.array(cfg.LESION_POSITION)
    tilt = cfg.INSERTION_ANGLE

    q_appr, q_targ = inverse_kinematics(target, tilt)

    tip_appr = forward_kinematics(q_appr)
    tip_targ = forward_kinematics(q_targ)

    print("Configurazione di avvicinamento:")
    print("  q = [" + ", ".join(f"{v:.4f}" for v in q_appr) + "]")
    print(f"  punta = [{tip_appr[0]:.4f}, {tip_appr[1]:.4f}, {tip_appr[2]:.4f}]")
    print(f"  altezza sulla cute = {(tip_appr[2] - cfg.SKIN_Z) * 1000:.1f} mm")

    print("Configurazione a bersaglio:")
    print("  q = [" + ", ".join(f"{v:.4f}" for v in q_targ) + "]")
    print(f"  punta = [{tip_targ[0]:.4f}, {tip_targ[1]:.4f}, {tip_targ[2]:.4f}]")

    error = np.linalg.norm(tip_targ - target)
    print(f"\nErrore di targeting analitico: {error * 1e6:.3f} micrometri")

    problems = check_limits(q_appr) + check_limits(q_targ)
    if problems:
        print("\nATTENZIONE, giunti fuori corsa:")
        for p in problems:
            print("  " + p)
    else:
        print("Tutte le configurazioni rientrano nelle corse dei giunti.")

    assert error < 1e-9, "cinematica diretta e inversa non coerenti"


if __name__ == "__main__":
    _self_test()