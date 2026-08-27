"""Esecuzione della procedura di inserimento ago.

La procedura è organizzata come una ASF di quattro fasi, che
corrispondono ai quattro movimenti clinicamente necessari:

    1. POSIZIONAMENTO  la punta viene portata sopra il punto di ingresso
    2. ALLINEAMENTO    l'ago viene orientato lungo la retta ingresso-lesione
    3. INSERIMENTO     avanzamento lento e rettilineo fino al bersaglio
    4. RETRAZIONE      estrazione lungo la stessa retta e ritorno a riposo

"""

from __future__ import annotations

import csv
import math
import os

import numpy as np

import config as cfg
import robot as rb
from build_scene import connect


# --- Registrazione dei dati -----------------------------------------------


class Logger:
    """Raccoglie lo stato del sistema a ogni passo di simulazione.
    """

    COLUMNS = ["t", "phase", "q1", "q2", "q3", "q4", "q5",
               "tip_x", "tip_y", "tip_z", "error", "depth"]

    def __init__(self):
        self.rows = []

    def record(self, t, phase, q, tip, error):
        depth = cfg.SKIN_Z - tip[2]
        self.rows.append([
            round(t, 4), phase,
            *[round(float(v), 6) for v in q],
            *[round(float(v), 6) for v in tip],
            round(float(error), 6), round(float(depth), 6),
        ])

    def save(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(self.COLUMNS)
            writer.writerows(self.rows)
        print(f"Log salvato in {path} ({len(self.rows)} campioni)")


# --- Generazione del moto -------------------------------------------------


def smooth(u: float) -> float:
    """Profilo di velocità con partenza e arrivo a velocità nulla.
    
    Il profilo cosinusoidale evita gli
    strappi ed è il minimo indispensabile per un moto credibile.
    """
    return 0.5 * (1.0 - math.cos(math.pi * u))


def move(sim, arm, logger, q_start, q_end, duration, phase, profile=smooth):
    """Interpola tra due configurazioni di giunto e avanza la simulazione.

    L'interpolazione avviene nello spazio dei giunti. Durante l'inserimento si muove un solo giunto prismatico,
    quindi un moto lineare nei giunti è anche un moto rettilineo nello spazio
    cartesiano, che è esattamente il requisito clinico.
    """
    q_start = np.asarray(q_start, dtype=float)
    q_end = np.asarray(q_end, dtype=float)

    dt = sim.getSimulationTimeStep()
    steps = max(1, int(round(duration / dt)))

    for i in range(1, steps + 1):
        q = q_start + (q_end - q_start) * profile(i / steps)
        arm.set_joints(q)
        sim.step()
        logger.record(sim.getSimulationTime(), phase, q,
                      arm.tip_position(), arm.targeting_error())

    return q_end


def hold(sim, arm, logger, q, duration, phase):
    """Mantiene la configurazione corrente per la durata indicata."""
    dt = sim.getSimulationTimeStep()
    for _ in range(max(1, int(round(duration / dt)))):
        sim.step()
        logger.record(sim.getSimulationTime(), phase, q,
                      arm.tip_position(), arm.targeting_error())
    return q


def linear(u: float) -> float:
    """Profilo a velocità costante, usato per l'avanzamento nel tessuto."""
    return u


# --- Procedura ------------------------------------------------------------


def run(sim, arm, logger):
    target = arm.target_position()
    q_appr, q_targ = rb.inverse_kinematics(target, cfg.INSERTION_ANGLE)

    problems = rb.check_limits(q_appr) + rb.check_limits(q_targ)
    if problems:
        raise RuntimeError("Configurazione irraggiungibile: " + "; ".join(problems))

    q_home = list(cfg.HOME_Q)

    q_placed = list(q_appr)
    q_placed[3] = 0.0

    print("Fase 1  posizionamento")
    distance = np.linalg.norm(np.array(q_placed[:3]) - np.array(q_home[:3]))
    q = move(sim, arm, logger, q_home, q_placed,
             distance / cfg.POSITIONING_SPEED, "positioning")

    print("Fase 2  allineamento")
    q = move(sim, arm, logger, q, q_appr,
             abs(cfg.INSERTION_ANGLE) / cfg.TILT_SPEED, "alignment")

    print(f"Fase 3  inserimento ({q_targ[4] * 1000:.1f} mm di corsa)")
    q = move(sim, arm, logger, q, q_targ,
             q_targ[4] / cfg.INSERTION_SPEED, "insertion", profile=linear)

    error_mm = arm.targeting_error() * 1000
    print(f"        bersaglio raggiunto, errore {error_mm:.3f} mm")

    print("Fase 3b prelievo")
    q = hold(sim, arm, logger, q, cfg.DWELL_TIME, "dwell")

    print("Fase 4  retrazione")
    q = move(sim, arm, logger, q, q_appr,
             q_targ[4] / cfg.RETRACTION_SPEED, "retraction", profile=linear)
    q = move(sim, arm, logger, q, q_home,
             distance / cfg.POSITIONING_SPEED + 1.0, "homing")

    return error_mm


def main() -> None:
    sim = connect()

    try:
        arm = rb.NeedleRobot(sim)
    except Exception as exc:
        raise RuntimeError(
            "Robot non trovato nella scena. Esegui prima build_scene.py."
        ) from exc

    logger = Logger()

    # Modalità stepping: la simulazione avanza solo su comando, quindi
    # ogni campione del log corrisponde a un passo esatto e i tempi sono
    # riproducibili indipendentemente dalla macchina su cui gira.
    sim.setStepping(True)
    sim.startSimulation()

    try:
        error_mm = run(sim, arm, logger)
    finally:
        sim.stopSimulation()
        while sim.getSimulationState() != sim.simulation_stopped:
            pass

    logger.save(cfg.LOG_PATH)

    duration = logger.rows[-1][0]
    print(f"\nProcedura completata in {duration:.1f} s simulati")
    print(f"Errore finale di targeting: {error_mm:.3f} mm")


if __name__ == "__main__":
    main()