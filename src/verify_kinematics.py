"""Verifica che la cinematica analitica coincida con quella simulata.

Imponendo la configurazione calcolata da inverse_kinematics, la punta letta
da CoppeliaSim deve trovarsi sul centro della lesione. Se i due valori non
coincidono, l'errore è negli orientamenti dei giunti nella scena, non nella
matematica.
"""

import numpy as np

import config as cfg
import robot as rb
from build_scene import connect


def report(name, sim_pos, analytic_pos) -> float:
    delta = float(np.linalg.norm(sim_pos - analytic_pos))
    print(f"\n{name}")
    print(f"  punta simulata  = [{sim_pos[0]:.4f}, {sim_pos[1]:.4f}, {sim_pos[2]:.4f}]")
    print(f"  punta analitica = [{analytic_pos[0]:.4f}, {analytic_pos[1]:.4f}, {analytic_pos[2]:.4f}]")
    print(f"  scarto = {delta * 1000:.3f} mm")
    return delta


def main() -> None:
    sim = connect()
    arm = rb.NeedleRobot(sim)

    target = arm.target_position()
    q_appr, q_targ = rb.inverse_kinematics(target, cfg.INSERTION_ANGLE)

    worst = 0.0

    arm.set_joints(q_appr)
    worst = max(worst, report("Avvicinamento",
                              arm.tip_position(),
                              rb.forward_kinematics(q_appr)))

    arm.set_joints(q_targ)
    worst = max(worst, report("A bersaglio",
                              arm.tip_position(),
                              rb.forward_kinematics(q_targ)))

    print(f"\nErrore di targeting misurato: {arm.targeting_error() * 1000:.3f} mm")

    arm.set_joints(cfg.HOME_Q)

    if worst < 1e-4:
        print("\nCinematica coerente: la scena riproduce il modello analitico.")
    else:
        print("\nDisallineamento: controllare gli orientamenti dei giunti.")


if __name__ == "__main__":
    main()