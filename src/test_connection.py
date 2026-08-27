"""Verifica della connessione tra Python e CoppeliaSim.
"""

import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


def main() -> None:
    print("Connessione a CoppeliaSim...")
    client = RemoteAPIClient()
    sim = client.require("sim")
    print("Connesso.")

    # Modalità stepping: la simulazione avanza solo quando lo chiediamo noi.
    # Serve per avere un controllo deterministico delle traiettorie.
    sim.setStepping(True)
    sim.startSimulation()

    for i in range(20):
        t = sim.getSimulationTime()
        print(f"step {i:02d}  t = {t:.3f} s")
        sim.step()
        time.sleep(0.05)

    sim.stopSimulation()

    # Attesa dell'effettivo arresto prima di chiudere.
    while sim.getSimulationState() != sim.simulation_stopped:
        time.sleep(0.05)

    print("Simulazione arrestata. Tutto funziona.")


if __name__ == "__main__":
    main()