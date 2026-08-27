"""Parametri geometrici della scena.

Questo modulo è l'unica sorgente per la costruzione della scena
e per il controllo del robot, così una modifica si propaga
ovunque senza disallineamenti.

Sistema di riferimento mondo:
    X = profondità (verso il "paziente")
    Y = lateralità
    Z = verticale, positivo verso l'alto
"""

import numpy as np


# --- Contenitore -----------------------------------------------------------
# Tutti gli oggetti creati vengono appesi a questo dummy, così la pulizia
# della scena si riduce alla rimozione di un singolo sottoalbero.
SCENE_ROOT = "NeedleScene"
 
# --- Phantom di tessuto ----------------------------------------------------
# Blocco che rappresenta il tessuto molle in cui l'ago viene inserito.
PHANTOM_SIZE = [0.30, 0.30, 0.12]          # larghezza, profondità, altezza
PHANTOM_CENTER = [0.50, 0.00, 0.06]        # appoggiato sul pavimento
PHANTOM_COLOR = [0.95, 0.78, 0.70]         # tonalità carne
PHANTOM_TRANSPARENCY = 0.55                # semitrasparente: si vede la lesione
 
# Quota della superficie cutanea: è il piano su cui si trova il punto
# di ingresso dell'ago.
SKIN_Z = PHANTOM_CENTER[2] + PHANTOM_SIZE[2] / 2.0
 
# --- Lesione target --------------------------------------------------------
# Sferetta interna al phantom: è il bersaglio da raggiungere con la punta.
LESION_RADIUS = 0.008
LESION_POSITION = [0.52, 0.015, 0.045]     # ~7.5 cm sotto la superficie
LESION_COLOR = [0.80, 0.10, 0.10]
 
# --- Geometria del robot ---------------------------------------------------
# Quota della guida orizzontale del gantry. L'origine dei tre giunti
# prismatici coincide con [0, 0, GANTRY_Z]: di conseguenza i valori dei
# primi tre giunti sono, letteralmente, le coordinate cartesiane del punto
# di pivot rispetto a quell'origine.
GANTRY_Z = 0.60
 
# Lunghezza dell'ago, dal punto di pivot alla punta a corsa di
# inserimento nulla.
NEEDLE_LENGTH = 0.15
NEEDLE_DIAMETER = 0.003
 
# Corse ammesse dei giunti: [minimo, escursione]
JOINT_LIMITS = {
    "X": [-0.10, 1.00],
    "Y": [-0.30, 0.60],
    "Z": [-0.55, 0.55],
    "Tilt": [-1.5708, 3.1416],
    "Ins": [0.00, 0.30],
}
 
# Configurazione di riposo del robot.
HOME_Q = [0.20, 0.00, -0.10, 0.0, 0.0]
 
# --- Parametri della procedura --------------------------------------------
# Angolo di inserimento rispetto alla verticale, nel piano XZ.
# 0 = ago perfettamente verticale; positivo = punta inclinata verso -X.
INSERTION_ANGLE = np.deg2rad(20.0)
 
# Distanza di sicurezza tra la punta dell'ago e la cute prima di iniziare
# l'inserimento vero e proprio.
STANDOFF = 0.03
 
# --- Velocità della procedura ---------------------------------------------
# Le velocità non sono arbitrarie. Il posizionamento avviene in aria e può
# essere rapido; l'inserimento nel tessuto è deliberatamente lento, perché
# nella pratica clinica la velocità di avanzamento influenza la deflessione
# dell'ago e il trauma sul tessuto. La retrazione può essere il doppio più
# rapida dell'inserimento, dato che l'ago ripercorre un tramite già aperto.
POSITIONING_SPEED = 0.08      # m/s, traslazioni in aria
TILT_SPEED = 0.35             # rad/s, allineamento
INSERTION_SPEED = 0.005       # m/s, avanzamento nel tessuto
RETRACTION_SPEED = 0.010      # m/s, estrazione
 
# Sosta a bersaglio raggiunto: rappresenta il tempo del prelievo bioptico.
DWELL_TIME = 1.5              # s
 
# File di log prodotto da main.py
LOG_PATH = "results/run_log.csv"
 
 
def entry_point() -> np.ndarray:
    """Punto di ingresso cutaneo, calcolato dal target e dall'angolo.
 
    Il punto di ingresso è l'intersezione tra la superficie
    cutanea e la retta che passa per la lesione con l'inclinazione scelta.
    Imponendo che l'ingresso abbia la stessa coordinata Y del target, la
    traiettoria giace interamente nel piano XZ e il problema si risolve in
    forma chiusa, senza cinematica inversa numerica.
    """
    target = np.array(LESION_POSITION, dtype=float)
    depth = (SKIN_Z - target[2]) / np.cos(INSERTION_ANGLE)
    direction = np.array([np.sin(INSERTION_ANGLE), 0.0, np.cos(INSERTION_ANGLE)])
    return target + depth * direction
 
 
def insertion_depth() -> float:
    """Lunghezza di ago da far avanzare, dalla cute alla lesione."""
    return float(np.linalg.norm(entry_point() - np.array(LESION_POSITION)))
 
 
if __name__ == "__main__":
    e = entry_point()
    print(f"Superficie cutanea a z = {SKIN_Z:.3f} m")
    print(f"Punto di ingresso     = [{e[0]:.4f}, {e[1]:.4f}, {e[2]:.4f}]")
    print(f"Profondità di inserimento = {insertion_depth() * 1000:.1f} mm")
 