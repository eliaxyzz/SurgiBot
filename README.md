# Needle Insertion Robot — CoppeliaSim + Python

Simulazione di un robot per inserimento automatico di ago (biopsia / needle
insertion) sviluppata in CoppeliaSim e controllata via Python attraverso la
ZMQ Remote API.

## Descrizione

Il robot è un manipolatore cartesiano a 5 gradi di libertà (3 prismatici per il
posizionamento, 1 rotoidale per l'orientamento dell'ago, 1 prismatico per
l'avanzamento). Questa architettura è quella tipicamente adottata dai robot
reali per biopsia sotto guida di immagini, perché disaccoppia il
posizionamento dall'inserimento e rende la cinematica inversa analitica.

La scena non è un file binario: viene interamente costruita da script
(`src/build_scene.py`), così da essere versionabile e riproducibile.

## Sequenza operativa

1. **Posizionamento** — la punta dell'ago viene portata sopra il punto di
   ingresso cutaneo.
2. **Allineamento** — l'ago viene orientato lungo la retta ingresso-lesione.
3. **Inserimento** — avanzamento lineare a velocità ridotta fino alla
   profondità calcolata.
4. **Retrazione** — estrazione lungo la stessa traiettoria e ritorno a home.

## Requisiti

- CoppeliaSim 4.6 o superiore (Edu)
- Python 3.10+
- Dipendenze: vedi `requirements.txt`

## Installazione

```bash
git clone <url-del-repo>
cd SurgiBot
python3 -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

## Utilizzo

Avviare CoppeliaSim e lasciarlo aperto su una scena vuota, poi:

```bash
python src/test_connection.py    # verifica della connessione
python src/build_scene.py        # costruzione della scena
python src/main.py               # esecuzione della procedura
```

## Struttura

```
src/
  test_connection.py   verifica della connessione alla Remote API
  build_scene.py       costruzione programmatica di robot e ambiente
  robot.py             cinematica e controllo dei giunti
  main.py              macchina a stati della procedura
  plots.py             grafici dei risultati
docs/                  presentazione e materiale di supporto
results/               log ed elaborati generati dalle esecuzioni
```

## Licenza

MIT