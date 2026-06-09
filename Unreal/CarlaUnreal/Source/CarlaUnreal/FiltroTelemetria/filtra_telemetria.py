# -*- coding: utf-8 -*-
"""
filtra_telemetria.py  —  v6  (soglia angolare dal CSV)

Piazza waypoint in base a quanto RUOTA effettivamente il veicolo:
  - Accumula il delta di yaw metro per metro mentre guidi
  - Appena lo yaw accumulato supera SOGLIA_GRADI mette un waypoint (= curva rilevata)
  - In rettilineo mette comunque un waypoint ogni MAX_DIST_RETTILINEO metri
  - Se i candidati sono piu di MAX_WAYPOINTS, li riduce tenendo
    quelli piu "importanti" (piu alta curvatura nel loro bucket)

Vantaggi rispetto alle versioni precedenti:
  - Non dipende dallo XODR per trovare le curve: le rileva direttamente
    dal comportamento reale del veicolo nel CSV
  - In una curva a 30° mette tanti piu punti che in una curva a 5°
  - Funziona anche se il CSV copre solo una parte del tracciato

Con 2-3 CSV: calcola la media delle traiettorie prima di campionare.

Unita dati CSV (Carla/Unreal):
  - PX, PY, PZ : cm  (standard Unreal Engine)
  - VX, VY, VZ : km/h (magnitudine = velocita)
  - VRZ         : yaw in gradi (-180 / +180)
"""

import csv
import json
import math
import statistics

# =============================================================================
# PARAMETRI — modifica solo questi
# =============================================================================

FILE_CSV_1  = r"C:\Dev\CarlaUE5\Unreal\CarlaUnreal\Source\CarlaUnreal\FiltroTelemetria\1_Demo_Male_VirtualCarpet_2026-06-08_15-25-09.csv"
FILE_CSV_2  = r""
FILE_CSV_3  = r""

FILE_OUTPUT = r"C:\Dev\CarlaUE5\Unreal\CarlaUnreal\Source\CarlaUnreal\FiltroTelemetria\waypoint_unreal.json"

SOGLIA_PARTENZA_METRI = 40.0   # ignora i primi N metri (veicolo fermo)
MAX_WAYPOINTS         = 80     # numero esatto di waypoint nel JSON

# Soglia angolare: mette un WP ogni volta che lo yaw cambia di questi gradi
# Valore basso = piu WP nelle curve  |  valore alto = meno WP nelle curve
# Consigliato: 3-5 gradi
SOGLIA_GRADI_CURVA = 10.0

# Distanza massima in rettilineo senza waypoint
# Garantisce che non ci siano "buchi" troppo lunghi anche in rettilineo
MAX_DIST_RETTILINEO = 200.0   # metri

# Distanza minima tra due waypoint consecutivi (evita punti sovrapposti)
MIN_DIST_WP = 20   # metri

# =============================================================================


def parse_numero(v):
    """Converte formato italiano: '29.092,049' -> 29092.049"""
    if not v: return 0.0
    try: return float(str(v).strip().replace('.','').replace(',','.'))
    except: return 0.0

def dist3d_m(p1, p2):
    """Distanza euclidea in METRI tra due dict con PX/PY/PZ in cm."""
    return math.sqrt(
        (p1['PX']-p2['PX'])**2 +
        (p1['PY']-p2['PY'])**2 +
        (p1['PZ']-p2['PZ'])**2
    ) / 100.0

def delta_yaw(y1, y2):
    """Delta angolare minimo tra due yaw in gradi."""
    d = abs(y2-y1) % 360
    return d if d <= 180 else 360-d

def media_angoli(angoli):
    """Media circolare corretta per angoli vicini a +/-180."""
    sin_s = sum(math.sin(math.radians(a)) for a in angoli)
    cos_s = sum(math.cos(math.radians(a)) for a in angoli)
    return math.degrees(math.atan2(sin_s, cos_s))


# =============================================================================
# STEP 1: lettura CSV con auto-rilevazione struttura
# =============================================================================
def carica_csv(file_csv):
    tutti = []
    with open(file_csv, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f, delimiter=';')
        for row in reader:
            try:
                tutti.append({
                    'PX':    parse_numero(row.get('PX',  0)),
                    'PY':    parse_numero(row.get('PY',  0)),
                    'PZ':    parse_numero(row.get('PZ',  0)),
                    'Yaw':   parse_numero(row.get('VRZ', 0)),
                    'Speed': math.sqrt(
                        parse_numero(row.get('VX', 0))**2 +
                        parse_numero(row.get('VY', 0))**2 +
                        parse_numero(row.get('VZ', 0))**2
                    )
                })
            except Exception:
                continue

    # Auto-rileva CSV con due veicoli alternati (es. CSV di test)
    campione = [dist3d_m(tutti[i], tutti[i+1]) for i in range(min(200, len(tutti)-1))]
    mediana  = statistics.median(campione)
    dati = tutti[::2] if mediana > 2.0 else tutti

    dist_prog = [0.0]
    for i in range(1, len(dati)):
        dist_prog.append(dist_prog[-1] + dist3d_m(dati[i-1], dati[i]))

    nome = file_csv.split('\\')[-1].split('/')[-1]
    tipo = "due veicoli -> righe pari" if mediana > 2.0 else "CSV pulito"
    print(f"    {nome}: {len(dati)} frame, {dist_prog[-1]:.0f}m  [{tipo}]")
    return dati, dist_prog


# =============================================================================
# STEP 2: normalizzazione e media (per 2-3 CSV)
# =============================================================================
def normalizza_e_media(lista_csv):
    N_CAMPIONI  = 2000
    dist_minima = min(dp[-1] for _, dp in lista_csv)
    print(f"  Distanza comune: {dist_minima:.0f}m  |  campioni interni: {N_CAMPIONI}")

    normalizzati = []
    for dati, dist_prog in lista_csv:
        idx_start  = next((i for i,d in enumerate(dist_prog) if d >= SOGLIA_PARTENZA_METRI), 0)
        dist_start = dist_prog[idx_start]
        step       = (min(dist_prog[-1], dist_minima) - dist_start) / N_CAMPIONI
        campioni   = []
        for k in range(N_CAMPIONI):
            target   = dist_start + k * step
            best_idx = idx_start
            best_d   = abs(dist_prog[idx_start] - target)
            for i in range(idx_start, len(dati)):
                d = abs(dist_prog[i] - target)
                if d < best_d: best_d=d; best_idx=i
                elif dist_prog[i] > target+step*2: break
            campioni.append(dati[best_idx])
        normalizzati.append(campioni)

    n_csv = len(normalizzati)
    media = []
    for k in range(N_CAMPIONI):
        pk = [normalizzati[c][k] for c in range(n_csv)]
        media.append({
            'PX':    sum(p['PX']    for p in pk) / n_csv,
            'PY':    sum(p['PY']    for p in pk) / n_csv,
            'PZ':    sum(p['PZ']    for p in pk) / n_csv,
            'Yaw':   media_angoli([p['Yaw']   for p in pk]),
            'Speed': sum(p['Speed'] for p in pk) / n_csv,
        })

    dist_media = [0.0]
    for i in range(1, len(media)):
        dist_media.append(dist_media[-1] + dist3d_m(media[i-1], media[i]))

    print(f"  Traiettoria media: {dist_media[-1]:.0f}m")
    return media, dist_media


# =============================================================================
# STEP 3: campionamento con soglia angolare
# =============================================================================
def campiona_angolare(dati, dist_prog):
    """
    1a passata: genera candidati ogni SOGLIA_GRADI_CURVA di rotazione
                o ogni MAX_DIST_RETTILINEO metri.
    2a passata: se i candidati sono piu di MAX_WAYPOINTS, riduce
                a MAX_WAYPOINTS tenendo i piu 'importanti' (piu curvi)
                in ogni bucket di distanza uniforme.
    """
    # Trova indice di partenza
    idx_start = 0
    for i, d in enumerate(dist_prog):
        if d >= SOGLIA_PARTENZA_METRI:
            idx_start = i
            break

    # --- 1a passata: genera candidati ---
    candidati = [{
        'idx':      idx_start,
        'yaw_acc':  0.0,
        'dist_acc': 0.0,
    }]
    yaw_acc  = 0.0
    dist_acc = 0.0

    for i in range(idx_start + 1, len(dati)):
        dy = delta_yaw(dati[i-1]['Yaw'], dati[i]['Yaw'])
        dd = dist3d_m(dati[i-1], dati[i])
        yaw_acc  += dy
        dist_acc += dd

        metti = False
        if yaw_acc >= SOGLIA_GRADI_CURVA and dist_acc >= MIN_DIST_WP:
            metti = True
        elif dist_acc >= MAX_DIST_RETTILINEO:
            metti = True

        if metti:
            candidati.append({
                'idx':      i,
                'yaw_acc':  yaw_acc,
                'dist_acc': dist_acc,
            })
            yaw_acc  = 0.0
            dist_acc = 0.0

    print(f"  Candidati angolari: {len(candidati)}")

    # --- 2a passata: riduci a MAX_WAYPOINTS se necessario ---
    if len(candidati) <= MAX_WAYPOINTS:
        selezionati = candidati
    else:
        # Calcola curvatura (gradi/metro) per ogni candidato
        for c in candidati:
            c['curv'] = c['yaw_acc'] / max(c['dist_acc'], 0.001)

        # Divide i candidati in MAX_WAYPOINTS bucket uniformi per distanza
        # In ogni bucket mantiene il candidato con la curvatura piu alta
        bucket_size = len(candidati) / MAX_WAYPOINTS
        selezionati = []
        for k in range(MAX_WAYPOINTS):
            start_b = int(k * bucket_size)
            end_b   = int((k+1) * bucket_size)
            bucket  = candidati[start_b : max(end_b, start_b+1)]
            best    = max(bucket, key=lambda c: c['curv'])
            selezionati.append(best)

    print(f"  WP selezionati    : {len(selezionati)}")

    # Costruisci output
    waypoints = []
    for k, cand in enumerate(selezionati):
        p = dati[cand['idx']]
        waypoints.append({
            "Name":        f"WP_{k}",
            "Number":      k,
            "X":           round(p['PX'],    3),
            "Y":           round(p['PY'],    3),
            "Z":           round(p['PZ'],    3),
            "Yaw":         round(p['Yaw'],   3),
            "TargetSpeed": round(p['Speed'], 2)
        })

    return waypoints


# =============================================================================
# MAIN
# =============================================================================
def elabora():
    print("=== filtra_telemetria.py  v6 (soglia angolare) ===\n")

    file_csv_list = [f for f in [FILE_CSV_1, FILE_CSV_2, FILE_CSV_3] if f.strip()]
    if not file_csv_list:
        print("ERRORE: nessun FILE_CSV specificato."); return

    print(f"[1/3] Carico {len(file_csv_list)} CSV...")
    lista_csv = [carica_csv(f) for f in file_csv_list]

    print(f"\n[2/3] Calcolo traiettoria di riferimento...")
    if len(lista_csv) == 1:
        dati_fin, dist_fin = lista_csv[0]
        idx = next((i for i,d in enumerate(dist_fin) if d >= SOGLIA_PARTENZA_METRI), 0)
        dati_fin = dati_fin[idx:]
        dist_fin = [d - dist_fin[idx] for d in dist_fin[idx:]]
        print(f"  CSV singolo: {dist_fin[-1]:.0f}m utili")
    else:
        dati_fin, dist_fin = normalizza_e_media(lista_csv)

    print(f"\n[3/3] Campiono con soglia angolare {SOGLIA_GRADI_CURVA}° / {MAX_DIST_RETTILINEO}m rettilineo...")
    waypoints = campiona_angolare(dati_fin, dist_fin)

    with open(FILE_OUTPUT, 'w', encoding='utf-8') as f:
        json.dump(waypoints, f, indent=4)

    gaps = [
        math.sqrt((waypoints[i]['X']-waypoints[i-1]['X'])**2 +
                  (waypoints[i]['Y']-waypoints[i-1]['Y'])**2) / 100.0
        for i in range(1, len(waypoints))
    ]

    print(f"\n=== Completato! ===")
    print(f"  Waypoint generati : {len(waypoints)}")
    print(f"  Gap tra WP        : min={min(gaps):.1f}m  |  max={max(gaps):.1f}m  |  media={sum(gaps)/len(gaps):.1f}m")
    print(f"  Output            : {FILE_OUTPUT}")


elabora()