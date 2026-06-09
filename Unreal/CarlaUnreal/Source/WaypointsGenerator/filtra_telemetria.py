# -*- coding: utf-8 -*-
"""
filtra_telemetria.py  —  v7

Piazza waypoint in base a quanto RUOTA effettivamente il veicolo:
  - Accumula il delta di yaw mentre guidi
  - Mette un WP ogni volta che lo yaw supera SOGLIA_GRADI_CURVA (curve dense)
  - Mette comunque un WP ogni MAX_DIST_RETTILINEO metri (rettilinei sparsi)
  - Con 2-3 CSV calcola la media delle traiettorie prima di campionare

NOVITA v7: rileva automaticamente il formato numerico del CSV
  - Formato italiano  (virgola=decimale, punto=migliaia): es. 28.774,918
  - Formato anglosassone (punto=decimale, virgola=migliaia): es. 34,554.355
  Funziona su entrambi senza modifiche manuali.

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

FILE_CSV_1  = r"C:\Carla_VR_2026\Carla_UE5-Polito\Unreal\CarlaUnreal\Saved\Telemetria\1\Buone\1_Demo_Male_NoVirtualCarpet_2026-06-09_11-47-56.csv"
FILE_CSV_2  = r"C:\Carla_VR_2026\Carla_UE5-Polito\Unreal\CarlaUnreal\Saved\Telemetria\1\Buone\1_Demo_Male_NoVirtualCarpet_2026-06-09_11-54-27.csv"
FILE_CSV_3  = r"C:\Carla_VR_2026\Carla_UE5-Polito\Unreal\CarlaUnreal\Saved\Telemetria\1\Buone\1_Demo_Male_NoVirtualCarpet_2026-06-09_11-59-27.csv"

FILE_OUTPUT = r"C:\Carla_VR_2026\Carla_UE5-Polito\Unreal\CarlaUnreal\Source\WaypointsGenerator\waypoint_unreal.json"

SOGLIA_PARTENZA_METRI = 40.0   # ignora i primi N metri (veicolo fermo)
MAX_WAYPOINTS         = 80     # numero massimo di waypoint nel JSON

# Soglia angolare: mette un WP ogni volta che lo yaw cambia di questi gradi
# Valore basso = piu WP nelle curve  |  valore alto = meno WP nelle curve
SOGLIA_GRADI_CURVA  = 10.0   # gradi

# Distanza massima in rettilineo senza waypoint
MAX_DIST_RETTILINEO = 200.0  # metri

# Distanza minima tra due waypoint consecutivi (evita duplicati in curva)
MIN_DIST_WP         = 20.0   # metri

# =============================================================================


def rileva_formato(file_csv):
    """
    Legge le prime righe del CSV e rileva il formato numerico:
      'en' = anglosassone (punto=decimale, virgola=migliaia): 34,554.355
      'it' = italiano     (virgola=decimale, punto=migliaia): 28.774,918
    """
    with open(file_csv, 'r', encoding='utf-8') as f:
        righe = list(csv.DictReader(f, delimiter=';'))[:8]
    en = 0; it = 0
    for row in righe:
        for val in row.values():
            if not val: continue
            v = str(val).strip()
            if ',' in v and '.' in v:
                if v.rfind('.') > v.rfind(','): en += 1   # punto dopo virgola = decimale EN
                else:                           it += 1   # virgola dopo punto  = decimale IT
    return 'en' if en >= it else 'it'


def parse_valore(v, fmt):
    """Converte un valore stringa nel formato rilevato."""
    if not v: return 0.0
    v = str(v).strip()
    try:
        if fmt == 'en':
            return float(v.replace(',', ''))              # rimuove sep. migliaia
        else:
            return float(v.replace('.', '').replace(',', '.'))  # italiano
    except:
        return 0.0


def dist3d_m(p1, p2):
    """Distanza euclidea in METRI tra due dict con PX/PY/PZ in cm."""
    return math.sqrt(
        (p1['PX'] - p2['PX'])**2 +
        (p1['PY'] - p2['PY'])**2 +
        (p1['PZ'] - p2['PZ'])**2
    ) / 100.0


def delta_yaw(y1, y2):
    """Delta angolare minimo tra due yaw in gradi."""
    d = abs(y2 - y1) % 360
    return d if d <= 180 else 360 - d


def media_angoli(angoli):
    """Media circolare corretta per angoli vicini a +/-180."""
    ss = sum(math.sin(math.radians(a)) for a in angoli)
    cs = sum(math.cos(math.radians(a)) for a in angoli)
    return math.degrees(math.atan2(ss, cs))


# =============================================================================
# STEP 1: lettura CSV
# =============================================================================
def carica_csv(file_csv):
    fmt = rileva_formato(file_csv)
    tutti = []
    with open(file_csv, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f, delimiter=';')
        for row in reader:
            try:
                tutti.append({
                    'PX':    parse_valore(row.get('PX',  0), fmt),
                    'PY':    parse_valore(row.get('PY',  0), fmt),
                    'PZ':    parse_valore(row.get('PZ',  0), fmt),
                    'Yaw':   parse_valore(row.get('VRZ', 0), fmt),
                    'Speed': math.sqrt(
                        parse_valore(row.get('VX', 0), fmt)**2 +
                        parse_valore(row.get('VY', 0), fmt)**2 +
                        parse_valore(row.get('VZ', 0), fmt)**2
                    )
                })
            except Exception:
                continue

    # Auto-rileva CSV con due veicoli alternati
    campione = [dist3d_m(tutti[i], tutti[i+1]) for i in range(min(200, len(tutti)-1))]
    mediana  = statistics.median(campione)
    dati = tutti[::2] if mediana > 2.0 else tutti

    dist_prog = [0.0]
    for i in range(1, len(dati)):
        dist_prog.append(dist_prog[-1] + dist3d_m(dati[i-1], dati[i]))

    nome = file_csv.split('\\')[-1].split('/')[-1]
    tipo = "due veicoli -> righe pari" if mediana > 2.0 else "CSV pulito"
    print(f"    {nome}: {len(dati)} frame, {dist_prog[-1]:.0f}m  [fmt={fmt}, {tipo}]")
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
                elif dist_prog[i] > target + step*2: break
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
                tenendo i piu 'importanti' (piu curvi) in ogni bucket.
    """
    idx_start = 0
    for i, d in enumerate(dist_prog):
        if d >= SOGLIA_PARTENZA_METRI:
            idx_start = i; break

    candidati = [{'idx': idx_start, 'yaw_acc': 0.0, 'dist_acc': 0.0}]
    yaw_acc = 0.0; dist_acc = 0.0

    for i in range(idx_start + 1, len(dati)):
        yaw_acc  += delta_yaw(dati[i-1]['Yaw'], dati[i]['Yaw'])
        dist_acc += dist3d_m(dati[i-1], dati[i])

        metti = False
        if yaw_acc >= SOGLIA_GRADI_CURVA and dist_acc >= MIN_DIST_WP:
            metti = True
        elif dist_acc >= MAX_DIST_RETTILINEO:
            metti = True

        if metti:
            candidati.append({'idx': i, 'yaw_acc': yaw_acc, 'dist_acc': dist_acc})
            yaw_acc = 0.0; dist_acc = 0.0

    print(f"  Candidati angolari: {len(candidati)}")

    if len(candidati) > MAX_WAYPOINTS:
        for c in candidati:
            c['curv'] = c['yaw_acc'] / max(c['dist_acc'], 0.001)
        bucket_size = len(candidati) / MAX_WAYPOINTS
        selezionati = []
        for k in range(MAX_WAYPOINTS):
            start_b = int(k * bucket_size)
            end_b   = int((k+1) * bucket_size)
            bucket  = candidati[start_b : max(end_b, start_b+1)]
            selezionati.append(max(bucket, key=lambda c: c['curv']))
        candidati = selezionati

    print(f"  WP selezionati    : {len(candidati)}")

    waypoints = []
    for k, cand in enumerate(candidati):
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
    print("=== filtra_telemetria.py  v7 ===\n")

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

    print(f"\n[3/3] Campiono con soglia {SOGLIA_GRADI_CURVA}° / {MAX_DIST_RETTILINEO}m rettilineo...")
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