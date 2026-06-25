# -*- coding: utf-8 -*-
# spawn_waypoints.py  �  UE 5.5 compatible
#
# Legge il JSON generato da filtra_telemetria.py e spawna i Blueprint
# waypoint nel livello attualmente aperto in Unreal Editor.
#
# Come lanciarlo dal cmd interno di Unreal:
#   py "C:/Dev/CarlaUE5/Unreal/CarlaUnreal/Source/CarlaUnreal/FiltroTelemetria/spawn_waypoints.py"

import unreal
import json

# =============================================================================
# PARAMETRI � modifica questi
# =============================================================================

JSON_PATH = "D:/lab2a/Downloads/FiltroTelemetria/FiltroTelemetria/waypoint_unreal.json"

# Path interno Unreal del tuo Blueprint waypoint.
# Aprilo nel Content Browser, tasto destro -> "Copy Reference" e incolla qui
# (senza il suffisso _C, lo aggiunge lo script automaticamente)
BLUEPRINT_PATH ='/CarlaVR_Vehicles/ARHUD_TestCase/VirtualCarpet/BP_Nav_Waypoint.BP_Nav_Waypoint'

# Altezza aggiuntiva in cm sopra la posizione registrata (0 = nessun offset)
OFFSET_Z_CM = 10.0

# =============================================================================


def spawn_waypoints():
    # --- Carica il JSON ---
    try:
        with open(JSON_PATH, 'r', encoding='utf-8') as f:
            waypoints = json.load(f)
    except Exception as e:
        unreal.log_error("[SpawnWaypoints] Impossibile leggere il JSON: " + str(e))
        return

    unreal.log("[SpawnWaypoints] Letti " + str(len(waypoints)) + " waypoint")

    # --- Carica la classe Blueprint (suffisso _C = classe compilata) ---
    percorso_pulito = BLUEPRINT_PATH.replace("Blueprint'", "").replace("'", "").strip()
    bp_class = unreal.load_class(None, percorso_pulito + "_C")
    if bp_class is None:
        unreal.log_error("[SpawnWaypoints] Classe non trovata: " + percorso_pulito + "_C")
        unreal.log_error("  -> Tasto destro sul Blueprint nel Content Browser -> Copy Reference")
        return

    # --- Subsystem UE5.5 (sostituisce EditorLevelLibrary deprecato) ---
    level_subsystem = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    world = level_subsystem.get_current_level()
    if world is None:
        unreal.log_error("[SpawnWaypoints] Nessun livello aperto.")
        return

    spawned = 0
    errori  = 0

    with unreal.ScopedEditorTransaction("Spawn Waypoints da CSV") as trans:
        with unreal.ScopedSlowTask(len(waypoints), "Spawn waypoint in corso...") as task:
            task.make_dialog(True)

            for wp in waypoints:
                task.enter_progress_frame(1, "Spawning " + wp['Name'] + "...")

                location = unreal.Vector(wp['X'], wp['Y'], wp['Z'] + OFFSET_Z_CM)
                rotation = unreal.Rotator(0.0, wp['Yaw'], 0.0)

                # In UE5.5 si usa spawn_actor_from_class direttamente sul world
                actor = unreal.EditorLevelLibrary.spawn_actor_from_class(
                    bp_class,
                    location,
                    rotation
                )

                if actor is None:
                    unreal.log_warning("[SpawnWaypoints] Spawn fallito per " + wp['Name'])
                    errori += 1
                    continue

                actor.set_actor_label(wp['Name'])

                # Imposta TargetSpeed
                try:
                    actor.set_editor_property("TargetSpeed", wp['TargetSpeed'])
                except Exception as e:
                    unreal.log_warning("[SpawnWaypoints] TargetSpeed non impostato su " + wp['Name'] + ": " + str(e))

                # Imposta Number
                try:
                    actor.set_editor_property("Number", wp['Number'])
                except Exception as e:
                    unreal.log_warning("[SpawnWaypoints] Number non impostato su " + wp['Name'] + ": " + str(e))

                spawned += 1

    # Salva il livello
    level_subsystem.save_current_level()

    unreal.log("[SpawnWaypoints] Completato: " + str(spawned) + " spawnati, " + str(errori) + " errori.")


spawn_waypoints()