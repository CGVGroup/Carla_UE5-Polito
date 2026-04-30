import time
import carla

def main():
    print("Connessione a CARLA...")

    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    world = client.get_world()
    print("Connesso al mondo:", world.get_map().name)

    vehicle = None

    print("⏳ In attesa del veicolo CarlaVR...")

    while vehicle is None:
        actors = world.get_actors()

        for actor in actors:
            if actor.type_id.startswith("vehicle"):
                vehicle = actor
                break

        if vehicle is None:
            time.sleep(0.5)

    print("✅ Veicolo trovato!")
    print(" - type_id:", vehicle.type_id)
    print(" - id:", vehicle.id)

    vehicle.set_autopilot(True)
    print("▶ Autopilot attivato")

    print("🟢 Controllo attivo. CTRL+C per uscire")
    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()
