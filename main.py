import subprocess
import time
# import threading # Not strictly needed for this version

def start_simulator():
    """Запускает 3D симулятор (Panda3D) как отдельный процесс."""
    print("Starting 3D Simulator (simulator/drone_simulator_3d.py)...")
    try:
        # Ensure this path is correct relative to the project root
        subprocess.Popen(['python', 'simulator/drone_simulator_3d.py'])
        print("Simulator process started.")
    except FileNotFoundError:
        print("Error: simulator/drone_simulator_3d.py not found. Make sure it's in the correct path.")
    except Exception as e:
        print(f"Error starting simulator: {e}")

def start_controller():
    """Запускает контроллер команд DroneKit (пока не реализован)."""
    print("Attempting to start Drone Controller (controller/drone_controller.py)...")
    try:
        # This file doesn't exist yet, so this will likely fail or do nothing if called.
        # For now, we can comment out the Popen or add a check.
        # subprocess.Popen(['python', 'controller/drone_controller.py'])
        print("Controller process start initiated (if controller file existed). Currently a placeholder.")
    except FileNotFoundError:
        print("Error: controller/drone_controller.py not found. Controller not started.")
    except Exception as e:
        print(f"Error starting controller: {e}")


def start_bridge():
    """Запускает MAVLink мост как отдельный процесс."""
    print("Starting MAVLink Bridge (communication/mavlink_bridge.py)...")
    try:
        subprocess.Popen(['python', 'communication/mavlink_bridge.py'])
        print("MAVLink Bridge process started.")
    except FileNotFoundError:
        print("Error: communication/mavlink_bridge.py not found. Make sure it's in the correct path.")
    except Exception as e:
        print(f"Error starting MAVLink bridge: {e}")

if __name__ == "__main__":
    print("Main application started.")
    
    print("\n--- Starting MAVLink Bridge ---")
    start_bridge()
    print("Waiting for MAVLink Bridge to initialize (2 seconds)...")
    time.sleep(2)  # Даем время мосту запуститься
    
    print("\n--- Starting Simulator ---")
    start_simulator()
    print("Waiting for Simulator to initialize (2 seconds)...")
    time.sleep(2)  # Даем время симулятору запуститься
    
    # print("\n--- Starting Controller ---")
    # start_controller() # Контроллер пока не реализован, вызов закомментирован
    # print("Controller part is commented out as it's not yet implemented.")
    
    print("\nAll components initiated. Main script will now idle or exit if components run in background.")
    # Если все запускается через Popen, этот скрипт может завершиться,
    # а дочерние процессы продолжат работать.
    # Для удержания основного скрипта активным (если нужно), можно добавить:
    # try:
    #     while True:
    #         time.sleep(10) # Просто держим главный поток живым
    # except KeyboardInterrupt:
    #     print("Main application terminated by user.")
    #     # Здесь можно было бы добавить логику для корректного завершения дочерних процессов,
    #     # но Popen по умолчанию их не убивает при завершении родителя.
