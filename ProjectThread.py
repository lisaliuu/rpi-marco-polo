from threading import Thread, Lock, Condition
import time

# Global variables for shared data and synchronization
mutex = Lock()  # Mutex to ensure thread-safe access to shared variables
wait_sound = Condition(mutex)  # Condition variable for sound detection
wait_OV = Condition(mutex)  # Condition variable for object avoidance

sound_detected = False  # Shared variable to signal sound detection
object_avoidance_running = True  # Shared variable to indicate whether object avoidance is running

def echolocation_thread():
    print('Echolocation thread is starting...')
    global sound_detected, object_avoidance_running
    while True:
        input("Press Enter to simulate sound detection...")
        with mutex:
            sound_detected = True
            while object_avoidance_running:
                wait_OV.wait()
            print("Running Motors: Echolocation")
            time.sleep(10)
            sound_detected = False
            wait_sound.notify_all()

def object_avoidance_thread():
    print('Object Avoidance thread is starting...')
    global sound_detected, object_avoidance_running
    while True:
        with mutex:
            object_avoidance_running = True
            while sound_detected:
                wait_sound.wait()
            print("Running Motors: Object Avoidance")
            object_avoidance_running = False
            wait_OV.notify()

if __name__ == '__main__':
    print('Program is starting ...')
    echolocation_thread = Thread(target=echolocation_thread)
    echolocation_thread.daemon = True
    echolocation_thread.start()

    object_avoidance_thread = Thread(target=object_avoidance_thread)
    object_avoidance_thread.daemon = True
    object_avoidance_thread.start()

    # Main program continues here if needed
    while True:
        time.sleep(1)  # Example sleep to keep the main program running