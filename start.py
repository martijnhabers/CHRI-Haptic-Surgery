import subprocess
import time
import signal
import sys

# Paths to your two Python files
robot = "robot3.py"   # Change to actual path if needed
haptic = "haptic.py"  # Change to actual path if needed

# Start both scripts in separate terminal windows
process_haptic = subprocess.Popen(["python", haptic], creationflags=subprocess.CREATE_NEW_CONSOLE)
time.sleep(1)  # Give haptic a moment to start before robot
process_robot = subprocess.Popen(["python", robot], creationflags=subprocess.CREATE_NEW_CONSOLE)

# Function to handle exit (terminate both processes)
def cleanup_and_exit(signum, frame):
    print("\nClosing both processes...")
    process_haptic.terminate()  # Kill haptic
    process_robot.terminate()   # Kill robot
    process_haptic.wait()
    process_robot.wait()
    print("Processes terminated. Exiting.")
    sys.exit(0)

# Catch exit signals (Ctrl+C or script termination)
signal.signal(signal.SIGINT, cleanup_and_exit)
signal.signal(signal.SIGTERM, cleanup_and_exit)

# Keep script running to detect exit signals
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    cleanup_and_exit(None, None)
