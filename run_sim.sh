#!/bin/zsh

# Function to handle SIGINT
cleanup() {
    echo "Terminating background processes..."
    pkill -P $$
    wait
}

# Trap SIGINT (ctrl_c) and call cleanup
trap cleanup SIGINT

# Open a new terminal and run robot2.py
osascript -e 'tell application "Terminal" to do script "cd /Users/martijn/Documents/MSC-Robotics/RO47013-control-in-human-robot-interaction/assignment3/CHRI-Haptic-Surgery && python3 ./robot2.py"'

# Open another new terminal and run haptic.py
osascript -e 'tell application "Terminal" to do script "cd /Users/martijn/Documents/MSC-Robotics/RO47013-control-in-human-robot-interaction/assignment3/CHRI-Haptic-Surgery && python3 ./haptic.py"'

# Wait for all background processes to finish
wait