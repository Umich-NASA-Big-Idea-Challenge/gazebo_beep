#!/bin/bash

# Simple script to send duty cycle commands to motors

# Function to send command to a motor
send_to_motor() {
    local topic=$1
    local value=$2
    
    # Validate duty cycle range (-1.0 to 1.0)
    if (( $(echo "$value < -1.0" | bc -l) )) || (( $(echo "$value > 1.0" | bc -l) )); then
        echo "Error: Duty cycle must be between -1.0 and 1.0"
        return 1
    fi
    
    # Send command via ignition transport
    ign topic -t "/$topic" -m ignition.msgs.Double -p "data: $value"
    echo "Sent $value to /$topic"
}

# Help message
if [[ "$1" == "-h" || "$1" == "--help" || $# -eq 0 ]]; then
    echo "Usage: $0 [OPTION]"
    echo "Options:"
    echo "  left VALUE    Set left motor duty cycle (-1.0 to 1.0)"
    echo "  right VALUE   Set right motor duty cycle (-1.0 to 1.0)"
    echo "  both VALUE    Set both motors to same duty cycle"
    echo "  stop          Stop all motors"
    exit 0
fi

# Process commands
case "$1" in
    left)
        send_to_motor "left_duty" "$2"
        ;;
    right)
        send_to_motor "right_duty" "$2"
        ;;
    both)
        send_to_motor "left_duty" "$2"
        send_to_motor "right_duty" "$2"
        ;;
    stop)
        send_to_motor "left_duty" "0.0"
        send_to_motor "right_duty" "0.0"
        echo "Motors stopped"
        ;;
    *)
        echo "Unknown command: $1"
        echo "Use -h or --help for usage information"
        exit 1
        ;;
esac