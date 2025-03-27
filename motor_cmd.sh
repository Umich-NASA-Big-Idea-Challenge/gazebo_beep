#!/bin/sh
# Simple script to send duty cycle commands to motors

# Function to send command to a motor
send_to_motor() {
    topic=$1
    value=$2
    
    # Simple validation without bc
    if [ "$value" = "stop" ]; then
        value="0.0"
    fi
    
    # Send command via ignition transport
    ign topic -t "/$topic" -m ignition.msgs.Double -p "data: $value"
    echo "Sent $value to /$topic"
}

# Process commands
if [ "$1" = "left" ]; then
    if [ "$2" = "stop" ] || [ -z "$2" ]; then
        send_to_motor "left_duty" "0.0"
    else
        send_to_motor "left_duty" "$2"
    fi
elif [ "$1" = "right" ]; then
    if [ "$2" = "stop" ] || [ -z "$2" ]; then
        send_to_motor "right_duty" "0.0"
    else
        send_to_motor "right_duty" "$2"
    fi
elif [ "$1" = "both" ]; then
    if [ "$2" = "stop" ] || [ -z "$2" ]; then
        send_to_motor "left_duty" "0.0"
        send_to_motor "right_duty" "0.0"
    else
        send_to_motor "left_duty" "$2"
        send_to_motor "right_duty" "$2"
    fi
elif [ "$1" = "stop" ]; then
    send_to_motor "left_duty" "0.0"
    send_to_motor "right_duty" "0.0"
    echo "Motors stopped"
else
    echo "Usage: $0 [OPTION]"
    echo "Options:"
    echo "  left VALUE    Set left motor duty cycle (-1.0 to 1.0)"
    echo "  right VALUE   Set right motor duty cycle (-1.0 to 1.0)"
    echo "  both VALUE    Set both motors to same duty cycle"
    echo "  stop          Stop all motors"
    echo ""
    echo "Examples:"
    echo "  $0 left 0.5     # Set left motor to 50% forward"
    echo "  $0 right -0.3   # Set right motor to 30% reverse"
    echo "  $0 both 0.7     # Set both motors to 70% forward"
    echo "  $0 left stop    # Stop left motor"
    echo "  $0 stop         # Stop all motors"
fi