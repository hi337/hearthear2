import serial
import time
from pynput import keyboard

# Configure the serial port
arduino_port = 'COM6'  # Change this to your Arduino's serial port
baud_rate = 9600

# Establish serial connection
try:
    ser = serial.Serial(arduino_port, baud_rate)
    time.sleep(2)  # Give some time for the connection to establish
    print(f"Connected to Arduino on port {arduino_port}")
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit()

def on_press(key):
    try:
        if key == keyboard.Key.space:
            ser.write(b' ')  # Send space character to Arduino
    except AttributeError:
        pass

def on_release(key):
    if key == keyboard.Key.esc:
        # Stop listener
        return False

# Start the keyboard listener
try:
    with keyboard.Listener(
            on_press=on_press,
            on_release=on_release) as listener:
        listener.join()
except Exception as e:
    print(f"Error with keyboard listener: {e}")

# Close the serial connection
ser.close()
print("Serial connection closed.")
