import serial
import struct
import threading
import time
import tkinter as tk

# Configure the serial port
ser = serial.Serial('/dev/tty.usbmodem1101', 115200)  # Update port and baud rate
if ser.is_open:
    print("Port is open!")
else:
    print("Failed to open port.")

# Global variable for shared value
value_lock = threading.Lock()  # Lock for thread-safe access to the value
value = 0
delay = 0.1 # 100ms 

# Function to send data
def send_data():
    global value
    while True:
        with value_lock:
            data = int(value)
        message = "set "+str(int(data)) + '\r' 
        ser.write(message.encode('utf-8'))
        print(f"Sent: set {data}")
        time.sleep(delay)  # Delay to avoid spamming the serial port
            


# Function to receive data
def receive_data():
    while True:
        if ser.in_waiting > 0:  # Check if there is data available
            data = ser.readline()  # Read one line of data
            print(data.decode('utf-8').strip())  # Print the received data, decoding it to string and stripping any extra whitespace

# Function to update the value from the slider
def update_value(new_value):
    global value
    new_value = float(new_value)  # Ensure the value is a float
    
    # Check if the value is 10 or greater
    if new_value >= 10:
        new_value *= 10  # Multiply by 10 if the value is 10 or greater

    with value_lock:
        value = new_value

# Create Tkinter GUI
def create_gui():
    root = tk.Tk()
    root.title("Serial Value Slider")

    # Make the window always on top
    root.attributes("-topmost", 1)

    # Raise the window to the top of the stack
    root.lift()

    tk.Label(root, text="Select Value").pack(pady=10)

    # Create a larger slider
    slider = tk.Scale(
        root,
        from_=0,
        to=120,
        resolution=1,
        orient=tk.HORIZONTAL,
        command=update_value,
        length=400,  # Adjust the length of the slider
        sliderrelief=tk.GROOVE,  # Style the slider
        width=20  # Adjust the thickness of the slider handle
    )
    slider.set(value)  # Set default value
    slider.pack(pady=10)

    root.mainloop()

# Create and start threads
send_thread = threading.Thread(target=send_data, daemon=True)
receive_thread = threading.Thread(target=receive_data, daemon=True)

send_thread.start()
receive_thread.start()

# Start the Tkinter GUI in the main thread
create_gui()

# Keep the program running
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nExiting...")