
from innopy.api import DeviceInterface
import keyboard
import threading
import time
import sys

# Flag to control the recording state
recording = False
exit_event = threading.Event()

def main():
    global recording
    config_files_path = '../lidar_configuration_files'
    di = DeviceInterface(config_file_name=config_files_path+'/om_config.json', is_connect=False)
    
    # Set up the key press listener
    keyboard.on_press_key('s', lambda event: toggle_recording(event, di, exit_event))

def toggle_recording(event, di, exit_event):
    global recording
    if not recording:
        print("Recording started.")
        threading.Thread(target=di.start_recording, args=("pcl_recording",)).start()
    else:
        di.stop_recording()
        exit_event.set()
        sys.exit()  # Exit the program after stopping recording

    recording = not recording  # Toggle the recording flag

# Create and start the main program thread
program_thread = threading.Thread(target=main, daemon=True)
program_thread.start()

print("Press 's' to start or stop recording.")
exit_event.wait() 
