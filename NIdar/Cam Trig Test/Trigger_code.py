# This is trigger_code.py
# This not an optimised code is manageable for the testing purpose 
# Dint use logger beause i dint have enough time to read about it fully , will do it later
# There is no frame reszing or frame capping done in this code 
# I also dint use os beause file is already created and path is mentioned 
# Dint use time and timer but used a while loop for easier implementation and not having total understanding of the modules 

import cv2

# parameters declared to start the cam feed and start recording

#--------------------------------------------------------------------------#

video_writer = None # obj to write vid frames 
recording = False # initial state of recording 
filename = "name of the text file used in switch_code.py" # filename to enter 

#--------------------------------------------------------------------------#

# Starts the camera and checks the status from the text file

#--------------------------------------------------------------------------#
cap = cv2.VideoCapture(0)  
if not cap.isOpened():
    print("Error: Camera not accessible")
    exit()
while True:
    read,frame = cap.read()
    if not read:
        print("Failed to grab frame")
        break
    try:
        with open(filename, "r") as file:
            status = file.read().strip()
    except FileNotFoundError:
        status = "0"  # Default to not recording if file is empty (edge case)
    if status == "1" and not recording:
        # Start recording
        x = cv2.VideoWriter_fourcc(*'mp4v')
        video_writer = cv2.VideoWriter("output.mp4", x, 20.0, (frame.shape[1], frame.shape[0]))
        recording = True
        print("recording")
    elif status == "0" and recording:
        # Stop recording
        video_writer.release()
        video_writer = None
        recording = False
        print("recording stopped")
    if recording:
        video_writer.write(frame)

#--------------------------------------------------------------------------#

# Used to display the cam feed 

#--------------------------------------------------------------------------#

    cv2.imshow("Camera", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

#--------------------------------------------------------------------------#

# Executes at the end of the script to release resources 

#--------------------------------------------------------------------------#

if video_writer:
    video_writer.release()
cap.release()
cv2.destroyAllWindows()

#--------------------------------------------------------------------------#
