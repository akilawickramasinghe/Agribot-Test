from lib2to3.pytree import Node
from pickle import TRUE
from statistics import mode
import sys, time
from matplotlib.style import available
sys.path.insert(1, 'modules')
from modules import control_Rover,vision_Rover
import keyboard
import cv2

grnd_Speed = 1600
mode = 'test'
control = 'PID'

MAX_ALT =  0                                  
STATE = "takeoff"  # takeoff land track search -  To begin first set STATE to takeoff, for the rover --> Arm

def setup():
    global cap
#Connecting the Rover
    print("Connecting to Agribot")
    if mode == "drive":
        print("MODE = Drive")
        control_Rover.connect_rover('/dev/ttyTHS1') #Refer Control Module
        cap = cv2.VideoCapture(0)
    else:
        print("MODE = test")  # Launch Simulator if rover is not connected
        control_Rover.connect_rover('tcp:127.0.0.1:5763')
        #cap = cv2.VideoCapture("slownewvideo.mp4")
        cap = cv2.VideoCapture(0)

setup()
control_Rover.configure_PID(control)



def readFrames():
    global ret,frame
    ret,frame = cap.read()
    #resizeframe = cv2.resize(frame,(640,480),interpolation = cv2.INTER_AREA)
    #cv2.imshow('frame', frame) 
    if cv2.waitKey(1) & 0xFF == ord('q'):
        return "holdMission"
    return frame


def trackPath():
    print("State is TRACKING -> " + STATE) # Setting the STATE to path tracking

    while True:
        if keyboard.is_pressed('q'):  # if key 'q' is pressed 
            print("Closing due to manual interruption")
            holdMission() # Closes the loop and program
        
        #debug
        tracks = TRUE

        if tracks == TRUE:           
            cropRawAngle = vision_Rover.getAngle(readFrames()) # Function to take angle of the row and the heading of the rover
            #MA_X.append(cropRawAngle)
            lenth = 1
            if lenth > 0:
                #while 1:
                #    print("ANGLE FOUNDED")
                #x_delta_MA = calculate_ma(MA_X) #Function to scale to millisecond values(Not Necessary)
                control_Rover.setPathdelta(cropRawAngle)
                steer_command = control_Rover.getMovementSteerAngle() #Return PID calculated values
                print("Steer Value = ",steer_command)
            control_Rover.control_rover() #In the back od the code, this funtions write mavlink command with PID calculated value
            print("Rover Controlled")
        else: # Keep search
            return "search"

def search():
    global tracks
    print("State is SEARCH -> " + STATE)
    start = time.time()
    control_Rover.stop_rover()
    while time.time() - start < 20:
        if keyboard.is_pressed('q'):  # if key 'q' is pressed 
            print("Closing due to manual interruption")
            holdMission() # Closes the loop and program
        currentHeading = control_Rover.rowSearch()
        while control_Rover.rowSearch() < currentHeading + 60:
            if vision_Rover.detect(readFrames()): #and control_Rover.rowSearch(): #Row Detector: Should return TRUE
                print("Row is Founded")
                tracks = TRUE #Checking if there are rows to follow
                return "track"
    return "hold"
    #Debug
    #return "track"

def takeoff():
    control_Rover.print_rover_report()
    print("State = TAKEOFF -> " + STATE)
    control_Rover.arm_and_takeoff() #start control when Agribot is ready
    #return "search"
    #debug
    #return"track"

def hold():
    print("State = Hold -> " + STATE)
    control_Rover.stop_rover()

def holdMission():
    print("State = Hold Mission")
    control_Rover.disarm()
    #while TRUE:
    #    if keyboard.is_pressed('q'):
    #        print("Closing due to manual interruption")
            #cap.relese()
            #cv2.destroyAllWindows()
    #        sys.exit(0)
    STATE = modeSelector()

def autoMode():
    while True:
        print("Auto Mode")

def manualMode():
    while TRUE:
        print("Manual Mode")    

def modeSelector():
    takeoff()
    mode = input("Enter Mode Number: \n 1: Guided Mode\n 2:Auto Mode\n 3:Manual Mode\n")
    #while True:
    print(mode)
    mode = int(mode)
    if mode == 1:
        return "track"
    elif mode == 2:
        return "auto"
    elif mode == 3:
        return "manual"
    else:
        print("Enter a Valid Mode:")

print("Initialization is completed")
STATE = modeSelector()

while True:
    # main program loop
    """" True or False values depend whether or not
        a PID controller or a P controller will be used  """
            
    if STATE == "track":
        control_Rover.set_system_state("track")
        STATE = trackPath()
    elif STATE == "search":
        control_Rover.set_system_state("search")
        STATE = search()
    elif STATE == "takeoff":
        control_Rover.set_system_state("takeoff")
        STATE = takeoff()
    elif STATE == "hold":
        control_Rover.set_system_state("hold")
        STATE = hold()
    elif STATE == "holdMission":
        control_Rover.set_system_state("holdMission")
        STATE = holdMission()
    elif STATE == "auto":
        control_Rover.set_system_state("auto")
        STATE = autoMode()
    elif STATE == "manual":
        control_Rover.set_system_state("manual")
        STATE = manualMode()
    elif STATE == "mode":
        control_Rover.set_system_state("mode")
        STATE = modeSelector()


