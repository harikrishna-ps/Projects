import numpy as np
import cv2
import pickle
import RPi.GPIO as GPIO
from time import sleep
import time, math
from RPLCD.i2c import CharLCD


dist_meas = 0.00
km_per_hour = 0
rpm = 0
elapse = 0
sensor = 12
Relay = 5
Buzzer = 6
pulse = 0
start_timer = time.time()
#############################################
 
frameWidth= 640         # CAMERA RESOLUTION
frameHeight = 480
brightness = 180
threshold = 0.75         # PROBABLITY THRESHOLD
font = cv2.FONT_HERSHEY_SIMPLEX
##############################################
 
# SETUP THE VIDEO CAMERA
lcd = CharLCD('PCF8574', 0x27)
cap = cv2.VideoCapture(0)
cap.set(3, frameWidth)
cap.set(4, frameHeight)
cap.set(10, brightness)
# IMPORT THE TRANNIED MODEL
pickle_in=open("model_trained.p","rb")  ## rb = READ BYTE
model=pickle.load(pickle_in)
 
def grayscale(img):
    img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    return img
def equalize(img):
    img =cv2.equalizeHist(img)
    return img
def preprocessing(img):
    img = grayscale(img)
    img = equalize(img)
    img = img/255
    return img
def getCalssName(classNo):
    if   classNo == 0: return 'Speed Limit 20km/h'
    elif classNo == 1: return 'Speed Limit 30km/h'
    elif classNo == 2: return 'Speed Limit 50km/h'
    elif classNo == 3: return 'Speed Limit 60km/h'
    elif classNo == 4: return 'Speed Limit 70km/h'
    elif classNo == 5: return 'Speed Limit 80km/h'
    elif classNo == 6: return 'End of Speed Limit 80 km/h'
    elif classNo == 7: return 'Speed Limit 100km/h'
    elif classNo == 8: return 'Speed Limit 120km/h'
    elif classNo == 9: return 'No passing'
    elif classNo == 10: return 'No passing for vechiles over 3.5 metric tons'
    elif classNo == 11: return 'Right-of-way at the next intersection'
    elif classNo == 12: return 'Priority road'
    elif classNo == 13: return 'Yield'
    elif classNo == 14: return 'Stop'
    elif classNo == 15: return 'No vechiles'
    elif classNo == 16: return 'Vechiles over 3.5 metric tons prohibited'
    elif classNo == 17: return 'No entry'
    elif classNo == 18: return 'General caution'
    elif classNo == 19: return 'Dangerous curve to the left'
    elif classNo == 20: return 'Dangerous curve to the right'
    elif classNo == 21: return 'Double curve'
    elif classNo == 22: return 'Bumpy road'
    elif classNo == 23: return 'Slippery road'
    elif classNo == 24: return 'Road narrows on the right'
    elif classNo == 25: return 'Road work'
    elif classNo == 26: return 'Traffic signals'
    elif classNo == 27: return 'Pedestrians'
    elif classNo == 28: return 'Children crossing'
    elif classNo == 29: return 'Bicycles crossing'
    elif classNo == 30: return 'Beware of ice/snow'
    elif classNo == 31: return 'Wild animals crossing'
    elif classNo == 32: return 'End of all speed and passing limits'
    elif classNo == 33: return 'Turn right ahead'
    elif classNo == 34: return 'Turn left ahead'
    elif classNo == 35: return 'Ahead only'
    elif classNo == 36: return 'Go straight or right'
    elif classNo == 37: return 'Go straight or left'
    elif classNo == 38: return 'Keep right'
    elif classNo == 39: return 'Keep left'
    elif classNo == 40: return 'Roundabout mandatory'
    elif classNo == 41: return 'End of no passing'
    elif classNo == 42: return 'End of no passing by vechiles over 3.5 metric tons'


def init_GPIO():                    # initialize GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(sensor,GPIO.IN,GPIO.PUD_UP)
    GPIO.setup(Relay,GPIO.OUT)
    GPIO.setup(Buzzer,GPIO.OUT)


def calculate_elapse(channel):              # callback function
    global pulse, start_timer, elapse
    pulse+=1                                # increase pulse by 1 whenever interrupt occurred
    elapse = time.time() - start_timer      # elapse for every 1 complete rotation made!
    start_timer = time.time()               # let current time equals to start_timer

def calculate_speed(r_cm):
    global pulse,elapse,rpm,dist_km,dist_meas,km_per_sec,km_per_hour
    if elapse !=0:                          # to avoid DivisionByZero error
        rpm = 1/elapse * 60
        circ_cm = (2*math.pi)*r_cm          # calculate wheel circumference in CM
        dist_km = circ_cm/100000            # convert cm to km
        km_per_sec = dist_km / elapse       # calculate KM/sec
        km_per_hour = km_per_sec * 3600     # calculate KM/h
        dist_meas = (dist_km*pulse)*1000    # measure distance traverse in meter
        return km_per_hour

def init_interrupt():
    GPIO.add_event_detect(sensor, GPIO.FALLING, callback = calculate_elapse, bouncetime = 20)

if __name__ == '__main__':
    init_GPIO()
    init_interrupt()
    
def getSignSpeed(classNo):
    if   classNo == 0: return 20
    elif classNo == 1: return 30
    elif classNo == 2: return 50
    elif classNo == 3: return 60
    elif classNo == 4: return 70
    elif classNo == 5: return 80
    elif classNo == 6: return 100
    elif classNo == 7: return 100
    elif classNo == 8: return 120
    elif classNo == 9: return 40
    elif classNo == 10: return 100
    elif classNo == 11: return 100
    elif classNo == 12: return 100
    elif classNo == 13: return 100
    elif classNo == 14: return 20
    elif classNo == 15: return 100
    elif classNo == 16: return 100
    elif classNo == 17: return 20
    elif classNo == 18: return 100
    elif classNo == 19: return 100
    elif classNo == 20: return 100
    elif classNo == 21: return 100
    elif classNo == 22: return 80
    elif classNo == 23: return 80
    elif classNo == 24: return 58
    elif classNo == 25: return 80
    elif classNo == 26: return 80
    elif classNo == 27: return 80
    elif classNo == 28: return 30
    elif classNo == 29: return 80
    elif classNo == 30: return 80
    elif classNo == 31: return 80
    elif classNo == 32: return 100
    elif classNo == 33: return 80
    elif classNo == 34: return 80
    elif classNo == 35: return 80
    elif classNo == 36: return 80
    elif classNo == 37: return 80
    elif classNo == 38: return 80
    elif classNo == 39: return 80
    elif classNo == 40: return 80
    elif classNo == 41: return 80
    elif classNo == 42: return 80
    else :return 100



while True:
 
    # READ IMAGE
    success, imgOrignal = cap.read()
 
    # PROCESS IMAGE
    img = np.asarray(imgOrignal)
    img = cv2.resize(img, (32, 32))
    img = preprocessing(img)
    #cv2.imshow("Processed Image", img)
    img = img.reshape(1, 32, 32, 1)
    #cv2.putText(imgOrignal, "CLASS: " , (20, 35), font, 0.75, (0, 0, 255), 2, cv2.LINE_AA)
    #cv2.putText(imgOrignal, "PROBABILITY: ", (20, 75), font, 0.75, (0, 0, 255), 2, cv2.LINE_AA)
    # PREDICT IMAGE
    predictions = model.predict(img)
    classIndex = model.predict_classes(img)
    probabilityValue =np.amax(predictions)
    if probabilityValue > threshold:
        #print(getCalssName(classIndex))
        cv2.putText(imgOrignal,str(classIndex)+" "+str(getCalssName(classIndex)), (120, 35), font, 0.75, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(imgOrignal, str(round(probabilityValue*100,2) )+"%", (180, 75), font, 0.75, (0, 0, 255), 2, cv2.LINE_AA)
    #cv2.imshow("Result", imgOrignal)
    calculate_speed(5)
    lcd.clear()
    lcd.write_string(str(getCalssName(classIndex)))
    lcd.write_string('\n\rVehicleSpeed:{0:.0f}KM_per_hour________---------'.format(km_per_hour))
    speed = getSignSpeed(classIndex)
    if km_per_hour > speed:
        GPIO.output(Buzzer, GPIO.HIGH)
        time.sleep(.1)
        GPIO.output(Buzzer, GPIO.LOW)
        time.sleep(.03)
        GPIO.output(Buzzer, GPIO.HIGH)
        time.sleep(.1)
        GPIO.output(Buzzer, GPIO.LOW)
        GPIO.output(Relay,GPIO.LOW)
    else:
        GPIO.output(Buzzer,GPIO.LOW)
        GPIO.output(Relay,GPIO.HIGH)
    
 
 
    if cv2.waitKey(1) and 0xFF == ord('q'):
        break

