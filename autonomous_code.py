import cv2
import RPi.GPIO as GPIO
import time
import numpy as np
import face_recognition
from PIL import Image

GPIO.setwarnings(False)
servo_pin = 16
dc2_motor = 12
dc_motor = 13
dc_motor3 = 23
dc_motor4 = 24
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)
GPIO.setup(dc_motor, GPIO.OUT)
GPIO.setup(dc2_motor, GPIO.OUT)
# for servo 2
GPIO.setup(dc_motor3, GPIO.OUT)
GPIO.setup(dc_motor4, GPIO.OUT)
# for the second dc motor
pwm_dc2_motor = GPIO.PWM(dc2_motor, 1000)
pwm_dc_motor = GPIO.PWM(dc_motor, 1000)
pwm_dc_motor3 = GPIO.PWM(dc_motor3, 1000)
pwm_dc_motor4 = GPIO.PWM(dc_motor4, 1000)

pwm_servo = GPIO.PWM(servo_pin, 50)
pwm_servo.start(0)

cap = cv2.VideoCapture(0)

face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Load a specific image file
face_image = face_recognition.load_image_file("/home/pi/Desktop/my_image.jpg")

# Known face encodings and names (replace these with your own data)
known_face_encodings = [face_recognition.face_encodings(face_image)[0]]
known_face_names = ["Thong"]

count = 0
def obj_data(img):
    global pwm_servo, pwm_dc2_motor, known_face_encodings, known_face_names

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Use face_recognition.face_locations() without specifying the model
    face_locations = face_recognition.face_locations(img)
    face_encodings = face_recognition.face_encodings(img, face_locations)

    if not face_encodings:
        print("No face detected")
        pwm_servo.start(0)
        pwm_dc2_motor.start(0)
        pwm_dc_motor.start(0)
        pwm_dc_motor3.start(0)
        pwm_dc_motor4.start(0)
    else:
        for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
            # Check if the face matches any known faces
            matches = face_recognition.compare_faces(known_face_encodings, face_encoding)

            name = "Unknown"

            # If a match is found, use the name of the matched known face
            if True in matches:
                first_match_index = matches.index(True)
                name = known_face_names[first_match_index]

            cv2.rectangle(img, (left, top), (right, bottom), (0, 0, 255), 2)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(img, name, (left + 6, bottom - 6), font, 0.5, (255, 255, 255), 1)

            cx = (left + right) // 2
            distance = abs(cx - 320)
            print("Face detected at (x, y):", (left + right) // 2, (top + bottom) // 2)
            print("Distance:", distance)
            a = cx // 62
            b = a + 90
            print("Servo angle: ", b)
            pwm_servo.start(a)

            motor_speed = int(200 - (distance / 3))
            motor_speed = max(0, min(motor_speed, 100))

            pwm_dc2_motor.ChangeDutyCycle(motor_speed)
            pwm_dc_motor.ChangeDutyCycle(motor_speed)
            pwm_dc_motor3.ChangeDutyCycle(motor_speed)
            pwm_dc_motor4.ChangeDutyCycle(motor_speed)
            print("ALL MOTORS ARE SPINNING")

            if a > 5:
                pwm_dc2_motor.ChangeDutyCycle(0)
                pwm_dc_motor4.ChangeDutyCycle(0)
                print("turn right")
            elif a < 2:
                pwm_dc_motor.ChangeDutyCycle(0)
                pwm_dc_motor3.ChangeDutyCycle(0)
                print("turn left")
            print("motor speed: ", motor_speed)

while True:
    ret, frame = cap.read()
    count += 1
    if count % 10 != 0:
        continue
    frame = cv2.resize(frame, (640, 480))
    frame = cv2.flip(frame, -1)
    obj_data(frame)

    cv2.imshow("FRAME", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
pwm_servo.stop()
GPIO.cleanup()
pwm_dc2_motor.stop()
pwm_dc_motor.stop()
pwm_dc_motor3.stop()
pwm_dc_motor4.stop()
cv2.destroyAllWindows()
