import cv2
import serial
import time
import numpy as np

def NIL(x):
    pass

ser = serial.Serial('COM7', baudrate=115200, timeout=1)
# Check and change the COM port to match with your Arduino connection
def move_camera_servo(centroid_x, centroid_y, frame_width, frame_height):
    panAngle = int(centroid_x * 1000 / frame_width)  # Map centroid_x to 0-180 range
    tiltAngle = int(centroid_y * 1000 / frame_height)  # Map centroid_y to 0-180 range
    panAngle = min(max(panAngle, 0), 1000)  # Ensure panAngle is within 0-180 range
    tiltAngle = min(max(tiltAngle, 0), 1000)  # Ensure tiltAngle is within 0-180 range
    ser.write(f"{panAngle},{tiltAngle}\r".encode())  # Send servo angles to Arduino
    print(f"Sent servo angles to Arduino: Pan={panAngle}, Tilt={tiltAngle}")

time.sleep(0.5)
pos = 90
# print(type(pos))
cv2.namedWindow('Trackbars')
cv2.createTrackbar('Hue_Low', 'Trackbars', 0, 179, NIL)
cv2.createTrackbar('Hue_High', 'Trackbars', 90, 179, NIL)
cv2.createTrackbar('Sat_Low', 'Trackbars', 160,255, NIL)
cv2.createTrackbar('Sat_High', 'Trackbars', 255, 255, NIL)
cv2.createTrackbar('Val_Low', 'Trackbars', 35, 255, NIL)
cv2.createTrackbar('Val_High', 'Trackbars', 255, 255, NIL)

cam = cv2.VideoCapture(0, cv2.CAP_DSHOW)

while True:
    _, frame = cam.read()
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    Hue_Low = cv2.getTrackbarPos('Hue_Low', 'Trackbars')
    Hue_High = cv2.getTrackbarPos('Hue_High', 'Trackbars')
    Sat_Low = cv2.getTrackbarPos('Sat_Low', 'Trackbars')
    Sat_High = cv2.getTrackbarPos('Sat_High', 'Trackbars')
    Val_Low = cv2.getTrackbarPos('Val_Low', 'Trackbars')
    Val_High = cv2.getTrackbarPos('Val_High', 'Trackbars')

    fore_mask = cv2.inRange(frameHSV, (Hue_Low, Sat_Low, Val_Low), (Hue_High, Sat_High, Val_High))
    fore = cv2.bitwise_and(frame, frame, mask=fore_mask)

    contours, hierarchy = cv2.findContours(fore_mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        (x, y, w, h) = cv2.boundingRect(cnt)
        if area > 500:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 3)
            cv2.drawContours(frame, [cnt], 0, (0, 0, 255), 2)

    cv2.imshow("Fore", fore)
    cv2.imshow("Fore_mask", fore_mask)
    cv2.imshow("nomal", frame)
    ret, img = cam.read()
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 3)
    errorPan = (x + w/2) - 640/2
    print('errorPan', errorPan)
    # print(type(errorPan))
    if abs(errorPan) > 10:
            pos = pos - errorPan/30
            print(type(pos))
    if pos > 180:
            pos = 160
            print("Out of range")
    if pos < 5:
            pos = 6
            print("out of range")
    servoPos = str(pos) + '\r'
    ser.write(servoPos.encode('utf-8'))
    print('servoPos = ', servoPos)
    # print(type(pos))
    time.sleep(0.1)

    lowerBound = np.array([Hue_Low, Sat_Low, Val_Low])
    upperBound = np.array([Hue_High, Sat_High, Val_High])
    # Threshold the HSV frame to get only the purple object
    mask = cv2.inRange(frameHSV, lowerBound, upperBound)

    # Apply mask to the original frame
    result = cv2.bitwise_and(frame, frame, mask=mask)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # If contours are found, calculate centroid and adjust servos
    if contours:

        # Get largest contour
        largestContour = max(contours, key=cv2.contourArea)

        # Get bounding rectangle around the largest contour
        x, y, w, h = cv2.boundingRect(largestContour)

        # Draw rectangle around the detected object
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Calculate centroid of the largest contour
        centroid_x = x + w // 2
        centroid_y = y + h // 2

        # Move camera servo to track the purple object
        move_camera_servo(centroid_x, centroid_y, frame.shape[1], frame.shape[0])


    if cv2.waitKey(1) & 0xff == 27:
        break

ser.close()
cam.release()
cv2.destroyAllWindows()