import cv2 as cv
import cv2.aruco as aruco
import socket 
import numpy as np
import time
def rot2eul(R):
    beta = -np.arcsin(R[2,0])
    alpha = np.arctan2(R[2,1]/np.cos(beta),R[2,2]/np.cos(beta))
    gamma = np.arctan2(R[1,0]/np.cos(beta),R[0,0]/np.cos(beta))
    return np.array((alpha, beta, gamma))

def eul2rot(theta) :

    R = np.array([[np.cos(theta[1])*np.cos(theta[2]),       np.sin(theta[0])*np.sin(theta[1])*np.cos(theta[2]) - np.sin(theta[2])*np.cos(theta[0]),      np.sin(theta[1])*np.cos(theta[0])*np.cos(theta[2]) + np.sin(theta[0])*np.sin(theta[2])],
                  [np.sin(theta[2])*np.cos(theta[1]),       np.sin(theta[0])*np.sin(theta[1])*np.sin(theta[2]) + np.cos(theta[0])*np.cos(theta[2]),      np.sin(theta[1])*np.sin(theta[2])*np.cos(theta[0]) - np.sin(theta[0])*np.cos(theta[2])],
                  [-np.sin(theta[1]),                        np.sin(theta[0])*np.cos(theta[1]),                                                           np.cos(theta[0])*np.cos(theta[1])]])

    return R

def initKalmanFilter(nStates, nMeasurements, nInputs, dt):
    kalmanFilter = cv.KalmanFilter(nStates, nMeasurements, nInputs, cv.CV_64F)                # init Kalman Filter
    kalmanFilter.processNoiseCov = cv.setIdentity(kalmanFilter.processNoiseCov, 1)  # set process noiseq
    kalmanFilter.measurementNoiseCov = cv.setIdentity(kalmanFilter.measurementNoiseCov, 1)  # set measurement noise
    kalmanFilter.errorCovPost = cv.setIdentity(kalmanFilter.errorCovPost, 1)  # error covariance 
                   # DYNAMIC MODEL q
    #  [1 0 0 dt  0  0 dt2   0   0 0 0 0  0  0  0     0   0   0]
    #  [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0]
    #  [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0]
    #  [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0]
    #  [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0]
    #  [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0]
    #  [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0]
    #  [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0]
    #  [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0]
    #  [0 0 0  0  0  0   0   0   0 1 0 0 dt  0  0 dt2   0   0]
    #  [0 0 0  0  0  0   0   0   0 0 1 0  0 dt  0   0 dt2   0]
    #  [0 0 0  0  0  0   0   0   0 0 0 1  0  0 dt   0   0 dt2]
    #  [0 0 0  0  0  0   0   0   0 0 0 0  1  0  0  dt   0   0]
    #  [0 0 0  0  0  0   0   0   0 0 0 0  0  1  0   0  dt   0]
    #  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  1   0   0  dt]
    #  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   1   0   0]
    #  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   1   0]
    #  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   0   1]
    # position
    new_transition = kalmanFilter.transitionMatrix.copy()
    new_transition[0,3] = dt
    new_transition[1,4] = dt
    new_transition[2,5] = dt
    new_transition[3,6] = dt
    new_transition[4,7] = dt
    new_transition[5,8] = dt
    new_transition[0,6] = 0.5*pow(dt,2)
    new_transition[1,7] = 0.5*pow(dt,2)
    new_transition[2,8] = 0.5*pow(dt,2)
#  // orientation
    new_transition[9,12]= dt
    new_transition[10,13] = dt
    new_transition[11,14] = dt
    new_transition[12,15] = dt
    new_transition[13,16] = dt
    new_transition[14,17] = dt
    new_transition[9,15]= 0.5*pow(dt,2)
    new_transition[10,16] = 0.5*pow(dt,2)
    new_transition[11,17] = 0.5*pow(dt,2)
    kalmanFilter.transitionMatrix = new_transition
#       /* MEASUREMENT MODEL */
#  //  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
#  //  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
#  //  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
#  //  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
#  //  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
#  //  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]
    new_measurement = kalmanFilter.measurementMatrix.copy()
    new_measurement[0,0] = 1  # x
    new_measurement[1,1] = 1  # y
    new_measurement[2,2] = 1  # z
    new_measurement[3,9] = 1  # roll
    new_measurement[4,10] = 1 # pitch
    new_measurement[5,11] = 1 # yaw
    kalmanFilter.measurementMatrix = new_measurement
    return kalmanFilter

UDP_IP = "127.0.0.1"
UDP_PORT = 5005
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

cameraMatrix =np.array([[666.53097871,   0.,         328.23260392],
 [  0.,         667.4393499,  232.80417494],
 [  0.,           0.,           1.        ]]
)

distCoeffs = np.array([ 0.15410804, -1.17528765,  0.00225397, -0.0025559,   2.15236892])


nStates = 18            # the number of states
nMeasurements = 6       # the number of measured states
nInputs = 0             # the number of action control
dt = 0.0333          # time between measurements (1/FPS)
kalmanFilter = initKalmanFilter(nStates, nMeasurements, nInputs, dt)    # init function


cap = cv.VideoCapture(0)

# used to record the time when we processed last frame 
prev_frame_time = 0
  
# used to record the time at which we processed current frame 
new_frame_time = 0

arucoDict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
arucoParams = aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(arucoDict, arucoParams)
frames_estimated = 30
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Display the resulting frame
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    (corners, ids, rejected) = detector.detectMarkers(gray)
    new_frame_time = time.time() 
    fps = 1/(new_frame_time-prev_frame_time) 
    prev_frame_time = new_frame_time 
    fps = int(fps)
    opimage = frame.copy()
    offset_tvec = np.array([0.0,0.,0.])
    offset_rvec = np.array([0.,0.,0.])
    is_tracking = False
    if np.all(ids is not None) and len(ids) ==2 :  # If there are markers found by detector/Good measurement
        frames_estimated = 0
        rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[0], 0.025, cameraMatrix, distCoeffs)
        rvec2, tvec2, markerPoints2 = aruco.estimatePoseSingleMarkers(corners[1], 0.025, cameraMatrix, distCoeffs)
        (rvec - tvec).any()  # get rid of that nasty numpy value array error
        (rvec2 - tvec2).any()  # get rid of that nasty numpy value array error

        tvec = (tvec + tvec2)/2
        # time when we finish processing for this frame 
        #Kalman Filter 
        #Fill Measurements
        rotation_matrix = cv.Rodrigues(rvec)[0]
        measured_eulers = rot2eul(rotation_matrix)
        measurements = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        measurements[0] = tvec[0,0,0] # x
        measurements[1] = tvec[0,0,1] # y
        measurements[2] = tvec[0,0,2] # z
        measurements[3] = measured_eulers[0]      # roll
        measurements[4] = measured_eulers[1]      # pitch
        measurements[5] = measured_eulers[2]      # yaw
        
        #Update Kalman Filter
        translation_estimated = np.array([0.0,0.0,0.0])
        eulers_estimated =  np.array([0.0,0.0,0.0])
        # First predict, to update the internal statePre variable
        prediction = kalmanFilter.predict()
        # The "correct" phase that is going to use the predicted value and our measurement
        estimated = kalmanFilter.correct(measurements)
        # Estimated translation
        translation_estimated[0] = estimated[0]
        translation_estimated[1] = estimated[1]
        translation_estimated[2] = estimated[2]
        
        # Estimated euler angles
        eulers_estimated[0] = estimated[9]
        eulers_estimated[1] = estimated[10]
        eulers_estimated[2] = estimated[11]
        # Convert estimated quaternion to rotation matrix
        rotation_estimated = eul2rot(eulers_estimated)
        rotation_vect = cv.Rodrigues(rotation_estimated)[0]

        camera_pos = cv.composeRT(rotation_vect, translation_estimated,offset_rvec, offset_tvec )[0:2]
        aruco.drawDetectedMarkers(opimage, corners)  # Draw A square around the markers
        cv.drawFrameAxes(opimage, cameraMatrix, distCoeffs, camera_pos[0], camera_pos[1], 0.01)  # Draw Axis
        is_tracking = True
    elif (frames_estimated < 10):
        frames_estimated += 1
        translation_estimated = np.array([0.0,0.0,0.0])
        eulers_estimated =  np.array([0.0,0.0,0.0])
        prediction = kalmanFilter.predict()
        # Estimated translation
        translation_estimated[0] = prediction[0]
        translation_estimated[1] = prediction[1]
        translation_estimated[2] = prediction[2]
        # Estimated euler angles
        eulers_estimated[0] = prediction[9]
        eulers_estimated[1] = prediction[10]
        eulers_estimated[2] = prediction[11]
        # Convert estimated quaternion to rotation matrix
        rotation_estimated = eul2rot(eulers_estimated)
        rotation_vect = cv.Rodrigues(rotation_estimated)[0]

        camera_pos = cv.composeRT(rotation_vect, translation_estimated,offset_rvec, offset_tvec )[0:2]
        cv.drawFrameAxes(opimage, cameraMatrix, distCoeffs, camera_pos[0], camera_pos[1], 0.01)  # Draw Axis
        is_tracking = True
    
    if(is_tracking):
        degrees = 10.0
        z_offset = 215.9
        message = bytes(str(translation_estimated[2]*-100)+","+str(translation_estimated[0]*-100)+","+str(translation_estimated[1]*-100)+","+str(degrees)+","+str(z_offset)
                        , 'utf-8')
        print(message)
        server_socket.sendto(message, (UDP_IP,UDP_PORT))


    
    cv.imshow("out", opimage)
    #server_socket.sendto(MESSAGE, (UDP_IP,UDP_PORT))
    if cv.waitKey(1) == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()