import numpy as np
import cv2

class KalmanCentroidPredictor:
    def __init__(self):
        self.kalman = cv2.KalmanFilter(4, 2)
        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0],
                                                   [0, 1, 0, 0]], np.float32)
        self.kalman.transitionMatrix = np.array([[1, 0, 1, 0],
                                                 [0, 1, 0, 1],
                                                 [0, 0, 1, 0],
                                                 [0, 0, 0, 1]], np.float32)
        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03

    def predict(self):
        return self.kalman.predict()

    def correct(self, centroid):
        measured = np.array([[np.float32(centroid[0])],
                             [np.float32(centroid[1])]])
        return self.kalman.correct(measured)