import cv2
import numpy as np

def projectPointToImage(pt, K, D):
	return cv2.projectPoints(np.float64(pt), np.zeros(3), np.zeros(3), K, D)[0][0]
