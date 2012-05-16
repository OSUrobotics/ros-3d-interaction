import numpy as np

SAME_OBJ_THRESH = 0.03

def dist(p1, p2):
    p1 = np.array(p1)
    return np.sum(np.sqrt((p1-p2)**2))

def sameObject(p1, p2):
    return dist(p1,p2) < SAME_OBJ_THRESH

def closestPoint(pts, pt):
    # which point in pts is the closest to pt? 
    distances = [dist(pp,pt) for pp in pts]
    return pts[np.argmin(distances)]