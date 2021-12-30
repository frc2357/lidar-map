import cv2
import sys
import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, atan2, pi, degrees

def get_correspondence_indices(P, Q):
    """For each point in P find closest one in Q."""
    p_size = P.shape[1]
    q_size = Q.shape[1]
    correspondences = []
    for i in range(p_size):
        p_point = P[:, i]
        min_dist = sys.maxsize
        chosen_idx = -1
        for j in range(q_size):
            q_point = Q[:, j]
            dist = np.linalg.norm(q_point - p_point)
            if dist < min_dist:
                min_dist = dist
                chosen_idx = j
        correspondences.append((i, chosen_idx))
    return correspondences
    
def center_data(data, exclude_indices=[]):
    reduced_data = np.delete(data, exclude_indices, axis=1)
    center = np.array([reduced_data.mean(axis=1)]).T
    return center, data - center

def compute_cross_covariance(P, Q, correspondences, kernel=lambda diff: 1.0):
    cov = np.zeros((2, 2))
    exclude_indices = []
    for i, j in correspondences:
        p_point = P[:, [i]]
        q_point = Q[:, [j]]
        weight = kernel(p_point - q_point)
        if weight < 0.01: exclude_indices.append(i)
        cov += weight * q_point.dot(p_point.T)
    return cov, exclude_indices

def icp_svd(P, Q, iterations=10, kernel=lambda diff: 1.0):
    """Perform ICP using SVD."""
    P_values = [P.copy()]
    P_copy = P.copy()
    corresp_values = []
    exclude_indices = []
    total_angle = 0
    total_x = 0
    total_y = 0

    for i in range(iterations):
        correspondences = get_correspondence_indices(P_copy, Q)
        corresp_values.append(correspondences)

        R = calculate_svd_rotation_correction(P_copy, Q, correspondences)
        t = calculate_correspondence_translate_correction(P_copy, Q, correspondences)

        angle = degrees(atan2(R[1, 0], R[0, 0]))
        x = t[1][0]
        y = t[0][0]

        print(f"R: {R}")
        print(f"angle: {angle}")
        print(f"T: {t}")
        print(f"x/y: {x}, {y}")
        total_angle += angle
        total_x = x
        total_y = y

        P_copy = R.dot(P_copy) + t
        P_values.append(P_copy)
    corresp_values.append(corresp_values[-1])

    print(f"Total adjustment: {total_angle} degress, x: {total_x}, y: {total_y}")

    return P_values, corresp_values

def calculate_svd_rotation_correction(P, Q, correspondences, kernel=lambda diff: 1.0):
    cov, exclude_indices = compute_cross_covariance(P, Q, correspondences, kernel)
    U, S, V_T = np.linalg.svd(cov)
    R = U.dot(V_T)  
    return R

def calculate_correspondence_translate_correction(P, Q, correspondences):
    x_offset_sum = 0.0
    y_offset_sum = 0.0

    for index in range(0, len(correspondences)):
        p_index = correspondences[index][0]
        q_index = correspondences[index][1]
        
        p_y = P[0][p_index]
        p_x = P[1][p_index]
        q_y = Q[0][q_index]
        q_x = Q[1][q_index]
        
        x_offset = q_x - p_x
        y_offset = q_y - p_y
        
        x_offset_sum += x_offset
        y_offset_sum += y_offset

    x_mean_offset = (x_offset_sum / len(correspondences))
    y_mean_offset = (y_offset_sum / len(correspondences))
    return [[y_mean_offset], [x_mean_offset]]
