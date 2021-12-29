import cv2
import sys
import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, atan2, pi

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

    for i in range(iterations):
        correspondences = get_correspondence_indices(P_copy, Q)

        x_offset_sum = 0.0
        y_offset_sum = 0.0

        for index in range(0, len(correspondences)):
            p_index = correspondences[index][0]
            q_index = correspondences[index][1]
            
            p_x = P_copy[0][p_index]
            p_y = P_copy[1][p_index]
            q_x = Q[0][q_index]
            q_y = Q[1][q_index]
            
            x_offset = q_x - p_x
            y_offset = q_y - p_y
            
            x_offset_sum += x_offset
            y_offset_sum += y_offset

        x_mean_offset = x_offset_sum / len(correspondences)
        y_mean_offset = y_offset_sum / len(correspondences)
        print(f"sums: {x_offset_sum},{y_offset_sum}")
        print(f"mean offset: {x_mean_offset},{y_mean_offset}")

        corresp_values.append(correspondences)
        cov, exclude_indices = compute_cross_covariance(P_copy, Q, correspondences, kernel)
        U, S, V_T = np.linalg.svd(cov)
        R = U.dot(V_T)  
        t = [[x_mean_offset], [y_mean_offset]]
        P_copy = R.dot(P_copy) + t
        P_values.append(P_copy)
    corresp_values.append(corresp_values[-1])
    return P_values, corresp_values
