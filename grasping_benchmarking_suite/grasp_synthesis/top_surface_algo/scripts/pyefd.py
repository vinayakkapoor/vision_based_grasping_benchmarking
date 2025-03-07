#!/usr/bin/env python

import matplotlib
matplotlib.use('Agg')

import rospy
import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
from itertools import combinations
from cv_bridge import CvBridge

from top_surface_algo.srv import EFDGrasp, EFDGraspResponse
from sensor_msgs.msg import PointField, Image
from sensor_msgs.point_cloud2 import read_points, create_cloud

def elliptic_fourier_descriptors(contour, order=10):
    """Calculate elliptical Fourier descriptors for a contour.
    :param numpy.ndarray contour: A contour array of size ``[M x 2]``.
    :param int order: The order of Fourier coefficients to calculate.
    :param bool normalize: If the coefficients should be normalized;
        see references for details.
    :param bool return_transformation: If the normalization parametres should be returned.
        Default is ``False``.
    :return: A ``[order x 4]`` array of Fourier coefficients and optionally the
        transformation parametres ``scale``, ``psi_1`` (rotation) and ``theta_1`` (phase)
    :rtype: ::py:class:`numpy.ndarray` or (:py:class:`numpy.ndarray`, (float, float, float))
    """
    dxy = np.diff(contour, axis=0)
    dt = np.sqrt((dxy ** 2).sum(axis=1))
    t = np.concatenate([([0.0]), np.cumsum(dt)])
    T = t[-1]

    phi = (2 * np.pi * t) / T

    orders = np.arange(1, order + 1)
    consts = T / (2 * orders * orders * np.pi * np.pi)
    phi = phi * orders.reshape((order, -1))

    d_cos_phi = np.cos(phi[:, 1:]) - np.cos(phi[:, :-1])
    d_sin_phi = np.sin(phi[:, 1:]) - np.sin(phi[:, :-1])

    a = consts * np.sum((dxy[:, 0] / dt) * d_cos_phi, axis=1)
    b = consts * np.sum((dxy[:, 0] / dt) * d_sin_phi, axis=1)
    c = consts * np.sum((dxy[:, 1] / dt) * d_cos_phi, axis=1)
    d = consts * np.sum((dxy[:, 1] / dt) * d_sin_phi, axis=1)

    coeffs = np.concatenate(
        [
            a.reshape((order, 1)),
            b.reshape((order, 1)),
            c.reshape((order, 1)),
            d.reshape((order, 1)),
        ],
        axis=1,
    )

    return coeffs
    
def calculate_dc_coefficients(contour):
    """Calculate the :math:`A_0` and :math:`C_0` coefficients of the elliptic Fourier series.
    :param numpy.ndarray contour: A contour array of size ``[M x 2]``.
    :return: The :math:`A_0` and :math:`C_0` coefficients.
    :rtype: tuple
    """
    dxy = np.diff(contour, axis=0)
    dt = np.sqrt((dxy ** 2).sum(axis=1))
    t = np.concatenate([([0.0]), np.cumsum(dt)])
    T = t[-1]

    xi = np.cumsum(dxy[:, 0]) - (dxy[:, 0] / dt) * t[1:]
    A0 = (1 / T) * np.sum(((dxy[:, 0] / (2 * dt)) * np.diff(t ** 2)) + xi * dt)
    delta = np.cumsum(dxy[:, 1]) - (dxy[:, 1] / dt) * t[1:]
    C0 = (1 / T) * np.sum(((dxy[:, 1] / (2 * dt)) * np.diff(t ** 2)) + delta * dt)

    # A0 and CO relate to the first point of the contour array as origin.
    # Adding those values to the coefficients to make them relate to true origin.
    return contour[0, 0] + A0, contour[0, 1] + C0

def get_curve(coeffs, locus=(0.0, 0.0), n=300):
    """Populate xt and yt using the given Fourier coefficient array.
    :param numpy.ndarray coeffs: ``[N x 4]`` Fourier coefficient array.
    :param list, tuple or numpy.ndarray locus:
        The :math:`A_0` and :math:`C_0` elliptic locus in [#a]_ and [#b]_.
    :param int n: Number of points to use for plotting of Fourier series.
    :return: Tuple of populated xt and yt arrays.
    """
    t = np.linspace(0, 1.0, n)
    xt = np.ones((n,)) * locus[0]
    yt = np.ones((n,)) * locus[1]
    for n in range(coeffs.shape[0]):
        xt += (coeffs[n, 0] * np.cos(2 * (n + 1) * np.pi * t)) + (
            coeffs[n, 1] * np.sin(2 * (n + 1) * np.pi * t)
        )
        yt += (coeffs[n, 2] * np.cos(2 * (n + 1) * np.pi * t)) + (
            coeffs[n, 3] * np.sin(2 * (n + 1) * np.pi * t)
        )
    return xt, yt

def compute_tangents_normals(xt, yt):
    '''
    Compute first and second derivatives of x(t) and y(t)
    '''
    dxdt = np.gradient(xt)
    dydt = np.gradient(yt)
    d2xdt2 = np.gradient(dxdt)
    d2ydt2 = np.gradient(dydt)
    
    tangents = np.stack([dxdt, dydt], axis=-1)    
    normals = np.stack([d2xdt2, d2ydt2], axis=-1)
    
    return tangents, normals

def compute_curvature(normals):
    '''
    Compute the curvature scores
    '''
    return np.linalg.norm(normals, axis=-1)
    
def find_concave(tangent):
    '''
    Find concave points in the curve
    '''
    # Take the dot product of consecutive normal vectors
    dot_products = np.cross(tangent[:-1], tangent[1:])
    
    # Handle the last normal vector
    dot_products = np.concatenate([dot_products, [np.cross(tangent[-1], tangent[0])]])
    return dot_products

def plot_random_lines(xt, yt, tangents, random_indices=None, color='red', scale=0.01):
    '''
    Plot tangents/normals at random points
    Replace tangents with normals to plot normals
    '''    
    if random_indices is None:
        random_indices = np.arange(len(xt))
    for i in random_indices:
        start = [xt[i], yt[i]]
        end = start + scale*tangents[i]
        plt.arrow(start[0], start[1], end[0]-start[0], end[1]-start[1], 
                  head_width=0.02*scale, head_length=0.02*scale, fc=color, ec=color, linewidth=scale*0.01)
    
def find_local_max_min_indices(arr):
    '''
    Get indices of extremum points in the curve
    '''
    diff = np.diff(arr)
    maxima = np.where((diff[:-1] > 0) & (diff[1:] < 0))[0] + 1
    minima = np.where((diff[:-1] < 0) & (diff[1:] > 0))[0] + 1
    return np.array(maxima), np.array(minima)

def have_common_indices(arr1, arr2):
    '''
    Get common indices from two arrays containing indices
    '''
    set1 = set(arr1)
    set2 = set(arr2)
    return list(set1.intersection(set2))

def get_extremum_points(largest_contour, even_sample=True):
    '''
    Get prospective grasp points from the curve
    '''
    # Fit curve based on EFD
    coeffs = elliptic_fourier_descriptors(largest_contour, order=10)
    a0, c0 = calculate_dc_coefficients(largest_contour)
    xt, yt = get_curve(coeffs, locus=(a0,c0), n=300)
    
    # Compute tangents and normals
    tangents, normals = compute_tangents_normals(xt, yt)
    normals_norm = normals / np.linalg.norm(normals, axis=-1, keepdims=True)
 
    # Compute curvature and find extremum points 
    curvature = compute_curvature(normals)
    maxima, minima = find_local_max_min_indices(curvature)
    maxima_minima = np.concatenate([maxima, minima])
    
    # Filter only concave points
    # concavities = find_concave(tangents)
    # concave_indices = np.where(concavities >= 0)[0]    
    # concave_curvature = curvature[concave_indices]
    # concave_maxima_minima = have_common_indices(concave_indices, maxima_minima)
    
    if even_sample:
        maxima_minima = np.arange(0, len(xt), len(xt)//80) # For even sampling of indices

    # Get the grasp points combinations
    combinations_list = list(combinations(maxima_minima, 2))
    
    rotation_matrix = np.array([[0, -1], [1, 0]])
    outward_normals = np.dot(rotation_matrix, tangents.T).T
    outward_normals /= np.linalg.norm(outward_normals, axis=-1, keepdims=True)
    
    return xt, yt, outward_normals, maxima_minima, combinations_list, normals_norm

def get_grasp(largest_contour, visualize=False, split=False):
    '''
    Calculate the grasp points from input contours
    '''
    
    # Split the contour into subarrays if distance between points is large
    if split:
        distances = np.sqrt(np.sum(np.diff(largest_contour, axis=0)**2, axis=1))
        split_indices = np.where(distances > 0.03)[0]
    
        # If there are no split indices, return the original array
        if len(split_indices) == 0:
            subarrays = [largest_contour]
        else:
            # Add the first and last indices to the split indices
            split_indices = np.concatenate(([0], split_indices, [largest_contour.shape[0] - 1]))
            # Split the array into subarrays based on the split indices
            subarrays = [largest_contour[split_indices[i]:split_indices[i + 1] + 1] for i in range(len(split_indices) - 1)]
        
        outer_contour = max(subarrays, key=lambda x: x.shape[0])
        
        # int_contour = [outer_contour[0]]
        # for i in range(len(outer_contour)-1):
        #     mid = (outer_contour[i] + outer_contour[i+1]) / 2
        #     int_contour.append(mid)
        #     int_contour.append(outer_contour[i+1])
        # outer_contour = np.array(int_contour)
    else:
        outer_contour = largest_contour

    # Get candidate grasp points
    xt, yt, outward_normals, maxima_minima, combinations_list, normals_norm = get_extremum_points(outer_contour)

    # Filter grasp points based on different criteria
    grasps = []
    backup_grasps = []
    for combination in combinations_list:
        idx1, idx2 = combination
        angle = np.arccos(np.dot(outward_normals[idx1], outward_normals[idx2])/(np.linalg.norm(outward_normals[idx1])*np.linalg.norm(outward_normals[idx2])))*180/3.14

        # Filter based on angle between normals        
        if angle > 170:
            center = np.array([np.mean(xt), np.mean(yt)])
    
            pt1 = np.array([xt[idx1] - center[0], yt[idx1] - center[1]])
            pt2 = np.array([xt[idx2] - center[0], yt[idx2] - center[1]])
            grasp_center = np.array([(xt[idx1] + xt[idx2])/2, (yt[idx1] + yt[idx2])/2])
            
            f1 = outward_normals[idx1]
            f2 = outward_normals[idx2]
            forces = np.array([f1, f2])
            points = np.array([pt1 - grasp_center, pt2 - grasp_center])
            
            # Calculate moments, distance between points and distance between points and center
            moment = np.cross(points, forces)
            moment_mag = np.linalg.norm(moment)
            
            pt_dist = np.linalg.norm(pt1) + np.linalg.norm(pt2)
            dist = np.linalg.norm(pt1 - pt2)
            
            dist_vec = (pt1 - pt2)/dist
            angle1 = min(np.arccos(np.dot(dist_vec, outward_normals[idx1])/(np.linalg.norm(dist_vec)*np.linalg.norm(outward_normals[idx1])))*180/3.14,
                             np.arccos(np.dot(-dist_vec, outward_normals[idx1])/(np.linalg.norm(-dist_vec)*np.linalg.norm(outward_normals[idx1])))*180/3.14)
            angle2 = min(np.arccos(np.dot(dist_vec, outward_normals[idx2])/(np.linalg.norm(dist_vec)*np.linalg.norm(outward_normals[idx2])))*180/3.14,
                            np.arccos(np.dot(-dist_vec, outward_normals[idx2])/(np.linalg.norm(-dist_vec)*np.linalg.norm(outward_normals[idx2])))*180/3.14)                    
            angle_metric = angle1 + angle2

            # Filter based on distance between points
            if dist < 0.06 and angle_metric < 40:
                grasps.append([[idx1, idx2], pt_dist])
        
        # Get backup grasps with linient criteria
        elif angle > 150:
            center = np.array([np.mean(xt), np.mean(yt)])
    
            pt1 = np.array([xt[idx1] - center[0], yt[idx1] - center[1]])
            pt2 = np.array([xt[idx2] - center[0], yt[idx2] - center[1]])
            dist = np.linalg.norm(pt1 - pt2)

            if dist < 0.06:
                backup_grasps.append([[idx1, idx2], dist])
    
    # Sort the grasps based on the second element of the tuple and get the best grasp
    if len(grasps) == 0 and len(backup_grasps) == 0:
        grasps.append([[0, 1], 0])
        rospy.logerr("Grasp Not Found. Tune Parameters")
    if len(grasps) == 0:
        sorted_grasp = sorted(backup_grasps, key=lambda x: x[1])
    else:
        sorted_grasp = sorted(grasps, key=lambda x: x[1])

    best_grasp = sorted_grasp[0]
    x1 = xt[best_grasp[0][0]]
    y1 = yt[best_grasp[0][0]]
    x2 = xt[best_grasp[0][1]]
    y2 = yt[best_grasp[0][1]]
    
    if visualize:
        plt.cla()
        plt.clf()
        plt.plot(xt, yt)
        # plt.scatter(outer_contour[:, 0], outer_contour[:, 1], s=2, c='r')
        plt.plot(largest_contour[:, 0], largest_contour[:, 1], "c--", linewidth=2)
        plt.plot(largest_contour[:, 0], largest_contour[:, 1], "bo", markersize=2)

        candidate_points = np.array([xt[maxima_minima], yt[maxima_minima]]).T    
        plt.plot(candidate_points[:, 0], candidate_points[:, 1], "ro", markersize=2)    
        
        plt.plot(x1, y1, "bo", markersize=10)
        plt.plot(x2, y2, "bo", markersize=10)
    
        # plot_random_lines(xt, yt, outward_normals, color='green', random_indices=maxima_minima)
        
        plt.axis('square')    
        # plt.savefig('efd_result.png')
        # plt.show()
        
        # Publish the image for visualization and logging (TODO: please make this bad code better later)
        fig = plt.gcf()
        fig.set_dpi(500)
        fig.canvas.draw()
        img = np.array(fig.canvas.renderer.buffer_rgba())
        img_bgr = cv.cvtColor(img, cv.COLOR_RGBA2BGR)
        pub = rospy.Publisher("visualisation", Image, queue_size=1)
        bridge = CvBridge()
        pub.publish(bridge.cv2_to_imgmsg(img_bgr))

    return np.array([[x1, y1], [x2, y2]])

def get_grasp_from_img_file(img_file):
    '''
    Calculates grasps from an image input, for testing purposes
    '''
    im = cv.imread(img_file)
    assert im is not None, "file could not be read, check with os.path.exists()"
    imgray = cv.cvtColor(im, cv.COLOR_BGR2GRAY)
    
    ret, thresh = cv.threshold(imgray,230,255,cv.THRESH_BINARY_INV)
    contours, _ = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    largest_contour = max(contours, key=cv.contourArea).squeeze()
        
    grasp = get_grasp(largest_contour, visualize=True)
    return grasp

def handle_get_grasp(req):
    '''
    Service callback for grasp point calculation
    '''
    rospy.loginfo("Received grasp request")
    point_cloud = np.array(list(read_points(req.input_cloud, skip_nans=True)))
    print("Point Cloud shape: ", point_cloud.shape)
    
    z_mean = np.mean(point_cloud[:, 2])    
    print("Mean found")
    grasp = get_grasp(point_cloud[:, :2], visualize=False, split=True)
    print("grasp found")
    grasp = np.hstack((grasp, np.ones((grasp.shape[0], 1)) * z_mean))
    
    header = rospy.Header()
    header.stamp = rospy.Time.now()

    # Create fields for the PointCloud2 message
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]

    # Create the PointCloud2 message
    point_cloud = create_cloud(header, fields, grasp)
    print("point_cloud created")
    return EFDGraspResponse(point_cloud)

if __name__ == "__main__":
    # get_grasp_from_img_file('test1.jpg')

    rospy.init_node('efd_detector')
    rospy.Service('/top_surface_grasp_service/predict_efd_grasp', EFDGrasp, handle_get_grasp)
    
    rospy.spin()