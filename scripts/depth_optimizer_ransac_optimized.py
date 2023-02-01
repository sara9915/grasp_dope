#!/usr/bin/env python3

"""
This node refines the estimated pose from dope through depth measurements by an optimization algorithm
Input: estimated_pose, aligned_depth_to_color
Output: refined_pose
"""

from __future__ import print_function
import nvisii

import numpy as np

from grasp_dope.srv import depth_optimizer, depth_optimizerResponse
import rospy
from geometry_msgs.msg import PoseStamped

import rospy

from scipy.optimize import minimize_scalar
import matplotlib.pyplot as plt
import matplotlib.markers

from mpl_toolkits import mplot3d
from sklearn import linear_model


width_ = 640
height_ = 480

estimated_pose = PoseStamped()
real_depth_array = np.zeros((height_, width_))
real_depth_inliers=[]
pixel_cad_h = []
pixel_cad_w = []


plot_value = False


def delete_outliers():
    global real_depth_inliers
    for i in range(len(pixel_cad_h)):
        real_depth_inliers.append(real_depth_array[pixel_cad_h[i],pixel_cad_w[i]])

    line_ransac = linear_model.RANSACRegressor()
    line_ransac.fit(np.linspace(0,1,len(real_depth_inliers)).reshape(-1,1), real_depth_inliers)
    real_depth_inliers = line_ransac.predict(np.linspace(0,1,len(real_depth_inliers)).reshape(-1,1))


def get_cad_pixels():
    virtual_depth_map(0)
    for i in range(height_):
        for j in range(width_):
            if virtual_depth_array[i, j, 0] < 5 and virtual_depth_array[i, j, 0] > 0:
                pixel_cad_h.append(i)
                pixel_cad_w.append(j)

def convert_from_uvd(u, v, d, fx, fy, cx, cy):
    # d *= self.pxToMetre
    x_over_z = (cx - u) / fx
    y_over_z = (cy - v) / fy
    z = d / np.sqrt(1. + x_over_z**2 + y_over_z**2)
    return z


def virtual_depth_map(sigma):
    global virtual_depth_array

    x = estimated_pose.pose.position.x
    y = estimated_pose.pose.position.y
    z = estimated_pose.pose.position.z

    # print("x, y, z: ", x, y, z)
    camera.get_transform().look_at(
        at=(0, 0, 0),
        up=(0, 0, 1),
        eye=(0, 1, 0)
    )

    # Create a scene to use for exporting segmentations
    # apple
    cad_dimension = [0.09756118059158325, 0.08538994193077087,
                     0.09590171277523041]  # x, y, z, cuboid dimensions [m]
    sigma = sigma * 0.01
    scale_obj = (z-sigma)/z
    for i in range(3):
        cad_dimension[i] = cad_dimension[i]*scale_obj

    obj_mesh.get_transform().set_parent(camera.get_transform())
    obj_mesh.get_transform().set_position(
        (scale_obj*x, -(scale_obj*y), -(z - sigma)))
    # obj_mesh.get_transform().set_position(
    # (scale_obj*x, -(scale_obj*y -cad_dimension[1]/2), -(z - sigma)))

    obj_mesh.get_transform().set_rotation(nvisii.quat(estimated_pose.pose.orientation.w,
                                                      estimated_pose.pose.orientation.x, estimated_pose.pose.orientation.y, estimated_pose.pose.orientation.z))
    obj_mesh.get_transform().set_scale(nvisii.vec3(scale_obj))

    nvisii.sample_pixel_area(
        x_sample_interval=(.5, .5),
        y_sample_interval=(.5, .5)
    )

    virtual_depth_array = nvisii.render_data(
        width=int(width_),
        height=int(height_),
        start_frame=0,
        frame_count=1,
        bounce=int(0),
        options="depth"
    )

    # virtual_depth_array = nvisii.render_data_to_file(
    #     width=int(width_),
    #     height=int(height_),
    #     start_frame=1,
    #     frame_count=2,
    #     bounce=int(0),
    #     options="depth",
    #     file_path="virtual_depth_map.png"
    # )

    virtual_depth_array = np.array(
        virtual_depth_array).reshape(height_, width_, 4)
    virtual_depth_array = np.flipud(virtual_depth_array)


def cost_function(sigma):
    global plot_value
    # create virtual depth map based on estimated pose
    virtual_depth_map(sigma)

    sum = 0
    num_pixel_obj = 0
    virtual_depth_array_used = []
    real_depth_array_used = []
    ascisse = []
    ordinate = []

    intrinsics = camera.get_camera().get_intrinsic_matrix(width_, height_)
    
    for k in range(len(pixel_cad_h)):
        i = pixel_cad_h[k]
        j = pixel_cad_w[k]
        # d_hat = virtual_depth_array[i, j, 0] + 0.0042
        d_hat = convert_from_uvd(i, j, virtual_depth_array[i, j, 0],
                                       intrinsics[0][0], intrinsics[1][1], intrinsics[2][0], intrinsics[2][1])
        d = real_depth_inliers[k]
        sum = sum + pow((d-d_hat), 2)
        num_pixel_obj = num_pixel_obj+1
        virtual_depth_array_used.append(d_hat)
        real_depth_array_used.append(real_depth_array[i, j])
        ascisse.append(i)
        ordinate.append(j)

    if plot_value == True:
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.scatter(ascisse, ordinate, real_depth_inliers,
                'gray', marker='o')
        ax.scatter(ascisse, ordinate, virtual_depth_array_used,
                'blue', marker='o')
        ax.set_xlabel('i')
        ax.set_ylabel('j')
        plt.show()
    return sum/num_pixel_obj


def plot_function(sigma, values):
    plt.plot(sigma, values,
             color='blue', label='Cost Function Plot')
    plt.xlabel('sigma')
    plt.ylabel('cost_function')
    plt.legend()
    plt.show()


def handle_depth_optimizer(req):
    global estimated_pose, real_depth_array, camera, obj_mesh, plot_value

    estimated_pose = req.estimated_pose

    for i in range(height_):
        for j in range(width_):
            real_depth_array[i, j] = req.depth_matrix[i*width_+j]

    nvisii.initialize(headless=False, verbose=True)

    # # # # # # # # # # # # # # # # # # # # # # # # #
    camera = nvisii.entity.create(
        name="camera",
        transform=nvisii.transform.create("camera"),
        camera=nvisii.camera.create_from_intrinsics(
            name="camera",
            fx=610.59326171875,  # opt.focal_length,
            fy=610.605712890625,  # opt.focal_length,
            cx=317.7075500488281,  # (opt.width / 2),
            cy=238.1421356201172,  # (opt.height / 2),
            width=width_,
            height=height_
        )
    )

    camera.get_transform().look_at(
        at=(0, 0, 0),
        up=(0, 0, 1),
        eye=(0, 1, 0)
    )
    nvisii.set_camera_entity(camera)

    obj_to_load = "/home/workstation/dope_ros_ws/src/grasp_dope/scripts/models/Apple/Apple_4K/food_apple_01_4k.obj"
    obj_name = "apple"

    obj_mesh = nvisii.entity.create(
        name=obj_name,
        mesh=nvisii.mesh.create_from_file(obj_name, obj_to_load),
        transform=nvisii.transform.create(obj_name),
        material=nvisii.material.create(obj_name)
    )

    # # # # # # # # # # # # # # # # # # # # # # # # #
    get_cad_pixels()
    delete_outliers()
    res = minimize_scalar(cost_function)
    plot_value = True
    cost_function(res.x)

    print("function_min: ", res.fun)
    print("sigma_min: ", res.x)

    # values = np.zeros(20)
    # sigma = np.zeros(20)

    # for x in range(10,30):
    #     sigma[x+10] = x*0.01
    #     values[x+10] = cost_function(x)

    # plot_function(sigma,values)

    # let's clean up the GPU
    nvisii.deinitialize()
    return depth_optimizerResponse(res.x)


def depth_optimizer_service():
    rospy.init_node('depth_optimizer_server')
    s = rospy.Service('depth_optimizer', depth_optimizer,
                      handle_depth_optimizer)

    rospy.spin()


if __name__ == "__main__":
    depth_optimizer_service()
