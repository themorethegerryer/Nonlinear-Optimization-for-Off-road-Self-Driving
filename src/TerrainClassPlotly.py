#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as R
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import plotly.io as pio
pio.renderers.default='browser'

class Terrain(object):
    def __init__(self, origin_ft, orientation_R, horizontal_angle_deg = 33, vertical_angle_deg = 21, max_rad_ft = 5, color = 'y'):
        # Initialize self
        self.origin_ft = origin_ft
        self.orientation_R = orientation_R
        self.horizontal_angle_deg = horizontal_angle_deg
        self.vertical_angle_deg = vertical_angle_deg
        self.max_rad_ft = max_rad_ft
        self.color = color
        # You can add other color options here
        if self.color == 'y':
            self.colorscale = [[0, 'rgb(255,255,0)'], [1, 'rgb(255,255,0)']]

    def plot_camera(self):
        # Create and plot the camera's shape
        camera_r_points = np.linspace(0, self.max_rad_ft, num = num_sim_points)
        camera_theta_points = np.linspace(-self.horizontal_angle_deg + self.orientation_R.as_euler('xyz', degrees=True)[2], self.horizontal_angle_deg + self.orientation_R.as_euler('xyz', degrees=True)[2], num = num_sim_points)
        camera_phi_points = np.linspace(-self.vertical_angle_deg + self.orientation_R.as_euler('xyz', degrees=True)[1], self.vertical_angle_deg + self.orientation_R.as_euler('xyz', degrees=True)[1], num = num_sim_points)
        camera_r_mesh, camera_theta_mesh, camera_phi_mesh = np.meshgrid(camera_r_points, camera_theta_points, camera_phi_points)
        camera_r_meshes = []
        camera_theta_meshes = []
        camera_phi_meshes = []
        for ii in range(6):
            camera_r_meshes.append(np.take(camera_r_mesh, ii%2 - 1, axis=ii%3))
            camera_theta_meshes.append(np.take(camera_theta_mesh, ii%2 - 1, axis=ii%3))
            camera_phi_meshes.append(np.take(camera_phi_mesh, ii%2 - 1, axis=ii%3))
        camera_x_meshes = []
        camera_y_meshes = []
        camera_z_meshes = []
        for ii in range(len(camera_r_meshes)):
            camera_x_meshes.append((camera_r_meshes[ii] * np.cos(camera_theta_meshes[ii]*deg_to_rad) * np.cos(camera_phi_meshes[ii]*deg_to_rad)) + self.origin_ft[0])
            camera_y_meshes.append((camera_r_meshes[ii] * np.sin(camera_theta_meshes[ii]*deg_to_rad) * np.cos(camera_phi_meshes[ii]*deg_to_rad)) + self.origin_ft[1])
            camera_z_meshes.append((camera_r_meshes[ii] * np.cos(camera_theta_meshes[ii]*deg_to_rad) * np.sin(camera_phi_meshes[ii]*deg_to_rad) * -1) + self.origin_ft[2])
        for ii in range(len(camera_x_meshes)):
            fig.add_trace(go.Surface(x=camera_x_meshes[ii], y=camera_y_meshes[ii], z=camera_z_meshes[ii], colorscale = self.colorscale), 1, 1)
        #     ax.plot_surface(camera_x_meshes[ii], camera_y_meshes[ii], camera_z_meshes[ii], color = self.color, alpha = 1, antialiased = True)


# Simulation variables
num_sim_points = 25
deg_to_rad = 2.0*np.pi/360.0
# Task Space Definitions
task_space_rad_max_ft = 3.0
task_space_rad_min_ft = 1.5
task_space_horizontal_max_deg = 30.0
task_space_horizontal_min_deg = -30.0
task_space_vertical_max_deg = 10.0
task_space_vertical_min_deg = -90.0
# Hand Spread Definitions
hand_space_rad_diff_ft = 0.0
hand_space_horizontal_angle_diff_deg = 25.0
hand_space_vertical_angle_diff_deg = 10.0
# Buffer Space Definitions
buffer_space_rad_diff_ft = 3.0/12.0
buffer_space_horizontal_angle_diff_deg = 5.0
buffer_space_vertical_angle_diff_deg = 5.0
# Additionally there is a stability space, which includes the task
# space and buffer space and additionally room for the user to move
# their body around during stabilization.

# Vision System Creation
cameras = []
# Camera class has the following variables:
#   origin_ft - origin (x,y,z) in feet
#   orientation_R - orientation as a scipy rotation object
#   horizontal_angle_deg - opt - the horizontal angular spread of the camera in degrees
#   vertical_angle_deg - opt - the vertical angular spread of the camera in degrees
#   max_rad_ft - opt - the maximum depth of the camera in feet (used to keep plotting sensible)
#   color - opt - string representing the color of the vision cone
#
# Append cameras
cameras.append(Camera(np.array([-6.0/12.0, -8.0/12.0, 8.0/12.0]), R.from_euler('xyz', [0,-20,20], degrees=True)))


# Extra points for reference
chest_origin = np.array([0,0,0])
original_camera_position = np.array([0,0,0])

# Check the task space
# Create and plot task space volume
task_r_points = np.linspace(task_space_rad_min_ft, task_space_rad_max_ft, num = num_sim_points)
task_theta_points = np.linspace(task_space_horizontal_min_deg, task_space_horizontal_max_deg, num = num_sim_points)
task_phi_points = np.linspace(task_space_vertical_min_deg, task_space_vertical_max_deg, num = num_sim_points)
task_r_mesh, task_theta_mesh, task_phi_mesh = np.meshgrid(task_r_points, task_theta_points, task_phi_points)
task_r_meshes = []
task_theta_meshes = []
task_phi_meshes = []
for ii in range(6):
    task_r_meshes.append(np.take(task_r_mesh, ii%2 - 1, axis=ii%3))
    task_theta_meshes.append(np.take(task_theta_mesh, ii%2 - 1, axis=ii%3))
    task_phi_meshes.append(np.take(task_phi_mesh, ii%2 - 1, axis=ii%3))
task_x_meshes = []
task_y_meshes = []
task_z_meshes = []
for ii in range(len(task_r_meshes)):
    task_x_meshes.append(task_r_meshes[ii] * np.cos(task_theta_meshes[ii]*deg_to_rad) * np.cos(task_phi_meshes[ii]*deg_to_rad))
    task_y_meshes.append(task_r_meshes[ii] * np.sin(task_theta_meshes[ii]*deg_to_rad) * np.cos(task_phi_meshes[ii]*deg_to_rad))
    task_z_meshes.append(task_r_meshes[ii] * np.cos(task_theta_meshes[ii]*deg_to_rad) * np.sin(task_phi_meshes[ii]*deg_to_rad) * -1)
fig = make_subplots(rows=1, cols=1,
                    specs=[[{'is_3d': True}]],
                    subplot_titles=['Coverage of Vision System Over Task Space']
                    )
for ii in range(len(task_x_meshes)):
    fig.add_trace(go.Surface(x=task_x_meshes[ii], y=task_y_meshes[ii], z=task_z_meshes[ii], colorscale = [[0, 'rgb(0,0,255)'], [1, 'rgb(0,0,255)']]), 1, 1)
for ii in range(len(cameras)):
    cameras[ii].plot_camera()
fig.update_layout()
fig.show()

# Matplotlib code
# fig = plt.figure()
# ax = plt.axes(projection='3d')
# for ii in range(len(task_x_meshes)):
#     ax.plot_surface(task_x_meshes[ii], task_y_meshes[ii], task_z_meshes[ii], color = 'b', alpha = 1, antialiased=True)
# ax.scatter3D(chest_origin[0], chest_origin[1], chest_origin[2], color = 'b', alpha = .8, lw = 1, s = 5)
# # Plot camera volumes
# for ii in range(len(cameras)):
#     cameras[ii].plot_camera()
# # ax.set_xlim(np.min(task_space_goals_cartesian)*1.1, np.max(task_space_goals_cartesian)*1.1)
# # ax.set_ylim(np.min(task_space_goals_cartesian)*1.1, np.max(task_space_goals_cartesian)*1.1)
# # ax.set_zlim(np.min(task_space_goals_cartesian)*1.1, np.max(task_space_goals_cartesian)*1.1)
# ax.set_xlabel('$X$', fontsize=15, rotation=150)
# ax.set_ylabel('$Y$', fontsize=15)
# ax.set_zlabel('$Z$', fontsize=15, rotation=60)
# ax.set_title('Coverage of Vision System Over Task Space')
# ax.view_init(elev=5, azim=110)
#fig.savefig(sys.argv[1] + "/Graphs" + "/tf" + "_" + "End_Effector_Position_in_Base_Frame_Colored_with_Time" + ".png", dpi = 300, bbox_inches = 'tight')


# Check the hand space
# Create and plot task space volume
hand_r_points = np.linspace(task_space_rad_min_ft - hand_space_rad_diff_ft, task_space_rad_max_ft + hand_space_rad_diff_ft, num = num_sim_points)
hand_theta_points = np.linspace(task_space_horizontal_min_deg - hand_space_horizontal_angle_diff_deg, task_space_horizontal_max_deg + hand_space_horizontal_angle_diff_deg, num = num_sim_points)
hand_phi_points = np.linspace(task_space_vertical_min_deg - hand_space_vertical_angle_diff_deg, task_space_vertical_max_deg + hand_space_vertical_angle_diff_deg, num = num_sim_points)
hand_r_mesh, hand_theta_mesh, hand_phi_mesh = np.meshgrid(hand_r_points, hand_theta_points, hand_phi_points)
hand_r_meshes = []
hand_theta_meshes = []
hand_phi_meshes = []
for ii in range(6):
    hand_r_meshes.append(np.take(hand_r_mesh, ii%2 - 1, axis=ii%3))
    hand_theta_meshes.append(np.take(hand_theta_mesh, ii%2 - 1, axis=ii%3))
    hand_phi_meshes.append(np.take(hand_phi_mesh, ii%2 - 1, axis=ii%3))
hand_x_meshes = []
hand_y_meshes = []
hand_z_meshes = []
for ii in range(len(hand_r_meshes)):
    hand_x_meshes.append(hand_r_meshes[ii] * np.cos(hand_theta_meshes[ii]*deg_to_rad) * np.cos(hand_phi_meshes[ii]*deg_to_rad))
    hand_y_meshes.append(hand_r_meshes[ii] * np.sin(hand_theta_meshes[ii]*deg_to_rad) * np.cos(hand_phi_meshes[ii]*deg_to_rad))
    hand_z_meshes.append(hand_r_meshes[ii] * np.cos(hand_theta_meshes[ii]*deg_to_rad) * np.sin(hand_phi_meshes[ii]*deg_to_rad) * -1)
fig = make_subplots(rows=1, cols=1,
                    specs=[[{'is_3d': True}]],
                    subplot_titles=['Coverage of Vision System Over Task + Hand Space']
                    )
for ii in range(len(hand_x_meshes)):
    fig.add_trace(go.Surface(x=hand_x_meshes[ii], y=hand_y_meshes[ii], z=hand_z_meshes[ii], colorscale = [[0, 'rgb(0,0,255)'], [1, 'rgb(0,0,255)']]), 1, 1)
for ii in range(len(cameras)):
    cameras[ii].plot_camera()
fig.update_layout()
fig.show()


# Check the + buffer space
# Create and plot task space volume
buffer_r_points = np.linspace(task_space_rad_min_ft - hand_space_rad_diff_ft - buffer_space_rad_diff_ft, task_space_rad_max_ft + hand_space_rad_diff_ft + buffer_space_rad_diff_ft, num = num_sim_points)
buffer_theta_points = np.linspace(task_space_horizontal_min_deg - hand_space_horizontal_angle_diff_deg - buffer_space_horizontal_angle_diff_deg, task_space_horizontal_max_deg + hand_space_horizontal_angle_diff_deg + buffer_space_horizontal_angle_diff_deg, num = num_sim_points)
buffer_phi_points = np.linspace(task_space_vertical_min_deg - hand_space_vertical_angle_diff_deg - buffer_space_vertical_angle_diff_deg, task_space_vertical_max_deg + hand_space_vertical_angle_diff_deg + buffer_space_vertical_angle_diff_deg, num = num_sim_points)
buffer_r_mesh, buffer_theta_mesh, buffer_phi_mesh = np.meshgrid(buffer_r_points, buffer_theta_points, buffer_phi_points)
buffer_r_meshes = []
buffer_theta_meshes = []
buffer_phi_meshes = []
for ii in range(6):
    buffer_r_meshes.append(np.take(buffer_r_mesh, ii%2 - 1, axis=ii%3))
    buffer_theta_meshes.append(np.take(buffer_theta_mesh, ii%2 - 1, axis=ii%3))
    buffer_phi_meshes.append(np.take(buffer_phi_mesh, ii%2 - 1, axis=ii%3))
buffer_x_meshes = []
buffer_y_meshes = []
buffer_z_meshes = []
for ii in range(len(buffer_r_meshes)):
    buffer_x_meshes.append(buffer_r_meshes[ii] * np.cos(buffer_theta_meshes[ii]*deg_to_rad) * np.cos(buffer_phi_meshes[ii]*deg_to_rad))
    buffer_y_meshes.append(buffer_r_meshes[ii] * np.sin(buffer_theta_meshes[ii]*deg_to_rad) * np.cos(buffer_phi_meshes[ii]*deg_to_rad))
    buffer_z_meshes.append(buffer_r_meshes[ii] * np.cos(buffer_theta_meshes[ii]*deg_to_rad) * np.sin(buffer_phi_meshes[ii]*deg_to_rad) * -1)
fig = make_subplots(rows=1, cols=1,
                    specs=[[{'is_3d': True}]],
                    subplot_titles=['Coverage of Vision System Over Task + Hand + Buffer Space']
                    )
for ii in range(len(buffer_x_meshes)):
    fig.add_trace(go.Surface(x=buffer_x_meshes[ii], y=buffer_y_meshes[ii], z=buffer_z_meshes[ii], colorscale = [[0, 'rgb(0,0,255)'], [1, 'rgb(0,0,255)']]), 1, 1)
for ii in range(len(cameras)):
    cameras[ii].plot_camera()
fig.update_layout()
fig.show()