"""
using codes from this link:

https://zhuanlan.zhihu.com/p/489887201?utm_id=0

"""

import glob
import os
import sys
import time
from datetime import datetime
import math
from tkinter import HORIZONTAL
import weakref
import random
import pandas as  pd
import open3d as o3d
from matplotlib import cm
from numpy.core.arrayprint import set_string_function

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from carla import Transform, Location, Rotation
import argparse
import logging
import random
import numpy as np

import open3d as o3d
from matplotlib import cm
from datetime import datetime

VIDIDIS = np.array(cm.get_cmap("plasma").colors)
VID_RANGE = np.linspace(0.0, 1.0, VIDIDIS.shape[0])

sys.path.append("/home/junchuan/Downloads/junchuan/CARLA/carla/LibCustomFunction") # append the LibCustomFunction
from enable_motion_distortion import LidarMotonDistortion # import the LidarMotonDistortion module

def lidar_callback(point_cloud, point_list):
    # We need to convert point cloud(carla-format) into numpy.ndarray
    data = np.copy(np.frombuffer(point_cloud.raw_data, dtype = np.dtype("f4")))
    data = np.reshape(data, (int(data.shape[0] / 5), 5))

    intensity = data[:, -2]
    intensity_col = 1.0 - np.log(intensity) / np.log(np.exp(-0.004 * 100))
    int_color = np.c_[
        np.interp(intensity_col, VID_RANGE, VIDIDIS[:, 0]),
        np.interp(intensity_col, VID_RANGE, VIDIDIS[:, 1]),
        np.interp(intensity_col, VID_RANGE, VIDIDIS[:, 2])]

    points = data[:, :-2] # we only use x, y, z coordinates
    points[:, 1] = -points[:, 1] # This is different from official script
    
    point_list.points = o3d.utility.Vector3dVector(points)
    point_list.colors = o3d.utility.Vector3dVector(int_color)

def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world =client.get_world()
    # world = client.load_world("Town04")

    try:
        original_settings = world.get_settings()
        settings = world.get_settings()
        traffic_manager = client.get_trafficmanager(8000)
        # traffic_manager.global_percentage_speed_difference(-133)
        traffic_manager.set_synchronous_mode(True)
        delta = 0.01
        # delta = 0.1
        settings.fixed_delta_seconds = delta
        settings.synchronous_mode = True
        world.apply_settings(settings)
        blueprint_library = world.get_blueprint_library()
        # --------------
        # Start recording
        # --------------
        """
        client.start_recorder('~/tutorial/recorder/recording01.log')
        """

        # --------------
        # Spawn ego vehicle
        # --------------

        ego_bp = world.get_blueprint_library().find('vehicle.lincoln.mkz_2017')
        ego_bp.set_attribute('role_name','ego')
        print('\nEgo role_name is set')
        # ego_color = random.choice(ego_bp.get_attribute('color').recommended_values)
        # ego_bp.set_attribute('color',ego_color)
        # print('\nEgo color is set')

        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if 0 < number_of_spawn_points:
            random.shuffle(spawn_points)
            # ego_transform = Transform(Location(x=-45,y=28,z=1.2), Rotation(pitch=0, yaw=-90, roll=0)) # 2
            ego_transform = Transform(Location(x=-49,y=-10,z=1.2), Rotation(pitch=0, yaw=90, roll=0)) # 1
            # ego_transform = Transform(Location(x=115,y=250,z=2), Rotation(pitch=0, yaw=-90, roll=0)) # seeside
            ego_vehicle = world.spawn_actor(ego_bp,ego_transform)
            print('\nEgo is spawned')
        else:
            logging.warning('Could not found any spawn points')

        # --------------
        # Add sensor to ego vehicle. 
        # --------------

        lidar_location = carla.Location(-0.5,0,2.095)
        lidar_rotation = carla.Rotation(0,0,0)
        lidar_transform = carla.Transform(lidar_location,lidar_rotation)

        lidar_list = []

        horizon_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast') # choose ray_cast or ray_cast_semantic
        horizon_bp.set_attribute("lidar_type", "Risley_prism") # choose lidar_type as Risley_prism
        horizon_bp.set_attribute("name","horizon") # choose lidar name as horizon
        horizon_bp.set_attribute("points_per_second", str(240000))
    

        pandar64_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        pandar64_bp.set_attribute("lidar_type","Surround") # choose lidar_type as mechanical
        pandar64_bp.set_attribute("name","pandar64") # choose lidar name as pandar64

        rs_m1_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        rs_m1_bp.set_attribute("lidar_type","Solid_state") # choose lidar_type as Solid_state
        rs_m1_bp.set_attribute("name","rs_m1") # choose lidar name as rs_m1

        horizon_bp.set_attribute("enable_ghost", "false") # set true or false to enable or disable the ghost mode. default false

        ego_horizon = world.spawn_actor(horizon_bp,lidar_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        # horizon_motion_distort = LidarMotonDistortion("./11_horizon/", 10) # initial LidarMotonDistortion with file_path and distortion delay_time
        # ego_horizon.listen(lambda data: horizon_motion_distort.enable_motion_distortion(data, True)) # set true to enable lidar motion distortion. default false
        lidar_list.append(ego_horizon)

        # ego_rs = world.spawn_actor(rs_m1_bp, lidar_transform, attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        # lidar_list.append(ego_rs)

        # 4. We set a point_list to store our point cloud
        point_list = o3d.geometry.PointCloud()

        # 5. Listen to the lidar to collect point cloud
        # ego_rs.listen(lambda data: lidar_callback(data, point_list))
        ego_horizon.listen(lambda data: lidar_callback(data, point_list))

        # 6. We set some basic settings for display with open3d
        vis = o3d.visualization.Visualizer()
        vis.create_window(
            window_name= "Display Point Cloud",
            width= 960,
            height= 540,
            left= 480,
            top= 270)

        vis.get_render_option().background_color = [0.05, 0.05, 0.05]
        vis.get_render_option().point_size = 1
        vis.get_render_option().show_coordinate_frame = True

        # --------------
        # Place spectator on ego spawning
        # --------------
        spectator = world.get_spectator()
        # world_snapshot = world.wait_for_tick()
        spectator.set_transform(ego_vehicle.get_transform())

        # --------------
        # Enable autopilot for ego vehicle
        # --------------

        ego_vehicle.set_autopilot(True)

        # --------------
        # Game loop. Prevents the script from finishing.
        # --------------
        frame=0
        dt0 = datetime.now()
        # ego_vehicle.apply_control(carla.VehicleControl(throttle=1, steer=0))
        i=0
        while True:
            if frame == 2:
                vis.add_geometry(point_list)
            vis.update_geometry(point_list)
            vis.poll_events()
            vis.update_renderer()
            time.sleep(0.005)

            world.tick()
            # while(process_time.total_seconds()<1):
            process_time = datetime.now() - dt0
            sys.stdout.write('\r' + 'FPS: ' + str(1.0 / process_time.total_seconds()))
            sys.stdout.flush()
            dt0 = datetime.now()

            time.sleep(0.01)
            dt0 = datetime.now()
            frame+=1
            spectator = world.get_spectator()
            transform = ego_vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location+carla.Location(z=50),carla.Rotation(pitch=-90,yaw=0)))
 
    finally:
        # --------------
        # Stop recording and destroy actors
        # --------------
        world.apply_settings(original_settings)
        traffic_manager.set_synchronous_mode(False)
        client.stop_recorder()
        ego_vehicle.destroy()
        for lidar in lidar_list:
            lidar.destroy()
        

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\nDone with tutorial_ego.')
