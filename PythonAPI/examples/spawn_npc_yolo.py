#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Spawn NPCs into the simulation"""

import glob
import os
import sys
import cv2
import numpy as np

import carla
import argparse
import logging
import random

# for socket
import socket
from PIL import Image
import time

def to_bgra_array(image):
    print(str(image.height))
    print(str(image.width))

    """Convert a CARLA raw image to a BGRA numpy array."""
    #if not isinstance(image, sensor.Image):
    #    raise ValueError("Argument must be a carla.sensor.Image")
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    return array

def to_rgb_array(image):
    """Convert a CARLA raw image to a RGB numpy array."""
    array = to_bgra_array(image)
    # Convert BGRA to RGB.
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    return array

def to_bgr_array(image):
    """Convert a CARLA raw image to a BGR numpy array."""
    array = to_bgra_array(image)
    # Convert BGRA to BGR.
    array = array[:, :, :3]
    return array

def recvall(sock, n):
    # Helper function to recv n bytes or return None if EOF is hit
    data = b''
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data += packet
    return data


def do_something(client_socket, data):
    print('do_something\n')
    #print(type(data))
    #print(str(len(data)))

    start_time = time.time()
    #time.sleep(10)
    imgnp=to_bgr_array(data)
    im = Image.fromarray(imgnp)
    im.save("your_file.jpeg")
    buffer=im.tobytes()
    #cv2.imshow('imwin', imgnp)
    print(type(buffer))
    print(str(len(buffer)))
    print("XXXXX")
    #return
    bbytee = client_socket.send(buffer)
    #bbytee = client_socket.send(imgnp)
    if (bbytee < 0):
        print("send failed")
    else:
        print("sent bytes " + str(bbytee))

    data_received = recvall(client_socket, bbytee)

    time_elapsed = time.time() - start_time
    bandwidth = (bbytee*2 / time_elapsed) / 1e6;
    fps = 1 / (time_elapsed);
    print("fps= " + str(fps) + ", BW= " + str(bandwidth) + " MB/s")

    if (bbytee < 0):
        print("recv failed")
    else:
        print("recv bytes " + str(len(data_received)))



try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


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
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=10,
        type=int,
        help='number of vehicles (default: 10)')
    argparser.add_argument(
        '-d', '--delay',
        metavar='D',
        default=2.0,
        type=float,
        help='delay in seconds between spawns (default: 2.0)')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='avoid spawning vehicles prone to accidents')
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    actor_list = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)

    try:

        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect(('9.4.125.67', 5555))

        world = client.get_world()
        blueprints = world.get_blueprint_library().filter('vehicle.*')

        if args.safe:
            blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
            blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
            blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]

        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if args.number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif args.number_of_vehicles > number_of_spawn_points:
            msg = 'requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
            args.number_of_vehicles = number_of_spawn_points

        # @todo cannot import these directly.
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        batch = []
        for n, transform in enumerate(spawn_points):
            if n >= args.number_of_vehicles:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            blueprint.set_attribute('role_name', 'autopilot')
            batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True)))

        for response in client.apply_batch_sync(batch):
            if response.error:
                logging.error(response.error)
            else:
                actor_list.append(response.actor_id)

        print('spawned %d vehicles, press Ctrl+C to exit.' % len(actor_list))

        #blueprint_veh = world.get_blueprint_library().filter('vehicle.1*')

        # Find the blueprint of the sensor.
        blueprint_cam = world.get_blueprint_library().find('sensor.camera.rgb')
        #blueprint.set_attribute('post_processing', 'SceneFinal')
        # Modify the attributes of the blueprint to set image resolution and field of view.
        blueprint_cam.set_attribute('image_size_x', '640')
        blueprint_cam.set_attribute('image_size_y', '480')
        blueprint_cam.set_attribute('fov', '58')
        # Set the time in seconds between sensor captures
        blueprint_cam.set_attribute('sensor_tick', '0.1')
        # Provide the position of the sensor relative to the vehicle.
        transform = carla.Transform(carla.Location(x=0.8, z=1.7))
        # Tell the world to spawn the sensor, don't forget to attach it to your vehicle actor.
        m = world.get_map()
        start_pose = random.choice(m.get_spawn_points())
        vehicle = world.spawn_actor(random.choice(blueprints.filter('vehicle.*')), start_pose)
        sensor = world.spawn_actor(blueprint_cam, transform, attach_to=vehicle)
        # Subscribe to the sensor stream by providing a callback function, this function is
        # called each time a new image is generated by the sensor.
        sensor.listen(lambda data: do_something(client_socket, data))



#        #settings = carla.CarlaSettings()
#        camera = Camera('MyCamera', PostProcessing='Depth')
#        camera.set(FOV=90.0)
#        camera.set_image_size(800, 600)
#        camera.set_position(x=0.30, y=0, z=1.30)
#        camera.set_rotation(pitch=0, yaw=0, roll=0)
#        carla_settings.add_sensor(camera)
#        # ...
#        #carla_client.load_settings(settings)

#        measurements, sensor_data = carla_client.read_data()
#        image = sensor_data['MyCamera']
#        cv2.imshow('imwin', image)

        print('OK')

        while True:
            world.wait_for_tick()

    finally:

        print('\ndestroying %d actors' % len(actor_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
