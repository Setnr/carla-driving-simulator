import glob
import os
import sys
import random
import time
import math
import datetime
import pygame
import json
import numpy as np 
from pathlib import Path
import argparse
import weakref

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla
from automatic_control import  KeyboardControl, FadingText, HelpText
from carla import ColorConverter as cc

import h5

SIMULATION_TIME = 150 # In Seconds
TIMESTAMP = datetime.datetime.timestamp(datetime.datetime.now())
CHUNNK = 5000
RENDER_HUD = True

display = pygame.display.set_mode(
            (1280, 720),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

from enum import Enum
class Scenario(Enum):

    RadarBlocked = 1
    RadarLooseContact = 2
    RadarConstantShift = 4
    RadarVibration = 8
    RadarDisturbance = 16
    RadarRandomShift = 32
    RadarCollosionShift = 64
    RadarSpoofing = 128

class VehicleEnvironment:
    vehicle_list = []
    sensor_list = []
    walker_list = []

    # Sensors
    front_radar = None
    front_radar_2 = None

    # Ego Vehicle
    vehicle = None
    
    # Number of other actors
    num_vehicles = 15
    num_pedestrians = 40

    def __init__(self, hud, world, store_radar_data):

        self.vehicle_list = []
        self.sensor_list = []
        self.walker_list = []
        self.hud = hud
        self.camera_manager = None
        self.world = world
        self.store_radar_data = store_radar_data
        self.detection_counter = 0 # Detection as in Frame
        self.dpoint_counter = 0 # Point as in database entry
        self.hdffilename = f"C:\carla\PythonAPI\examples\Data\\"
        if not os.path.exists(self.hdffilename):
            os.makedirs(self.hdffilename)
        if not os.path.exists(self.hdffilename+"camera"):
            os.makedirs(self.hdffilename+"camera")
        self.hdffile = None
        # Library of actor blueprints available in the given world
        self.blueprint_library = self.world.get_blueprint_library()
        # Setup ego vehicle
        self.vehicle_model = "model3"
        self.vehicle_bp = self.blueprint_library.filter(self.vehicle_model)[0]
        self.vehicle_bp.set_attribute('color', "138, 25, 102")
        self.vehicle_bp.set_attribute('role_name', "IQZ Vehicle")
        self.spawn_points = self.world.get_map().get_spawn_points()
        self.walker_controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
        self.runner_percentage = 0.1
        self.street_crossing_percentage = 0.15
        self.runner = 0

        self.pedestrian_spawn_points = []
        for i in range(self.num_pedestrians): #perhaps use more
            sp = carla.Transform()
            sp.location = world.get_random_location_from_navigation()
            if sp.location != None:
                self.pedestrian_spawn_points.append(sp)

        if self.store_radar_data: 
            self.hdffile = h5.RadarScenes(self.hdffilename)


    def destroy_actor(self, actor_list):
        for actor in actor_list:
            actor.destroy()

    def clamp(self, min_v, max_v, value):
        return max(min_v, min(value, max_v))

    def process_radar_data(self, data, radar):
        velocity_range = 7.5 # m/s
        current_rot = data.transform.rotation

        for detect in data:
            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            # The 0.25 adjusts a bit the distance so the dots can
            # be properly seen
            fw_vec = carla.Vector3D(x=detect.depth - 0.25)
            carla.Transform(carla.Location(), carla.Rotation(pitch=current_rot.pitch + alt, 
                yaw=current_rot.yaw + azi, roll=current_rot.roll)).transform(fw_vec)
            detection_pos = data.transform.location + fw_vec

            norm_velocity = detect.velocity / velocity_range # range [-1, 1]
            r = int(self.clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
            g = int(self.clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
            b = int(abs(self.clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
            self.world.debug.draw_point(detection_pos, size=0.075, life_time=0.06, 
                persistent_lines=False, color=carla.Color(255, 255, 255))

            if self.store_radar_data:
                self.hdffile.SaveRadarData(detect,radar,data.transform)


    def render(self, display):
        """Render world"""
        self.camera_manager.render(display)
        self.hud.render(display)
        if self.store_radar_data:
            pygame.image.save(display, self.hdffilename +f"camera/{self.hdffile.GetTimeStamp()}.jpg")

    def tick(self, clock):
        """Method for every tick"""
        self.hud.tick(self, clock)
        
        if self.hdffile: 
            self.hdffile.NewTimeStamp()
        if self.vehicle.is_at_traffic_light():
            traffic_light = self.vehicle.get_traffic_light()
            if traffic_light.get_state() == carla.TrafficLightState.Red:
                traffic_light.set_state(carla.TrafficLightState.Green)
                traffic_light.set_green_time(4.0)
       

    def spawn_vehicle(self, blueprint):
        actor = ""
        spawn = ""
        for spawn in self.spawn_points:
            actor = self.world.try_spawn_actor(blueprint, spawn)
            if actor:
                break

        if spawn in self.spawn_points: self.spawn_points.remove(spawn)
        return actor

    def spawn_pedestrian(self, blueprint):
        actor = ""
        spawn = ""
        for spawn in self.pedestrian_spawn_points:
            actor = self.world.try_spawn_actor(blueprint, spawn)
            if actor:
                break

        if spawn in self.spawn_points: self.spawn_points.remove(spawn)
        # Initialize the walker AI
        controller = self.world.spawn_actor(self.walker_controller_bp, carla.Transform(), actor)
        self.world.tick()
        self.world.set_pedestrians_cross_factor(self.street_crossing_percentage)
        controller.start()
        controller.go_to_location(self.world.get_random_location_from_navigation())
        if (random.random() > self.runner_percentage): # Walking
            controller.set_max_speed(float(blueprint.get_attribute('speed').recommended_values[1]))
        else: # Running
            controller.set_max_speed(float(blueprint.get_attribute('speed').recommended_values[2]))
            self.runner += 1
        return actor


    def setup(self):
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_id = self.camera_manager.transform_index if self.camera_manager is not None else 0

        # Add ego vehicle
        self.vehicle = self.spawn_vehicle(self.vehicle_bp)
        if not self.vehicle:
            print("Error spawning ego vehicle")
            exit(1)
        self.vehicle_list.append(self.vehicle)
        self.vehicle.set_autopilot(True)

        # Add Camera Manager to follow the ego vehicle with a view
        self.camera_manager = CameraManager(self.vehicle, self.hud,2.2)
        self.camera_manager.transform_index = cam_pos_id
        self.camera_manager.set_sensor(cam_index, notify=False)

        # Initialize and Setup front RADAR
        front_radar_model = self.world.get_blueprint_library().find('sensor.other.faulty_radar')
        front_radar_model.set_attribute('horizontal_fov', str(35))
        front_radar_model.set_attribute('vertical_fov', str(20))
        front_radar_model.set_attribute('range', str(80))
        front_radar_model.set_attribute("scenario",str(Scenario.RadarDisturbance.value))

        front_radar_model.set_attribute('RadarDisturbance_Interval', str(10))
        front_radar_model.set_attribute('RadarDisturbance_Duration', str(8))
        front_radar_model.set_attribute('RadarDisturbance_StartOffset', str(30))
        front_radar_model.set_attribute('RadarDisturbance_ProgressionRate', str(0))

        bound_x = 0.5 + self.vehicle.bounding_box.extent.x
        bound_z = 0.5 + self.vehicle.bounding_box.extent.z

        front_radar_position = carla.Location(x=bound_x, z=bound_z)
        front_radar_rotation = carla.Rotation(pitch=5,yaw=0)
        front_radar_transform = carla.Transform(front_radar_position, front_radar_rotation)
        self.front_radar = self.world.spawn_actor(front_radar_model, front_radar_transform, 
            attach_to=self.vehicle, attachment_type=carla.AttachmentType.Rigid)
        self.sensor_list.append(self.front_radar)
        self.front_radar.ID = 1
        self.front_radar.Yaw = 0 
        self.front_radar.listen(lambda data: self.process_radar_data(data,self.front_radar))

        front_radar_position = carla.Location(x=(bound_x*-1), z=bound_z)
        front_radar_rotation = carla.Rotation(pitch=5, yaw=0)
        front_radar_transform = carla.Transform(front_radar_position, front_radar_rotation)
        #self.front_radar_2 = self.world.spawn_actor(front_radar_model, front_radar_transform, 
        #    attach_to=self.vehicle, attachment_type=carla.AttachmentType.Rigid)
        #self.sensor_list.append(self.front_radar_2)
        #self.front_radar_2.ID = 2
        #self.front_radar_2.Yaw = 180 
        #self.front_radar_2.listen(lambda data: self.process_radar_data(data,self.front_radar_2))


        # Add other actors
        for i in range(self.num_vehicles):
            fresh_bp = random.choice(self.blueprint_library.filter('vehicle.*.*'))
            actor = self.spawn_vehicle(fresh_bp)
            if not actor:
                print("Error spawning ego vehicle")
                exit(1)
            self.vehicle_list.append(actor)
            actor.set_autopilot(True)

        for i in range(self.num_pedestrians):
            fresh_bp = random.choice(self.blueprint_library.filter('walker.pedestrian.*'))
            if fresh_bp.has_attribute('is_invincible'):
                fresh_bp.set_attribute('is_invincible', 'false')
            actor = self.spawn_pedestrian(fresh_bp)
            self.walker_list.append(actor)
        # Add Controller

        print("Spawned NPCs: %d vehicles • %d pedestrians • thereof %d walkers and %d runners" % 
            (len(self.vehicle_list)-1, len(self.walker_list), len(self.walker_list)-self.runner, self.runner))

        time.sleep(5) # Just making sure everything is spawned and ready
        self.start_time = time.time()

    def destruct(self):
        #self.vehicle.destroy()

        self.destroy_actor(self.vehicle_list)
        self.destroy_actor(self.sensor_list)
        self.destroy_actor(self.walker_list)

        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

        if self.hdffile:
            self.hdffile.close()


# -- HUD ---------------------------------------------------------------
class HUD(object):
    """Class for HUD text"""

    def __init__(self, width, height,):
        """Constructor method"""
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 24), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        global RENDER_HUD
        self._show_info = RENDER_HUD 
        self._info_text = []
        self._server_clock = pygame.time.Clock()

    def on_world_tick(self, timestamp):
        """Gets informations from the world at every tick"""
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame_count
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, env, clock):
        """HUD method for every tick"""
        self._notifications.tick(env, clock)
        if not self._show_info:
            return
        transform = env.vehicle.get_transform()
        velocity = env.vehicle.get_velocity()
        control = env.vehicle.get_control()
        heading = 'N' if abs(transform.rotation.yaw) < 89.5 else ''
        heading += 'S' if abs(transform.rotation.yaw) > 90.5 else ''
        heading += 'E' if 179.5 > transform.rotation.yaw > 0.5 else ''
        heading += 'W' if -0.5 > transform.rotation.yaw > -179.5 else ''
        #colhist = world.collision_sensor.get_collision_history()
        #collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        #max_col = max(1.0, max(collision))
        #collision = [x / max_col for x in collision]
        vehicles = env.world.get_actors().filter('vehicle.*')

        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % env.vehicle_model,
            'Map:     % 20s' % env.world.get_map().name.split('/')[-1],
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)),
            u'Heading:% 16.0f\N{DEGREE SIGN} % 2s' % (transform.rotation.yaw, heading),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (transform.location.x, transform.location.y)),
            #'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
            'Height:  % 18.0f m' % transform.location.z,
            '']
        if isinstance(control, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', control.throttle, 0.0, 1.0),
                ('Steer:', control.steer, -1.0, 1.0),
                ('Brake:', control.brake, 0.0, 1.0),
                ('Reverse:', control.reverse),
                ('Hand brake:', control.hand_brake),
                ('Manual:', control.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(control.gear, control.gear)]
        #self._info_text += [
        #    '',
        #    'Collision:',
        #    collision,
        #    '',
        #    'Number of vehicles: % 8d' % len(vehicles)]

        #if len(vehicles) > 1:
        #    self._info_text += ['Nearby vehicles:']

    def toggle_info(self):
        """Toggle info on or off"""
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        """Notification text"""
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        """Error text"""
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        """Render for HUD class"""
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)
        

# ------------------------------------------------------------------------

def simulate(recording, store_radar_data):
    """ Setups the required objects and starts the simulation according to previously
    parsed argumemts"""
    print("Starting IQZ Vehicle Simulation")

    # For more repetitive results
   # random.seed(1)
   # np.random.seed(1)
    #tf.set_random_seed(1)

    # Memory fraction, used mostly when trai8ning multiple agents
    #gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=MEMORY_FRACTION)
    #backend.set_session(tf.Session(config=tf.ConfigProto(gpu_options=gpu_options)))
    print("Initializing Pygame ...")
    pygame.init()
    pygame.font.init()

    # Create agent and environment
    hud = HUD(1280, 720)

    world = None
    env = None
    recordingfile = ""
    try:
        print("Connecting to Simulation Environment ...")
        # Create a client object that can run the agent and connect to the world
        print("Creating client and connecting to host ...")
        client = carla.Client('192.168.178.42', 2000)
        client.set_timeout(5.0) # might need a longer timeout with bad hardware
        time.sleep(5.0)

        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(False)            
        
        print("Creating Display ...")
        # Get world object that was started by another source (hopefully)
        world = client.get_world()
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)
            
        world.on_tick(lambda snapshot: hud.on_world_tick(snapshot))
        print("Setting up Vehicle Environment ...")
        env = VehicleEnvironment(hud, world, store_radar_data)
        env.setup()
        print("Vehicel got setup")
        controller = KeyboardControl(env)
        print("Running Simulation Loop ...")
        clock = pygame.time.Clock()
        end_time = env.start_time + SIMULATION_TIME
        if recording:
            recordingfile = f"recording{TIMESTAMP}.rec"  
            client.start_recorder(str(Path().resolve().joinpath(recordingfile)), True)
        
        while time.time() <= end_time:
            clock.tick()
            world.tick()
            if controller.parse_events():
                return
            env.tick(clock)
            env.render(display)
            pygame.display.flip()

    finally:
        # Clean up and call destruction routines
        if recording:
            client.stop_recorder()

        if world is not None:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)
        if env is not None:
            env.destruct()
        

    pygame.quit()
   

def replay_recording(recording):

    path = Path(recording)
    if not path.is_file():
        print("Could not find file: %s" % recording)
        return
    if not os.path.isabs(recording):
        path = os.path.abspath(path)

    print(path)

    print("Creating client and connecting to host ...")
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0) # might need a longer timeout with bad hardware
    time.sleep(5.0)
    world = client.get_world()

    # set to ignore the hero vehicles or not
    client.set_replayer_ignore_hero(False)

    # replay the session
    print(client.replay_file(path, 0.0, 0.0, 46, True))


def main():
    """ Parses the provided arguments and triggers simulation process accordingly. """
    parser = argparse.ArgumentParser(description='IQZ Vehicle Simulation')
    parser.add_argument(
        '--recording','-r', 
        action='store_true', 
        help='Boolean switch to trigger the recording')
    parser.add_argument(
        '--playback', '-p',
        default='',
        help='Triggers playback of provided file e.g. recording.rec'
        )
    parser.add_argument(
        '--store-radar-data', '-s',
        action='store_true', 
        help='Boolean switch to trigger storage of the data produced by the RADAR sensor')
    args = parser.parse_args()

    if args.playback:
        replay_recording(args.playback)
    else:
        simulate(args.recording, True)



# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    def __init__(self, parent_actor, hud, gamma_correction):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z
        Attachment = carla.AttachmentType

        if not self._parent.type_id.startswith("walker.pedestrian"):
            self._camera_transforms = [
                (carla.Transform(carla.Location(x=-2.0*bound_x, y=+0.0*bound_y, z=2.0*bound_z), carla.Rotation(pitch=8.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=+0.8*bound_x, y=+0.0*bound_y, z=1.3*bound_z)), Attachment.Rigid),
                (carla.Transform(carla.Location(x=+1.9*bound_x, y=+1.0*bound_y, z=1.2*bound_z)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=-2.8*bound_x, y=+0.0*bound_y, z=4.6*bound_z), carla.Rotation(pitch=6.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=-1.0, y=-1.0*bound_y, z=0.4*bound_z)), Attachment.Rigid)]
        else:
            self._camera_transforms = [
                (carla.Transform(carla.Location(x=-2.5, z=0.0), carla.Rotation(pitch=-8.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=1.6, z=1.7)), Attachment.Rigid),
                (carla.Transform(carla.Location(x=2.5, y=0.5, z=0.0), carla.Rotation(pitch=-8.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=-4.0, z=2.0), carla.Rotation(pitch=6.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=0, y=-2.5, z=-0.0), carla.Rotation(yaw=90.0)), Attachment.Rigid)]

        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
        ]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
            elif item[0].startswith('sensor.lidar'):
                self.lidar_range = 50

                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
                    if attr_name == 'range':
                        self.lidar_range = float(attr_value)

            item.append(bp)
        self.index = None

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False, force_respawn=True)

    def set_sensor(self, index, notify=True, force_respawn=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else \
            (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1])
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / (2.0 * self.lidar_range)
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        elif self.sensors[self.index][0].startswith('sensor.camera.dvs'):
            # Example of converting the raw_data from a carla.DVSEventArray
            # sensor into a NumPy array and using it as an image
            dvs_events = np.frombuffer(image.raw_data, dtype=np.dtype([
                ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', np.bool)]))
            dvs_img = np.zeros((image.height, image.width, 3), dtype=np.uint8)
            # Blue is positive, red is negative
            dvs_img[dvs_events[:]['y'], dvs_events[:]['x'], dvs_events[:]['pol'] * 2] = 255
            self.surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))
        elif self.sensors[self.index][0].startswith('sensor.camera.optical_flow'):
            image = image.get_color_coded_flow()
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)



if __name__ == '__main__':
    main()


