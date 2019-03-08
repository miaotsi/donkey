'''
Usage:
    manage.py --model="mymodel.json" [--type="categorical"] [--name="myrobot"]


Options:
    -h --help     Show this screen.

file: manage_remote.py
author: Tawn Kramer
date: 2019-01-24
desc: Control a remote donkey robot over network
'''
import time
import zlib
import pickle
import socket
import io
from threading import Lock
import math

from docopt import docopt
from keras.engine import saving
import h5py

import donkeycar as dk
from donkeycar.parts.camera import PiCamera
from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle
from donkeycar.parts.network import MQTTValueSub, MQTTValuePub
from donkeycar.parts.image import ImgArrToJpg
from donkeycar.parts.transform import Lambda
from donkeycar.parts.simulation import MovingSquareTelemetry, SquareBoxCamera
import tensorflow as tf

cfg = dk.load_config()

V = dk.Vehicle()
args = docopt(__doc__)
print(args)

TEST_ON_PC = True

cfg.DONKEY_UNIQUE_NAME = args['--name']

print("starting up", cfg.DONKEY_UNIQUE_NAME, "for remote management.")

class KeepAlive():
    def __init__(self, host, port, name):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.host = host
        self.port = port
        self.name = name

    def run_threaded(self):
        pass

    def update(self):
        while True:
            self.sock.sendto(self.name.encode(), (self.host, self.port))
            time.sleep(1.0)

alive = KeepAlive(host=cfg.MQTT_BROKER, port=8886, name=cfg.DONKEY_UNIQUE_NAME)
V.add(alive, threaded=True)

if TEST_ON_PC:
    V.add(MovingSquareTelemetry(), outputs=['sqr/x', 'sqr/y'])
    cam = SquareBoxCamera()
    V.add(cam, inputs=['sqr/x', 'sqr/y'], outputs=["camera/arr"])
else:
    cam = PiCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH)
    V.add(cam, outputs=["camera/arr"], threaded=True)

img_to_jpg = ImgArrToJpg()
V.add(img_to_jpg, inputs=["camera/arr"], outputs=["camera/jpg"])

if TEST_ON_PC:
    class MockJoystick:
        def __init__(self):
            self.theta = 0.0

        def run(self):
            self.theta += 0.01
            return math.sin(self.theta), 0.3, "user", True

        def is_avail(self):
            return False

    ctr = MockJoystick()
    V.add(ctr, outputs=['js/steering', 'js/throttle', 'js/mode', 'js/recording'])

else:

    from donkeycar.parts.controller import PS3JoystickController

    ctr = PS3JoystickController(throttle_scale=cfg.JOYSTICK_MAX_THROTTLE,
                                    steering_scale=cfg.JOYSTICK_STEERING_SCALE,
                                    auto_record_on_throttle=cfg.AUTO_RECORD_ON_THROTTLE)
            
    V.add(ctr, 
            inputs=['camera/arr'],
            outputs=['js/steering', 'js/throttle', 'js/mode', 'js/recording'],
            threaded=True)

class TelemetryPacker:
    def run(self, steering, throttle, mode, recording, img):
        
        data = { 'steering' : steering,
                'throttle': throttle,
                'mode': mode,
                'recording': recording,
                'img' : img }

        return data

V.add(TelemetryPacker(), inputs=['js/steering', 'js/throttle', 'js/mode', 'js/recording', "camera/jpg"],
    outputs=['packed/telem'])

pub_cam = MQTTValuePub("donkey/%s/telem" % cfg.DONKEY_UNIQUE_NAME, broker=cfg.MQTT_BROKER)
V.add(pub_cam, inputs=["packed/telem"])

sub_controls = MQTTValueSub("donkey/%s/web_controls" % cfg.DONKEY_UNIQUE_NAME, def_value=(0., 0.), broker=cfg.MQTT_BROKER)
V.add(sub_controls, outputs=["packed/data"])

class WebDataUnpacker:
    def run(self, data):
        if not data or type(data) is tuple:
            return 0., 0., 'user', False
        return data['steering'], data['throttle'], data['mode'], data['recording']

V.add(WebDataUnpacker(), inputs=['packed/data'], outputs=['web/steering', 'web/throttle', 'web/mode', 'web/recording'])

class ModeChoice():
    def __init__(self, js_controller):
        self.js_controller = js_controller

    def run(self, js_mode, web_mode):
        if self.js_controller.is_avail():
            return js_mode
        return web_mode

V.add(ModeChoice(ctr), inputs=['js/mode', 'web/mode'], outputs=["user/mode"])

def load_model_json(kl, json_fnm):
    start = time.time()
    print('loading model json', json_fnm)
    import keras
    try:
        with open(json_fnm, 'r') as handle:
            contents = handle.read()
            kl.model = keras.models.model_from_json(contents)
        print('finished loading json in %s sec.' % (str(time.time() - start)) )
    except Exception as e:
        print(e)
        print("ERR>> problems loading model json", json_fnm)

def load_weights(kl, weights_path):
    start = time.time()
    try:
        print('loading model weights', weights_path)
        kl.model.load_weights(weights_path)
        print('finished loading in %s sec.' % (str(time.time() - start)) )
    except Exception as e:
        print(e)
        print('ERR>> problems loading weights', weights_path)

model_path = args["--model"]
kl = dk.utils.get_model_by_type(args["--type"], cfg)
load_model_json(kl, model_path)
weights_path = model_path.replace('.json', '.weights')
load_weights(kl, weights_path)
graph = tf.get_default_graph()

def is_ai_running(mode):
    return mode != "user"

V.add(Lambda(is_ai_running), inputs=['user/mode'], outputs=["ai_running"])

V.add(kl, inputs=['camera/arr'], outputs=["ai/steering", "ai/throttle"], run_condition="ai_running")

class ControlChoice():
    def run(self, js_steer, js_throttle,
        web_steer, web_throttle,
        ai_steer, ai_throttle, mode):

        if mode == "user":
            if js_throttle != 0.0 or js_steer != 0.0:
                return js_steer, js_throttle
            if web_throttle != 0.0 or web_steer != 0.0:
                return web_steer, web_throttle
            return 0.0, 0.0
        elif mode == "local_angle":
            throttle = 0.0
            if js_throttle != 0.0:
                throttle = js_throttle
            elif web_throttle != 0.0:
                throttle = web_throttle
            return ai_steer, throttle
        else:
            return ai_steer, ai_throttle

V.add(ControlChoice(), inputs=['js/steering', 'js/throttle',
    'web/steering', 'web/throttle', "ai/steering", "ai/throttle",
    "user/mode"],
    outputs=['steering', 'throttle'])

if not TEST_ON_PC:
    steering_controller = PCA9685(cfg.STEERING_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
    steering = PWMSteering(controller=steering_controller,
                                    left_pulse=cfg.STEERING_LEFT_PWM, 
                                    right_pulse=cfg.STEERING_RIGHT_PWM)

    throttle_controller = PCA9685(cfg.THROTTLE_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
    throttle = PWMThrottle(controller=throttle_controller,
                                    max_pulse=cfg.THROTTLE_FORWARD_PWM,
                                    zero_pulse=cfg.THROTTLE_STOPPED_PWM, 
                                    min_pulse=cfg.THROTTLE_REVERSE_PWM)

    V.add(steering, inputs=['steering'])
    V.add(throttle, inputs=['throttle'])

class WeightReloader():
    def __init__(self, keras_part):
        self.kl = keras_part
        self.mutex = Lock()

    def reload_model(self, client, userdata, message):
        self.mutex.acquire()
        try:
            import keras
            global graph
            with graph.as_default():
                print("got some a model to reload")
                data = message.payload    
                p = zlib.decompress(data)
                obj = pickle.loads(p)
                json_data = obj['val']        
                self.kl.model = keras.models.model_from_json(json_data)
        except Exception as e:
            print(e)
        finally:
            self.mutex.release()
    
    def reload_weights(self, client, userdata, message):
        self.mutex.acquire()
        try:
            print("got some weights to reload")
            data = message.payload    
            p = zlib.decompress(data)
            obj = pickle.loads(p)
            hdf5_data = obj['val']
                
            by_name = False
            skip_mismatch = True
            reshape = False
            '''
            #from https://stackoverflow.com/questions/11588630/pass-hdf5-file-to-h5py-as-binary-blob-string/45900556#45900556
            import tempfile
            import contextlib

            file_access_property_list = h5py.h5p.create(h5py.h5p.FILE_ACCESS)
            file_access_property_list.set_fapl_core(backing_store=False)
            file_access_property_list.set_file_image(hdf5_data)

            file_id_args = {
                'fapl': file_access_property_list,
                'flags': h5py.h5f.ACC_RDONLY,
                'name': next(tempfile._get_candidate_names()).encode(),
            }

            h5_file_args = {'backing_store': False, 'driver': 'core', 'mode': 'r'}

            with contextlib.closing(h5py.h5f.open(**file_id_args)) as file_id:
                with h5py.File(file_id, **h5_file_args) as f:
                    if 'layer_names' not in f.attrs and 'model_weights' in f:
                        f = f['model_weights']
                    if by_name:
                        saving.load_weights_from_hdf5_group_by_name(
                            f, kl.model.layers, skip_mismatch=skip_mismatch,
                            reshape=reshape)
                    else:
                        saving.load_weights_from_hdf5_group(
                            f, kl.model.layers, reshape=reshape)
            '''
            '''
            #if h5py verion >= 2.9
            bio = io.BytesIO(hdf5_data)
            with h5py.File(bio) as f:
                if 'layer_names' not in f.attrs and 'model_weights' in f:
                        f = f['model_weights']
                    if by_name:
                        saving.load_weights_from_hdf5_group_by_name(
                            f, kl.model.layers, skip_mismatch=skip_mismatch,
                            reshape=reshape)
                    else:
                        saving.load_weights_from_hdf5_group(
                            f, kl.model.layers, reshape=reshape)
            '''
            #yuck, write out the weights to a temp file and load them from disk. gagg...
            global graph
            with graph.as_default():
                temp_filename = "temp.weights"
                outfile = open(temp_filename, "wb")
                outfile.write(hdf5_data)
                outfile.close()
                with h5py.File(temp_filename, mode='r') as f:
                    if 'layer_names' not in f.attrs and 'model_weights' in f:
                        f = f['model_weights']
                    if by_name:
                        saving.load_weights_from_hdf5_group_by_name(
                            f, kl.model.layers, skip_mismatch=skip_mismatch,
                            reshape=reshape)
                    else:
                        saving.load_weights_from_hdf5_group(
                            f, kl.model.layers, reshape=reshape)
        except Exception as e:
            print(e)
        finally:
            self.mutex.release()
    

reloader = WeightReloader(kl)

#test file reloader
class TestMessage():
    def __init__(self, name, values):
        packet = { "name": name, "val" : values }
        p = pickle.dumps(packet)
        z = zlib.compress(p)
        self.payload = z

file = open(weights_path, 'rb')
reloader.reload_weights( None, None, TestMessage("model/weights", file.read()))
file.close()
file = open(model_path, 'rb')
reloader.reload_model( None, None, TestMessage("model/model", file.read()))
file.close()

#These are not parts, but single run callbacks from MQTT
sub_controls.client.message_callback_add("donkey/%s/model" % cfg.DONKEY_UNIQUE_NAME, reloader.reload_model)
sub_controls.client.subscribe("donkey/%s/model" % cfg.DONKEY_UNIQUE_NAME, qos=1)
sub_controls.client.message_callback_add("donkey/%s/weights" % cfg.DONKEY_UNIQUE_NAME, reloader.reload_weights)
sub_controls.client.subscribe("donkey/%s/weights" % cfg.DONKEY_UNIQUE_NAME, qos=1)

V.start(rate_hz=cfg.DRIVE_LOOP_HZ)
