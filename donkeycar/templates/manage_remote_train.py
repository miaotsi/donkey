'''
Usage:
    manage.py --model_path="mymodel.json" [--type="categorical"]


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

from docopt import docopt
from keras.engine import saving
import h5py

import donkeycar as dk
from donkeycar.parts.camera import PiCamera
from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle
from donkeycar.parts.network import MQTTValueSub, MQTTValuePub
from donkeycar.parts.image import ImgArrToJpg
from donkeycar.parts.transform import Lambda

cfg = dk.load_config()

V = dk.Vehicle()
args = docopt(__doc__)
print(args)


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

cam = PiCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH)
V.add(cam, outputs=["camera/arr"], threaded=True)

img_to_jpg = ImgArrToJpg()
V.add(img_to_jpg, inputs=["camera/arr"], outputs=["camera/jpg"])

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

V.add(TelemetryPacker(), inputs=['js/steering', 'js/throttle', 'js/mode', 'js/recording', "camera/jpg"])

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

V.add(ModeChoice(ctr), inputs=['js/mode', 'web/mode'], ouputs=["user/mode"])

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

model_path = args["--model_path"]
kl = dk.utils.get_model_by_type(args["--type"], cfg)
load_model_json(kl, model_path)
weights_path = model_path.replace('.json', '.weights')
load_weights(kl, weights_path)

def is_ai_running(mode):
    return mode != "user"

V.add(Lambda(is_ai_running), inputs=['user/mode'], outputs="ai_running")

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

    def reload_weights(self, client, userdata, message):
        print("got some weights to reload")
        data = message.payload    
        p = zlib.decompress(data)
        obj = pickle.loads(p)
        bin_weights = obj['val']
              
        by_name = False
        skip_mismatch = True
        reshape = False
        bio = io.BytesIO(bin_weights)
        
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

reloader = WeightReloader(kl)

#These are not parts, but single run callbacks from MQTT
sub_controls.client.message_callback_add("donkey/%s/weights" % cfg.DONKEY_UNIQUE_NAME, reloader.reload_weights)
sub_controls.client.subscribe("donkey/%s/weights" % cfg.DONKEY_UNIQUE_NAME, qos=1)

V.start(rate_hz=cfg.DRIVE_LOOP_HZ)
