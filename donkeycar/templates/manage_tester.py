'''
file: manage_remote.py
author: Tawn Kramer
date: 2019-01-24
desc: Control a remote donkey robot over network
'''
import time
import zlib, pickle
import sys

import donkeycar as dk
from donkeycar.parts.cv import CvCam, ImgBGR2RGB
from donkeycar.parts.actuator import PWMSteering, PWMThrottle
from donkeycar.parts.network import MQTTValueSub, MQTTValuePub
from donkeycar.parts.image import ImgArrToJpg
from donkeycar.parts.transform import Lambda
from donkeycar.parts.simulation import MovingSquareTelemetry, SquareBoxCamera
import socket
from keras.engine import saving
import h5py

class MockPCA9685():
    def __init__(self, a, b, busnum):
        pass

    def set_pulse(self, val):
        pass

cfg = dk.load_config()

V = dk.Vehicle()

cfg.DONKEY_UNIQUE_NAME = sys.argv[1]

print("starting up", cfg.DONKEY_UNIQUE_NAME, "for remote management.")

#cam = CvCam(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH)
V.add(MovingSquareTelemetry(), outputs=['sqr/x', 'sqr/y'])
cam = SquareBoxCamera()
V.add(cam, inputs=['sqr/x', 'sqr/y'], outputs=["camera/arr"])
V.add(ImgBGR2RGB(), inputs=["camera/arr"], outputs=["camera/arr"])


img_to_jpg = ImgArrToJpg()
V.add(img_to_jpg, inputs=["camera/arr"], outputs=["camera/jpg"])

class MockJoystick:
    def run(self):
        return 0.0, 0.0, "user", False

V.add(MockJoystick(), outputs=['js/steering', 'js/throttle', 'js/mode', 'js/recording'])

class TelemetryPacker:
    def run(self, steering, throttle, mode, recording, img):
        
        data = { 'steering' : steering,
                'throttle': throttle,
                'mode': mode,
                'recording': recording,
                'img' : img }

        return data

V.add(TelemetryPacker(), inputs=['js/steering', 'js/throttle', 'js/mode', 'js/recording', "camera/jpg"],
    outputs=["packed/telem"])

pub_cam = MQTTValuePub("donkey/%s/telem" % cfg.DONKEY_UNIQUE_NAME, broker=cfg.MQTT_BROKER)
V.add(pub_cam, inputs=["packed/telem"])

class KeepAlive():
    def __init__(self, host, port, name):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.host = host
        self.port = port
        self.name = name

    def run(self):
        self.sock.sendto(self.name.encode(), (self.host, self.port))

alive = KeepAlive(host=cfg.MQTT_BROKER, port=8886, name=cfg.DONKEY_UNIQUE_NAME) #cfg.MQTT_BROKER)
V.add(alive)

sub_controls = MQTTValueSub("donkey/%s/controls" % cfg.DONKEY_UNIQUE_NAME, def_value=(0., 0.), broker=cfg.MQTT_BROKER)
V.add(sub_controls, outputs=["packed/data"])

class DonkeyDataUnpacker:
    def run(self, data):
        if not data or type(data) is tuple:
            return 0., 0., 'user', False
        return data['steering'], data['throttle'], data['mode'], data['recording']

V.add(DonkeyDataUnpacker(), inputs=['packed/data'], outputs=['angle', 'throttle', 'mode', 'recording'])


steering_controller = MockPCA9685(cfg.STEERING_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
steering = PWMSteering(controller=steering_controller,
                                left_pulse=cfg.STEERING_LEFT_PWM, 
                                right_pulse=cfg.STEERING_RIGHT_PWM)

throttle_controller = MockPCA9685(cfg.THROTTLE_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
throttle = PWMThrottle(controller=throttle_controller,
                                max_pulse=cfg.THROTTLE_FORWARD_PWM,
                                zero_pulse=cfg.THROTTLE_STOPPED_PWM, 
                                min_pulse=cfg.THROTTLE_REVERSE_PWM)

V.add(steering, inputs=['angle'])
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
