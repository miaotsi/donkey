'''
file: manage_remote.py
author: Tawn Kramer
date: 2019-01-24
desc: Control a remote donkey robot over network
'''
import time
import zlib, pickle

import donkeycar as dk
from donkeycar.parts.camera import PiCamera
from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle
from donkeycar.parts.network import MQTTValueSub, MQTTValuePub
from donkeycar.parts.image import ImgArrToJpg


cfg = dk.load_config()

V = dk.Vehicle()

print("starting up", cfg.DONKEY_UNIQUE_NAME, "for remote management.")

cam = PiCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH)
V.add(cam, outputs=["camera/arr"], threaded=True)


img_to_jpg = ImgArrToJpg()
V.add(img_to_jpg, inputs=["camera/arr"], outputs=["camera/jpg"])

pub_cam = MQTTValuePub("donkey/%s/camera" % cfg.DONKEY_UNIQUE_NAME, broker=cfg.MQTT_BROKER)
V.add(pub_cam, inputs=["camera/jpg"])

sub_controls = MQTTValueSub("donkey/%s/controls" % cfg.DONKEY_UNIQUE_NAME, def_value=(0., 0.), broker=cfg.MQTT_BROKER)
V.add(sub_controls, outputs=["packed/data"])

class DonkeyDataUnpacker:
    def run(self, data):
        if not data or type(data) is tuple:
            return 0., 0., 'user', False
        return data['steering'], data['throttle'], data['mode'], data['recording']

V.add(DonkeyDataUnpacker(), inputs=['packed/data'], outputs=['angle', 'throttle', 'mode', 'recording'])

steering_controller = PCA9685(cfg.STEERING_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
steering = PWMSteering(controller=steering_controller,
                                left_pulse=cfg.STEERING_LEFT_PWM, 
                                right_pulse=cfg.STEERING_RIGHT_PWM)

throttle_controller = PCA9685(cfg.THROTTLE_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
throttle = PWMThrottle(controller=throttle_controller,
                                max_pulse=cfg.THROTTLE_FORWARD_PWM,
                                zero_pulse=cfg.THROTTLE_STOPPED_PWM, 
                                min_pulse=cfg.THROTTLE_REVERSE_PWM)

V.add(steering, inputs=['angle'])
V.add(throttle, inputs=['throttle'])

def reload_weights(client, userdata, message):
    print("got some weights to reload")
    data = message.payload
    print('got %d bytes' % len(data))
    p = zlib.decompress(data)
    obj = pickle.loads(p)
    weights = obj['val']
    print("weights are %d bytes" % len(weights))


sub_controls.client.message_callback_add("donkey/%s/weights" % cfg.DONKEY_UNIQUE_NAME, reload_weights)
sub_controls.client.subscribe("donkey/%s/weights" % cfg.DONKEY_UNIQUE_NAME, qos=1)

V.start(rate_hz=cfg.DRIVE_LOOP_HZ)
