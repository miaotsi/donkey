"""
Scripts to train a donkey and control remotely

Usage:
    train_service.py --name=<robot_name> --broker="localhost" --port=8887 [--record=<path>]


Options:
    -h --help     Show this screen.
"""
import os
import time
import math
import zlib
import pickle

from docopt import docopt
import donkeycar as dk
import cv2

from donkeycar.parts.cv import CvImageView, ImgBGR2RGB, ImgRGB2BGR, ImageScale, ImgWriter, ArrowKeyboardControls
from donkeycar.parts.network import MQTTValuePub, MQTTValueSub
from donkeycar.parts.transform import Lambda
from donkeycar.parts.image import JpgToImgArr
from donkeycar.parts.controller import LocalWebController
from donkeycar.utils import img_to_binary
from donkeycar.parts.file_watcher import FileWatcher
from donkeycar.parts.datastore import TubHandler, TubGroup
from donkeycar.parts.transform import Lambda, TriggeredCallback, DelayedTrigger
from trainer import Trainer

cfg = dk.load_config()

V = dk.vehicle.Vehicle()
args = docopt(__doc__)
print(args)

from PIL import Image
img = Image.open('donkey.jpg')
bin_jpg = img_to_binary(img)

V.add(MQTTValueSub(name="donkey/%s/telem" % args["--name"], broker=args["--broker"], outputs=["telem"])

class TelemetryUnpacker:
    def run(self, data):
        if not data:
            return 0., 0., 'user', False, bin_jpg
        return data['steering'], data['throttle'], data['mode'], data['recording'], data['img']

V.add(TelemetryUnpacker(), inputs=['telem'], 
    outputs=['js/steering', 'js/throttle', 'js/mode', 'js/recording', "jpg"])

V.add(JpgToImgArr(), inputs=["jpg"], outputs=["cam/image_array"])
web = LocalWebController(port=args["--port"])
V.add(web,
          inputs=['cam/image_array'],
          outputs=['web/steering', 'web/throttle', 'web/mode', 'web/recording'],
          threaded=True)


class DonkeyDataPacker:
    def run(self, steering, throttle, mode, recording):
        data = { 'steering' : steering, 'throttle': throttle, 'mode':mode, 'recording':recording }
        return data

V.add(DonkeyDataPacker(), inputs=['web/steering', 'web/throttle', 'web/mode', 'web/recording'], outputs=['packed/data'])
pub = MQTTValuePub(name="donkey/%s/web_controls" % args["--name"], broker=args["--broker"])
V.add(pub, inputs=['packed/data'])

class RecordingChoice():
    def run(self, js_recording, web_recording):
        return js_recording or web_recording

V.add(RecordingChoice(), inputs=['js/recording', 'web/recording'], outputs=['recording']) 

class ControlChoice():
    def run(self, js_steer, js_throttle, web_steer, web_throttle):
        if js_throttle != 0.0 or js_steer != 0.0:
            return js_steer, js_throttle
        if web_throttle != 0.0 or web_steer != 0.0:
            return web_steer, web_throttle
        return 0.0, 0.0

V.add(ControlChoice(), inputs=['js/steering', 'js/throttle', 'web/steering', 'web/throttle'],
    outputs=['user/angle', 'user/throttle'])

record_path = args["--record"]
if record_path is None:
    record_path = "data"

inputs=['cam/image_array',
            'user/angle', 'user/throttle', 
            'web/mode']

types=['image_array',
        'float', 'float',
        'str']

th = TubHandler(path=record_path)
tub = th.new_tub_writer(inputs=inputs, types=types)
V.add(tub, inputs=inputs, outputs=["tub/num_records"], run_condition='recording')

class RunTrainerTest():
    def run(self, num_records):
        if num_records is None:
            return False
        return num_records > 200

'''
model trainer, reloader, publisher
'''
model_path = 'data/%s/mymodel.h5' % args["--name"]
data_path = tub.path
V.add(RunTrainerTest(), inputs=['tub/num_records'], outputs=['do_train'])
trainer = Trainer(cfg=cfg, dirs=data_path, model=model_path, transfer=None, model_type='categorical', continuous=True, aug=False)
V.add(trainer, threaded=True, run_condition='do_train')
weights_file = model_path.replace('.h5', '.weights')
V.add(FileWatcher(weights_file, verbose=True), outputs=['modelfile/dirty'])
V.add(DelayedTrigger(100), inputs=['modelfile/dirty'], outputs=['modelfile/reload'])

class FileSender():
    def __init__(self, filename, client, name):
        self.filename = filename
        self.client = client
        self.name = name

    def run(self, do_load):
        if not do_load:
            return

        try:
            file = open(self.filename, 'rb')
            buffer = file.read()
            file.close()

            packet = { "name": self.name, "val" : buffer }
            p = pickle.dumps(packet)
            z = zlib.compress(p)
            self.client.publish(self.name, z)
        except Exception as e:
            print(e)

loader = FileSender(filename=weights_file, client=pub.client, name="donkey/%s/weights" % args["--name"])
V.add(loader, inputs=['modelfile/reload'])

V.start(rate_hz=20)

