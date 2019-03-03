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

class DonkeyDataPacker:
    def run(self, steering, throttle, mode, recording):
        data = { 'steering' : steering, 'throttle': throttle, 'mode':mode, 'recording':recording }
        return data

class DonkeyDataUnpacker:
    def run(self, data):
        if not data:
            return 0., 0., 'user', False
        return data['steering'], data['throttle'], data['mode'], data['recording']

from PIL import Image
img = Image.open('donkey.jpg')
V.add(MQTTValueSub(name="donkey/%s/camera" % args["--name"], broker=args["--broker"], def_value=img_to_binary(img)), outputs=["jpg"])
V.add(JpgToImgArr(), inputs=["jpg"], outputs=["cam/image_array"]) 
web = LocalWebController(port=args["--port"])
V.add(web,
          inputs=['cam/image_array'],
          outputs=['user/angle', 'user/throttle', 'user/mode', 'recording'],
          threaded=True)

#V.add(DonkeyDataUnpacker(), inputs=['packed/data'], outputs=['steering', 'throttle', 'mode', 'recording'])
V.add(DonkeyDataPacker(), inputs=['user/angle', 'user/throttle', 'user/mode', 'recording'], outputs=['packed/data'])
pub = MQTTValuePub(name="donkey/%s/controls" % args["--name"], broker=args["--broker"])
V.add(pub, inputs=['packed/data'])

record_path = args["--record"]
if record_path is None:
    record_path = "data"

inputs=['cam/image_array',
            'user/angle', 'user/throttle', 
            'user/mode']

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
model_path = 'mymodel.h5'
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

