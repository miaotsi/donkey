"""
Scripts launch training services as donkeys show up

Usage:
    user_service.py --broker="localhost"


Options:
    -h --help     Show this screen.
"""
import os
import time
import math
import zlib
import pickle
from subprocess import Popen
import shlex

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

args = docopt(__doc__)
print(args)

class Trainer():
    def __init__(self, name, port):
        self.name = name
        os.system('mkdir data/%s' % name)
        command = 'python train_service.py --broker=localhost --name=%s --record=data/%s --port=%d' % (name, name, port)
        print('command:', command)
        args =  shlex.split(command)
        self.proc = Popen(args)
        self.t = time.time()

    def age(self):
        e = time.time() - self.t
        #print(e)
        return e > 3.0 

    def run(self):
        print("updating trainer", self.name)
        self.t = time.time()

    def stop(self):
        if self.proc is not None:
            print("killing trainer", self.name)
            self.proc.terminate()
            self.proc = None

    def __del__(self):
        self.stop()


trainers = {}

alive = MQTTValueSub(name="donkey/alive", broker=args["--broker"])

print("user service listening for donkey/alive messages...")

while True:
    '''
    listen for alive messages
    '''
    alive_donkey = alive.run()

    '''
    delete old donkey trainers we haven't heard from in a while
    '''
    for key, trainer in trainers.items():
        if trainer.age():
            trainer.stop()
            del trainers[key]
            break

    if alive_donkey is None:
        continue
    
    '''
    spawn new trainer
    '''
    if not alive_donkey in trainers:
        num_donkeys = len(trainers)
        port = 8887 + num_donkeys
        trainers[alive_donkey] = Trainer(alive_donkey, port)

    '''
    update alive trainer
    '''
    trainers[alive_donkey].run()

    '''
    clear last message
    '''
    alive.reset_message()

    time.sleep(1)

