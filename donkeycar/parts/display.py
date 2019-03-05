import time

import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

import subprocess

# Raspberry Pi pin configuration:
RST = None     # on the PiOLED this pin isnt used

class OLED_SD1306():
    '''
    This is a display that can be found on Adafruit's site in a few resolutions.
    https://www.adafruit.com/?q=SD1306
    Install the library dependencies with
    pip install Adafruit_SSD1306
    This assumes I2C connection at default address 0x3C
    See their github for more information on different connection types
    and the proper method for intialization.
    https://github.com/adafruit/Adafruit_Python_SSD1306
    '''
    def __init__(self, disp=None):
        # default to the 128x64 monochrome display, if none provided.
        self.disp = disp or Adafruit_SSD1306.SSD1306_128_64(rst=RST)
        
        # Initialize library.
        self.disp.begin()
        self.clear()

    def clear(self):
        # Clear display.
        self.disp.clear()
        self.disp.display()

    def run(self, image):
        '''
        this expects a one bit image of the correct dimensions:

        from PIL import Image
        image = Image.new('1', (disp.width, disp.height))

        '''
        # Display image.
        self.disp.image(image)
        self.disp.display()

    def shutdown(self):
        self.clear()


class MonochromeImage():
    '''
    this makes monochrome image suitable for display on the above OLED device
    '''
    def __init__(self, width=128, height=64):
        from PIL import Image
        self.image = Image.new('1', (width, height))

    def run(self):
        return self.image



    