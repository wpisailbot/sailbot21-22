# *****************************************************************************
# * | File        :	  epd2in9b_V3.py (changed to epd_library.py)
# * | Author      :   Waveshare team
# * | Function    :   Electronic paper driver
# * | Info        :
# *----------------
# * | This version:   V1.1
# * | Date        :   2020-12-03
# # | Info        :   python demo
# -----------------------------------------------------------------------------
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
# *****************************************************************************



# *****************************************************************************
#
# Fitted for use with WPI's Sailbot MQP 2021-2022
# Contributors:     Tony Tesoriero
# Last Edited:      Jan. 8, 2022
#
# Resources:
# Wiki for our e-Paper model (NOTE: VERSION 3)
# https://www.waveshare.com/wiki/2.9inch_e-Paper_Module_(B)
#      User Guides of Jetson Nano Tab has quickstart guide and pinout
#      Resources tab has useful docs
#      NOTE: MAKE SURE TO FOLLOW INSTALL INSTRUCTIONS ON NEW SETUP
#
# Data sheet with SPI command table and timing
# https://www.waveshare.com/w/upload/a/af/2.9inch-e-paper-b-v3-specification.pdf
#
# Github with examples and this library
# https://github.com/waveshare/e-Paper
# 
# PINOUT:
    # RST_PIN  -->   15   any digital pin
    # VCC      -->   17   any 3.3V
    # BUSY_PIN -->   18   any digital pin
    # DIN      -->   19   don't change (SPI1_MOSI)
    # GND      -->   20   any GND
    # DC_PIN   -->   22   any digital pin
    # CLK      -->   23   don't change (SPI1_SCK)
    # CS_PIN   -->   24   don't change (SPI1_CS0)
#
# *****************************************************************************

import os
import time
import ctypes
import Jetson.GPIO

# Display resolution
EPD_WIDTH       = 128
EPD_HEIGHT      = 296

class EPaperDisplay():

    """
    This is a class serves as a library to control our e-Paper Display 
      
    Attributes:
        RST_PIN  (int): The pin # connect to the EPD reset pin
        BUSY_PIN (int): The pin # connect to the EPD busy pin
        DC_PIN   (int): The pin # connect to the EPD dc pin (dictates command or data)
        CS_PIN   (int): The pin # connect to the EPD cs pin (dictates command or data)
        SPI      (obj): The object of Waveshare's custom SPI lib
        GPIO     (obj): The object of the Jetson.GPIO lib
        logger   (obj): The ROS2 rclpy logger imported from node (remove to use lib oustode of ROS)
        width    (int): The pixel width of the EPD (128, short side)
        height   (int): The pixel height of the EPD (298, long side)
    """

    ############################
    ###    Init Functions    ###
    ############################

    def __init__(self, logger):

        # Declare all GPIO pin #'s
        self.RST_PIN    = 15 #  any digital pin
        # VCC     -->     17    any 3.3V
        self.BUSY_PIN   = 18 #  any digital pin
        # DIN     -->     19    Don't change (SPI1_MOSI)
        # GND     -->     20    any GND
        self.DC_PIN     = 22 #  any digital pin
        # CLK     -->     23    Don't change (SPI1_SCK)
        self.CS_PIN     = 24 #  Don't change (SPI1_CS0)
        self.GPIO = Jetson.GPIO

        # Import logger from top node and set if logging starts on
        self.logger = logger
        self.__logLevelSelf = 1 # int (0 none, 1 display, 2 busy, 3 ok Calls, 4 all calls)

        # Set pixel dimentions
        self.width = EPD_WIDTH
        self.height = EPD_HEIGHT

        # TODO: SWITCH TO NEW SPI LIBRARY!! Check func module_exit for reason
        # replacements: python-spidev, or CircutPython board lib
        # CircutPython example: https://bit.ly/3FXT92a
        self.SPI = None
        find_dir = os.path.dirname(os.path.realpath(__file__))
        so_filename = os.path.join(find_dir, 'sysfs_software_spi.so')
        self.SPI = ctypes.cdll.LoadLibrary(so_filename)
        if self.SPI is None:
            raise RuntimeError('Cannot find sysfs_software_spi.so')

    
    def module_init(self):
        """
        This method initializes the e-Paper, after the EPD is ready to print

        """
        # Log call
        self.log('E-Paper Powerup', 3)

        # Setup all GPIO pins
        self.GPIO.setmode(self.GPIO.BOARD)  # BOARD:    Printed pin numbers
                                            # BCM:      https://bit.ly/3F8oFtG
        self.GPIO.setwarnings(False)
        self.GPIO.setup(self.RST_PIN, self.GPIO.OUT)
        self.GPIO.setup(self.DC_PIN, self.GPIO.OUT)
        self.GPIO.setup(self.CS_PIN, self.GPIO.OUT)
        self.GPIO.setup(self.BUSY_PIN, self.GPIO.IN)

        # Start SPI communication
        self.SPI.SYSFS_software_spi_begin() # TODO: SWITCH TO NEW SPI LIBRARY!!

        # EPD hardware reset (wakes from deep sleep)
        self.reset()
                                                # NOTE: command codes defined at end of file
        self.send_command(POWER_ON)             # power on
        self.readBusy()                         # waiting for the electronic paper IC to release the idle signal

        self.send_command(PANEL_SETTING)        # panel setting
        self.send_data(0x0f)                    # LUT from OTP,128x296
        self.send_data(0x89)                    # Temperature sensor, boost and other related timing settings

        self.send_command(RESOLUTION_SETTING)   # resolution setting
        self.send_data (0x80)                   # default
        self.send_data (0x01)                   # default
        self.send_data (0x28)                   # default

        self.send_command(VCOM_AND_DATA_INTERVAL_SETTING)     # WBRmode:VBDF F7 VBDW 77 VBDB 37  VBDR B7
        self.send_data(0x77)                                  # WBmode:VBDF 17|D7 VBDW 97 VBDB 57

        self.send_command(PLL_CONTROL)                        # Sets EPD operation frequency, can be removed 
        self.send_data(0x3F)                                  # NOTE: No effect to refresh time, can be removed 
        return 0

    def reset(self):
        """
        This method resets the EPD hardware and awakes if in deep sleep 

        """
        self.log('RESET', 3)
        self.GPIO.output(self.RST_PIN, 1)
        self.delay_ms(200)
        self.GPIO.output(self.RST_PIN, 0)
        self.delay_ms(2)
        self.GPIO.output(self.RST_PIN, 1)
        self.delay_ms(200)
    


    ###############################
    ###    Logging Functions    ###
    ###############################

    def setLog(self, loggerLevel):
        """
        This method sets the logging level local to this library
        0 => Log Off | 1 => Display | 2 => Busy | 3 => Some | 4 => Most

        Parameters:
            turnOn (int): integer for desired local logging level
        """
        self.__logLevelSelf = loggerLevel

    def log(self, string, logLevel):
        """
        This helper method prints logs for this library formatted correctly

        Parameters:
            string   (str): string to log
            logLevel (int): integer of lowest log level to prompt this log
        """
        levelStr = 'OFF', 'Disp', 'Busy', 'Some', 'Most'
        header = '[EPD Lib] (' + levelStr[logLevel] + '):    '
        (self.__logLevelSelf > logLevel-1) and self.logger.debug(header + string)



    ###############################
    ###    Display Functions    ###
    ###############################

    def getbuffer(self, image):
        """
        This method creates a buffer byte list formatted for display() method

        Parameters:
            image (byte): a PIL Image object

        Returns:
            buf (list): The list of bytes ready to display to screen

        """
        
        buf = [0xFF] * (int(self.width/8) * self.height)
        image_monocolor = image.convert('1')
        imwidth, imheight = image_monocolor.size
        pixels = image_monocolor.load()

        self.log('Buffer size = ' + str(len(buf)), 3)
        self.log('Image width = ' + str(imwidth) + ', height = ' + str(imheight), 3)

        if(imwidth == self.width and imheight == self.height):
            self.log('Image is Vertical', 4)
            for y in range(imheight):
                for x in range(imwidth):
                    # Set the bits for the column of pixels at the current position.
                    if pixels[x, y] == 0:
                        buf[int((x + y * self.width) / 8)] &= ~(0x80 >> (x % 8))

        elif(imwidth == self.height and imheight == self.width):
            self.log('Image is Horizontal', 4)
            for y in range(imheight):
                for x in range(imwidth):
                    newx = y
                    newy = self.height - x - 1
                    if pixels[x, y] == 0:
                        buf[int((newx + newy*self.width) / 8)] &= ~(0x80 >> (y % 8))
        return buf


    def display(self, blackimage, redimage):                       
        """
        This method prints the red and black images to the e-Paper

        Parameters:
            blackimage (list): The byte list buffer returned from getbuffer()
            redimage   (list): The byte list buffer returned from getbuffer()

        """

        self.log("Print start", 1)

        if (blackimage != None):
            self.send_command(DATA_START_TRANSMISSION_1)           # write black
            for i in range(0, int(self.width * self.height / 8)):
                self.send_data(blackimage[i])

        if (redimage != None):
            self.send_command(DATA_START_TRANSMISSION_2)           # write red
            for i in range(0, int(self.width * self.height / 8)):
                self.send_data(redimage[i])

        self.send_command(DISPLAY_REFRESH)                         # display refresh
        self.readBusy()

        self.log('Print done', 1)


    def clear(self): 
        """
        This method clears the screen by displaying all white
        NOTE: only needs to be used after a few screen updates

        """
        self.log('Clear', 3)

        white = [0xff] * int(self.width * self.height / 8)
        self.display(white, white)



    ###########################
    ###    SPI Functions    ###
    ###########################

    def spi_writebyte(self, data):
        """
        This method sends a byte of data over SPI bus

        TODO: SWITCH TO NEW SPI LIBRARY!! Check func module_exit for reason
        replacements: python-spidev, or CircutPython board lib
        CircutPython example: https://bit.ly/3FXT92a 
        """
        self.SPI.SYSFS_software_spi_transfer(data) # TODO: SWITCH TO NEW SPI LIBRARY!!

    def send_command(self, command):
        """
        This method sends a command to the EPD. DC high and CS low signifies command.

        Command codes can be found in variables at end of file or 
        https://www.waveshare.com/w/upload/a/af/2.9inch-e-paper-b-v3-specification.pdf

        Parameters:
            command (byte): The byte command to be sent to the EPD

        """
        self.log('Command Sent: ' + str(command), 4)
        self.GPIO.output(self.DC_PIN, 0)
        self.GPIO.output(self.CS_PIN, 0)
        self.spi_writebyte(command)
        self.GPIO.output(self.CS_PIN, 1)

    def send_data(self, data):
        """
        This method sends data to the EPD. DC low and CS high signifies command.

        NOTE: Always preceeded by send_command() call.
        \/ Proper command data format can be found in datasheet \/ 
        https://www.waveshare.com/w/upload/a/af/2.9inch-e-paper-b-v3-specification.pdf

        Parameters:
            data (byte): The byte of data to be sent to the EPD

        """
        self.log('Command Sent: ' + str(data), 4)
        self.GPIO.output(self.DC_PIN, 1)
        self.GPIO.output(self.CS_PIN, 0)
        self.spi_writebyte(data)
        self.GPIO.output(self.CS_PIN, 1)

    def readBusy(self):
        """
        This method haults until EPD is not busy. EPD goes busy after every command.

        """
        self.log("e-Paper busy", 2)

        self.send_command(GET_STATUS)                 #  get status (update busy pin)
        while(self.GPIO.input(self.BUSY_PIN) == 0):   #  0: idle, 1: busy
            self.send_command(GET_STATUS)             #  get status (update busy pin)

        self.log('e-Paper busy release', 2)

    def delay_ms(self, delaytime):
        """
        This method haults program using time lib

        Parameters:
            delaytime (int): amount of time to hault in milliseconds      
        """
        self.log('Delay ' + str(delaytime) + 'ms', 3)
        time.sleep(delaytime / 1000.0)



    ################################
    ###    Shutdown Functions    ###
    ################################

    def module_exit(self):
        """
        This method exits the EPD, freeing any used GPIO and SPI pins and 
        FIXME: ending the SPI prevents SPI from starting again without reboot (see TODO/FIXME markers for info)

        """

        self.log('E-Paper Powerdown', 3)
        # self.SPI.SYSFS_software_spi_end()             # FIXME: ending custom SPI library locks 
                                                        # FIXME: ^SPI and needs reboot to re-enable
                                                        # FIXME: SWITCH TO NEW SPI LIBRARY!!
        self.GPIO.output(self.RST_PIN, 0)
        self.GPIO.output(self.DC_PIN, 0)

        self.GPIO.cleanup([self.RST_PIN, self.DC_PIN, self.CS_PIN, self.BUSY_PIN])

    def sleep(self):
        """
        This method turns e-Paper off,sends it to deep sleep, ends SPI, and frees GPIO
        NOTE: call to reset() needed to wake

        """
        self.log('Sleep', 3)
        self.send_command(POWER_OFF)    # power off
        self.readBusy()
        self.send_command(DEEP_SLEEP)   # deep sleep
        self.send_data(0xA5)            # confirmation packet
        self.module_exit()              # clear up GPIO


#####################################
##### E-Paper SPI Command Codes #####
#####################################
PANEL_SETTING                  = 0x00
POWER_SETTING                  = 0x01
POWER_OFF                      = 0x02
POWER_OFF_SEQUENCE_SETTING     = 0x03
POWER_ON                       = 0x04
POWER_ON_MEASURE               = 0x05
BOOSTER_SOFT_START             = 0x06
DEEP_SLEEP                     = 0x07
DATA_START_TRANSMISSION_1      = 0x10
DATA_STOP                      = 0x11
DISPLAY_REFRESH                = 0x12
DATA_START_TRANSMISSION_2      = 0x13

AUTO_SEQUENCE = 0x17

LUT_FOR_VCOM                   = 0x20
LUT_WHITE_TO_WHITE             = 0x21
LUT_BLACK_TO_WHITE             = 0x22
LUT_WHITE_TO_BLACK             = 0x23
LUT_BLACK_TO_BLACK             = 0x24

LUT_OPTION = 0x2A

PLL_CONTROL                    = 0x30
TEMPERATURE_SENSOR_COMMAND     = 0x40
TEMPERATURE_SENSOR_SELECTION   = 0x41
TEMPERATURE_SENSOR_WRITE       = 0x42
TEMPERATURE_SENSOR_READ        = 0x43

PANEL_BREAK_CHECK = 0x44

VCOM_AND_DATA_INTERVAL_SETTING = 0x50
LOW_POWER_DETECTION            = 0x51
TCON_SETTING                   = 0x60
RESOLUTION_SETTING             = 0x61
GSST_SETTING                   = 0x65

REV = 0x70

GET_STATUS                     = 0x71
AUTO_MEASUREMENT_VCOM          = 0x80
READ_VCOM_VALUE                = 0x81
VCOM_DC_SETTING                = 0x82
PARTIAL_WINDOW                 = 0x90
PARTIAL_IN                     = 0x91
PARTIAL_OUT                    = 0x92
PROGRAM_MODE                   = 0xA0
ACTIVE_PROGRAMMING             = 0xA1
READ_OTP                       = 0xA2

CASCADE_SETTING = 0xE0
POWER_SAVING = 0xE3
LVD_VOLTAGE_SELECT = 0xE4
FORCE_TEMP = 0xE5
