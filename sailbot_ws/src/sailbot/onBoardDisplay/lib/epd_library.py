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
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furnished to do so, subject to the following conditions:
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
    NOTE: e-Paper works by saving and image to black and red SRAM 
            then when commanded will display from its SRAM
      
    Attributes:
        RST_PIN         (int): The pin # connect to the EPD reset pin
        BUSY_PIN        (int): The pin # connect to the EPD busy pin
        DC_PIN          (int): The pin # connect to the EPD dc pin (dictates command or data)
        CS_PIN          (int): The pin # connect to the EPD cs pin (dictates command or data)
        width           (int): The pixel width of the EPD (128, short side)
        height          (int): The pixel height of the EPD (298, long side)
        __SPI           (obj): The object of Waveshare's custom SPI lib
        __GPIO          (obj): The object of the Jetson.GPIO lib
        __logger        (obj): The ROS2 rclpy logger imported from node (remove to use lib oustode of ROS)
        __logLevelSelf  (int): The local log level for the EPaperDisplay obj, see __log()
        __logExclusive 
    """


    ############################
    ###    Init Functions    ###
    ############################

    def __init__(self, logger=None, logLevel=0, logLevelExclusive=False):
        """
        Declares all class attributes

        Parameters:
            logger              (obj):  The ROS2 rclpy logger imported from node (remove to use lib oustode of ROS)
            logLevel            (int):  Level the logger will print for
            logLevelExclusive   (bool): If logger should print logs for specified level only, default 
        """
                                # Declare all GPIO pin #'s
        self.RST_PIN   = 15     # any digital pin
        # VCC     -->    17       any 3.3V
        self.BUSY_PIN  = 18     # any digital pin
        # DIN     -->    19       Don't change (SPI1_MOSI)
        # GND     -->    20       any GND
        self.DC_PIN    = 22     # any digital pin
        # CLK     -->    23       Don't change (SPI1_SCK)
        self.CS_PIN    = 24     # Don't change (SPI1_CS0)
        self.__GPIO = Jetson.GPIO
       
        self.__logger = logger  # Import logger from top node 
        self.setLog(logLevel, logLevelExclusive)  # see setLog()
        self.__log('EPD Lib declaired, use module_init() to begin', 1)

        self.width = EPD_WIDTH  # Set EPD constants
        self.height = EPD_HEIGHT
        self.white_buf = (0xff,) * int(self.width * self.height / 8)

        # TODO: SWITCH TO NEW SPI LIBRARY!! Check func module_exit for reason
        # replacements: python-spidev, or CircutPython board lib
        # CircutPython example: https://bit.ly/3FXT92a
        self.__SPI = None
        find_dir = os.path.dirname(os.path.realpath(__file__))
        so_filename = os.path.join(find_dir, 'sysfs_software_spi.so')
        self.__SPI = ctypes.cdll.LoadLibrary(so_filename)
        if self.__SPI is None:
            raise RuntimeError('Cannot find sysfs_software_spi.so')


    def module_init(self):
        """
        Turns e-Paper on with initializing settings

        """
        if (self.__setupComms() != 0):
            return -1

        self.__log('e-Paper power up', 1)
        self.__reset()                          # EPD hardware reset (wakes from deep sleep)
        self.__send_command(POWER_ON)           # power on, NOTE: command codes defined at end of file
        self.readBusy()                         # waiting for the electronic paper IC to release the idle signal

        self.__send_command(PANEL_SETTING)      # panel setting
        self.__send_data(0x0f)                  # LUT from OTP,128x296
        self.__send_data(0x89)                  # Temperature sensor, boost and other related timing settings

        self.__send_command(RESOLUTION_SETTING) # resolution setting
        self.__send_data (0x80)                 # default
        self.__send_data (0x01)                 # default
        self.__send_data (0x28)                 # default

        self.__send_command(VCOM_AND_DATA_INTERVAL_SETTING)     # WBRmode:VBDF F7 VBDW 77 VBDB 37  VBDR B7
        self.__send_data(0x77)                                  # WBmode:VBDF 17|D7 VBDW 97 VBDB 57

        self.__send_command(PLL_CONTROL)                        # Sets EPD operation frequency, can be removed 
        self.__send_data(0x3F)                                  # NOTE: No effect to refresh time, can be removed 

        self.clearSRAM()
        return 0


    def __setupComms(self):
        """
        Initializes all pin communications, GPIO and SPI

        """
        self.__log('SPI + GPIO start', 2)       # Setup all GPIO pins
        self.__GPIO.setmode(self.__GPIO.BOARD)  # BOARD:    Printed pin numbers
                                                # BCM:      https://bit.ly/3F8oFtG
        self.__GPIO.setwarnings(False)
        self.__GPIO.setup(self.RST_PIN, self.__GPIO.OUT)
        self.__GPIO.setup(self.DC_PIN, self.__GPIO.OUT)
        self.__GPIO.setup(self.CS_PIN, self.__GPIO.OUT)
        self.__GPIO.setup(self.BUSY_PIN, self.__GPIO.IN)
                                                # Start SPI communication
        self.__SPI.SYSFS_software_spi_begin()   # TODO: SWITCH TO NEW SPI LIBRARY!!
        return 0


    def __reset(self):
        """
        Resets the EPD hardware and awakes if in deep sleep 

        """
        self.__log('Reset', 3)
        self.__GPIO.output(self.RST_PIN, 1)
        self.__delay_ms(200)
        self.__GPIO.output(self.RST_PIN, 0)
        self.__delay_ms(2)
        self.__GPIO.output(self.RST_PIN, 1)
        self.__delay_ms(200)
        self.__log('Reset done', 3)

        
    def __delay_ms(self, delaytime):
        """
        Halts program using time lib

        Parameters:
            delaytime (int): amount of time to halt in milliseconds      
        """
        self.__log('Delay ' + str(delaytime) + 'ms', 4)         
        time.sleep(delaytime / 1000.0)



    ###############################
    ###    Display Functions    ###
    ###############################

    def setSRAM(self, blackimage, redimage):
        """
        Set the black and red SRAM, if None, keep current SRAM
        NOTE: this does not refresh the display, printScreen() does

        Parameters:
            blackimage (list): The byte list buffer to save to black SRAM, returned from getbuffer()
            redimage   (list): The byte list buffer to save to red SRAM, returned from getbuffer()
        """
        self.__log('Set SRAM:    ' + 'Black: ' + str(blackimage != None) + ',  Red: ' + str(redimage != None), 2)

        if (blackimage != None):
            self.__send_command(DATA_START_TRANSMISSION_1) # write black
            for i in range(0, int(self.width * self.height / 8)):
                self.__send_data(blackimage[i])

        if (redimage != None):
            self.__send_command(DATA_START_TRANSMISSION_2) # write red
            for i in range(0, int(self.width * self.height / 8)):
                self.__send_data(redimage[i])


    def clearSRAM(self, black=True, red=True):
        """
        Clears the SRAM by saving and all white buffer
        NOTE: this does not refresh the display, printScreen() does

        Parameters:
            black (bool): If true clear black SRAM (Defaults to True)
            red   (bool): If true clear red SRAM (Defaults to True)
        """
        self.__log('Clear SRAM:  ' + 'Black: ' + str(black) + ',  Red: ' + str(red), 2)
        self.setSRAM(self.white_buf if black else None, self.white_buf if red else None)


    def printScreen(self, blackimage=None, redimage=None):                       
        """
        Prints the red and black images from SRAM to the e-Paper
        NOTE: if an image buffer is supplied, it will save it to SRAM then print

        Parameters:
            blackimage (list): The byte list buffer to print to display, returned from getbuffer()
            redimage   (list): The byte list buffer to print to dispay, returned from getbuffer()
        """
        self.__log('PRINT START', 1)
        self.setSRAM(blackimage, redimage)
        self.__send_command(DISPLAY_REFRESH)
        self.readBusy()
        self.__log('PRINT DONE', 1)


    def clearDisplay(self): 
        """
        Clears the screen by clearing both SRAMs and printing
        Use only after a few screen updates to remove ghosts and shadows

        NOTE: This is the same as printScreen(self.white_buf, self.white_buf)
        """
        self.__log('Clear display', 3)
        self.clearSRAM()
        self.printScreen()


    def getbuffer(self, image):
        """
        Creates a buffer byte list formatted for printScreen() method

        Parameters:
            image (byte): a PIL Image object

        Returns:
            buf (list): The list of bytes ready to display to screen
        """
        
        buf = list(self.white_buf)
        image_monocolor = image.convert('1')
        imwidth, imheight = image_monocolor.size
        pixels = image_monocolor.load()

        self.__log('Buffer size: ' + str(len(buf)), 2)
        self.__log('Image width, height:  ' + str(imwidth) + ', ' + str(imheight), 2)

        if(imwidth == self.width and imheight == self.height):
            self.__log('Image is vertical', 4)
            for y in range(imheight):
                for x in range(imwidth):
                    # Set the bits for the column of pixels at the current position.
                    if pixels[x, y] == 0:
                        buf[int((x + y * self.width) / 8)] &= ~(0x80 >> (x % 8))

        elif(imwidth == self.height and imheight == self.width):
            self.__log('Image is horizontal', 4)
            for y in range(imheight):
                for x in range(imwidth):
                    newx = y
                    newy = self.height - x - 1
                    if pixels[x, y] == 0:
                        buf[int((newx + newy*self.width) / 8)] &= ~(0x80 >> (y % 8))
        return buf



    ###########################
    ###    SPI Functions    ###
    ###########################

    def readBusy(self):
        """
        Halts until EPD is not busy. EPD goes busy after every command.

        """
        self.__log('e-Paper busy', 2)
        self.__send_command(GET_STATUS)                 #  get status (update busy pin)
        while(self.__GPIO.input(self.BUSY_PIN) == 0):   #  0: idle, 1: busy
            self.__send_command(GET_STATUS)             #  get status (update busy pin)
        self.__log('e-Paper busy release', 2)


    def __spi_writebyte(self, data):
        """
        Sends a byte of data over SPI bus
        NOTE: needs to be accompanied by DC and CS pin states

        TODO: SWITCH TO NEW SPI LIBRARY!! Check func module_exit for reason
        replacements: python-spidev, or CircutPython board lib
        CircutPython example: https://bit.ly/3FXT92a 

        Parameters:
            data (byte): The byte to be sent over SPI
        """
        self.__SPI.SYSFS_software_spi_transfer(data) # TODO: SWITCH TO NEW SPI LIBRARY!!


    def __send_command(self, command):
        """
        Sends a command byte to the EPD. DC high and CS low signifies command.

        Command codes can be found in variables at end of file or 
        https://www.waveshare.com/w/upload/a/af/2.9inch-e-paper-b-v3-specification.pdf

        Parameters:
            command (byte): The byte command to be sent to the EPD

        """
        self.__log('Command sent: ' + str(command), 5)
        self.__GPIO.output(self.DC_PIN, 0)
        self.__GPIO.output(self.CS_PIN, 0)
        self.__spi_writebyte(command)
        self.__GPIO.output(self.CS_PIN, 1)


    def __send_data(self, data):
        """
        Send a data byte to the EPD. DC low and CS high signifies command.

        NOTE: Always preceded by __send_command() call.
        \/ Proper command data format can be found in datasheet \/ 
        https://www.waveshare.com/w/upload/a/af/2.9inch-e-paper-b-v3-specification.pdf

        Parameters:
            data (byte): The byte of data to be sent to the EPD

        """
        self.__log('Data sent: ' + str(data), 5)
        self.__GPIO.output(self.DC_PIN, 1)
        self.__GPIO.output(self.CS_PIN, 0)
        self.__spi_writebyte(data)
        self.__GPIO.output(self.CS_PIN, 1)



    ################################
    ###    Shutdown Functions    ###
    ################################

    def shutdown(self):
        """
        Turns e-Paper off,sends it to deep sleep, ends SPI, and frees GPIO
        NOTE: have to call module_init() to use EPD again, or maybe just __reset()

        """
        self.__log('e-Paper power down', 1)
        self.__send_command(POWER_OFF)    # power off
        self.readBusy()
        self.__send_command(DEEP_SLEEP)   # deep sleep
        self.__send_data(0xA5)            # confirmation packet
        self.__cleanupComms()             # clear up GPIO
        self.__log('##### ENDED EPD FUNCTION #####', 1)


    def __cleanupComms(self):
        """
        Exits the EPD, freeing any used GPIO and SPI pins and 
        BUG: ending the SPI prevents SPI from starting again without reboot (see TODO/FIXME/BUG markers for info)

        """
        self.__log('e-Paper pin cleanup', 3)
        # self.__SPI.SYSFS_software_spi_end()           # BUG: ending custom SPI library locks 
                                                        # BUG: ^SPI and needs reboot to re-enable
                                                        # TODO: SWITCH TO NEW SPI LIBRARY!!
        self.__GPIO.output(self.RST_PIN, 0)
        self.__GPIO.output(self.DC_PIN, 0)
        self.__GPIO.cleanup([self.RST_PIN, self.DC_PIN, self.CS_PIN, self.BUSY_PIN])



    ###############################
    ###    Logging Functions    ###
    ###############################

    def setLog(self, level, exclusive=False):
        """
        Sets the logging level local to this library 
        0 ==> No logs
        1 ==> Lifecycle updates
        2 ==> Useful debugging data 
        3 ==> Helper functions
        4 ==> Rest of method calls except SPI
        5 ==> Mayhem, all SPI method calls

        Parameters:
            level     (int):  integer for desired local logging level
            exclusive (bool): print logs only for defined level, default prints defined
        """
        self.__logLevelSelf = level
        self.__logExclusive = exclusive

        tag = self.__levelTag = 'OFF', 'Life', 'Data', 'Help', 'Misc', 'SPI0'
        info = 'No logs', 'lifecycle updates', 'useful debugging data', 'helper functions', 'miscellaneous methods', 'SPI'
        str_a = 'Log Level set to: ' + tag[level] + (' (exclusive)' if exclusive else '')
        str_b = '. Logs for ' + info[level] + ('' if exclusive else ' and lower') + ' will be printed.'
        self.__log(str_a + str_b, level)
        


    def __log(self, string, logLevel):
        """
        This helper method prints logs for this library formatted correctly

        Parameters:
            string   (str): string to log
            logLevel (int): integer of lowest log level to prompt this log
        """
        if self.__logger is None:
            return 0 
        header = '[EPD Lib] (' + self.__levelTag[logLevel] + '):    '
        # header = '[EPD Lib]:    '           # NOTE: Uncomment to remove log level tag
        doLog = (self.__logLevelSelf == logLevel) if self.__logExclusive else (self.__logLevelSelf >= logLevel)
        doLog and self.__logger.debug(header + string)


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
