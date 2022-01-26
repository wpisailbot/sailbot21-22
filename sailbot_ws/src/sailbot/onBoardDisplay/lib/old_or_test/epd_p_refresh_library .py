# DO NOT USE. This is the unfinished partial refresh library


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

# Fitted for use with WPI's Sailbot MQP 2021-2022
# Contributors:     Tony Tesoriero
# Last Edited:      Jan. 8, 2022

# Resources:
# Wiki for our e-Paper model (NOTE: VERSION 3)
# https://www.waveshare.com/wiki/2.9inch_e-Paper_Module_(B)
#      User Guides of Jetson Nano Tab has quickstart guide and pinout
#      Resources tab has useful docs

# Data sheet with SPI command table and timing
# https://www.waveshare.com/w/upload/a/af/2.9inch-e-paper-b-v3-specification.pdf

# Github with examples and this library
# https://github.com/waveshare/e-Paper
#
# PINOUT (defined in epdconfig.py):
    # RST_PIN  -->   15   any digital pin
    # VCC      -->   17   any 3.3V
    # BUSY_PIN -->   18   any digital pin
    # DIN      -->   19   don't change (SPI1_MOSI)
    # GND      -->   20   any GND
    # DC_PIN   -->   22   any digital pin
    # CLK      -->   23   don't change (SPI1_SCK)
    # CS_PIN   -->   24   don't change (SPI1_CS0)
    
# ****************************************************************************


import logging
from . import epdconfig

# Display resolution
EPD_WIDTH       = 128
EPD_HEIGHT      = 296

logger = logging.getLogger(__name__)



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

class EPaperDisplay:
    def __init__(self):
        self.reset_pin = epdconfig.RST_PIN
        self.dc_pin = epdconfig.DC_PIN
        self.busy_pin = epdconfig.BUSY_PIN
        self.cs_pin = epdconfig.CS_PIN
        self.width = EPD_WIDTH
        self.height = EPD_HEIGHT

    def init(self):
        if (epdconfig.module_init() != 0):
            return -1

        # EPD hardware init start
        self.reset()

        # self.send_command(POWER_SETTING)
        # self.send_data(0x03)                  # VDS_EN, VDG_EN
        # self.send_data(0x00)                  # VCOM_HV, VGHL_LV[1], VGHL_LV[0]
        # self.send_data(0x26)                  # VDH
        # self.send_data(0x26)                  # VDL
        # self.send_data(0x03)                  # VDHR

        # self.send_command(BOOSTER_SOFT_START) #checked
        # self.send_data(0x17)
        # self.send_data(0x17)
        # self.send_data(0x17)                  #07 0f 17 1f 27 2F 37 2f

        self.send_command(POWER_ON)             # power on
        self.readBusy()                         # waiting for the electronic paper IC to release the idle signal

        self.send_command(PANEL_SETTING)        # panel setting
        self.send_data(0x0f)                    # LUT from OTP,128x296
        self.send_data(0x89)                    # Temperature sensor, boost and other related timing settings

        self.send_command(RESOLUTION_SETTING)   # resolution setting
        self.send_data (0x80)
        self.send_data (0x01)
        self.send_data (0x28)
        
        # self.send_command(RESOLUTION_SETTING)
        # self.send_data(self.width >> 8)
        # self.send_data(self.height >> 8)
        # self.send_data(self.height & 0xff)

        self.send_command(VCOM_AND_DATA_INTERVAL_SETTING)     # VCOM AND DATA INTERVAL SETTING
        self.send_data(0x77)                                  # WBmode:VBDF 17|D7 VBDW 97 VBDB 57
                                                              # WBRmode:VBDF F7 VBDW 77 VBDB 37  VBDR B7
        # self.send_command(VCOM_DC_SETTING)
        # self.send_data(0x00)

        # DO NOT USE
        self.send_command(PLL_CONTROL)
        self.send_data(0x3F)
        return 0

    # Hardware reset
    def reset(self):
        epdconfig.digital_write(self.reset_pin, 1)
        epdconfig.delay_ms(200)
        epdconfig.digital_write(self.reset_pin, 0)
        epdconfig.delay_ms(2)
        epdconfig.digital_write(self.reset_pin, 1)
        epdconfig.delay_ms(200)

    def send_command(self, command):
        epdconfig.digital_write(self.dc_pin, 0)
        epdconfig.digital_write(self.cs_pin, 0)
        epdconfig.spi_writebyte([command])
        epdconfig.digital_write(self.cs_pin, 1)

    def send_data(self, data):
        epdconfig.digital_write(self.dc_pin, 1)
        epdconfig.digital_write(self.cs_pin, 0)
        epdconfig.spi_writebyte([data])
        epdconfig.digital_write(self.cs_pin, 1)

    def readBusy(self):
        logger.debug("    e-Paper busy")
        self.send_command(GET_STATUS)                         #  get status (update busy pin)
        while(epdconfig.read_busy(self.busy_pin) == 0): #  0: idle, 1: busy
            self.send_command(GET_STATUS)                     #  get status (update busy pin)
            epdconfig.delay_ms(20)
        logger.debug("    e-Paper busy release")

    def getbuffer(self, image):
        # logger.debug("bufsiz = ",int(self.width/8) * self.height)
        buf = [0xFF] * (int(self.width/8) * self.height)
        image_monocolor = image.convert('1')
        imwidth, imheight = image_monocolor.size
        pixels = image_monocolor.load()

        # logger.debug("imwidth = %d, imheight = %d",imwidth,imheight)
        if(imwidth == self.width and imheight == self.height):
            logger.debug("    image is Vertical")
            for y in range(imheight):
                for x in range(imwidth):
                    # Set the bits for the column of pixels at the current position.
                    if pixels[x, y] == 0:
                        buf[int((x + y * self.width) / 8)] &= ~(0x80 >> (x % 8))

        elif(imwidth == self.height and imheight == self.width):
            logger.debug("    image is Horizontal")
            for y in range(imheight):
                for x in range(imwidth):
                    newx = y
                    newy = self.height - x - 1
                    if pixels[x, y] == 0:
                        buf[int((newx + newy*self.width) / 8)] &= ~(0x80 >> (y % 8))
        return buf

    def display(self, blackimage, ryimage):                          # ryimage: red or yellow image
        if (blackimage != None):
            self.send_command(DATA_START_TRANSMISSION_1)                                  # write white
            for i in range(0, int(self.width * self.height / 8)):
                self.send_data(blackimage[i])
        if (ryimage != None):
            self.send_command(DATA_START_TRANSMISSION_2)                                  # write red
            for i in range(0, int(self.width * self.height / 8)):
                self.send_data(ryimage[i])

        self.send_command(DISPLAY_REFRESH)                                      # display refresh
        #epdconfig.delay_ms(200)
        self.readBusy()

    def clear(self):
        white = [0xff] * int(self.width * self.height / 8)
        self.display(white, white)


    def sleep(self):
        self.send_command(POWER_OFF) # power off
        self.readBusy()
        self.send_command(DEEP_SLEEP) # deep sleep
        self.send_data(0xA5)    # confirmation packet

       # epdconfig.delay_ms(2000)
        #epdconfig.module_exit()


    ################ NEW DRIVERS ################

    def InitQuick(self):
        if (epdconfig.module_init() != 0):
            return -1

        self.reset()
        # self.send_command(POWER_SETTING)
        # self.send_data(0x03)                  # VDS_EN, VDG_EN
        # self.send_data(0x00)                  # VCOM_HV, VGHL_LV[1], VGHL_LV[0]
        # self.send_data(0x2b)                  # VDH
        # self.send_data(0x2b)                  # VDL
        # self.send_data(0xff)                  # VDHR
        self.send_command(BOOSTER_SOFT_START) #checked
        self.send_data(0x17)
        self.send_data(0x17)
        self.send_data(0x17)                  #07 0f 17 1f 27 2F 37 2f
        self.send_command(POWER_ON)
        self.readBusy()
        self.send_command(PANEL_SETTING)
        #self.send_data(0xbf)   # KW-BF   KWR-AF  BWROTP 0f
        #self.send_data(0x0b)
        #self.send_data(0x0F)   #300x400 Red mode, LUT from OTP
        #self.send_data(0x1F)   #300x400 B/W mode, LUT from OTP
        self.send_data(0x3F)    #300x400 B/W mode, LUT set by register
        #self.send_data(0x2F)   #300x400 Red mode, LUT set by register

        self.send_command(RESOLUTION_SETTING)   # resolution setting
        self.send_data (0x80)
        self.send_data (0x01)
        self.send_data (0x28)

        #self.send_command(PLL_CONTROL)
        #self.send_data(0x3C)        # 3A 100Hz   29 150Hz   39 200Hz    31 171Hz       3C 50Hz (default)    0B 10Hz
        #self.send_data(0x0B)   #0B is 10Hz

        return 0

    # transmit partial data to the SRAM.  The final parameter chooses between dtm=1 and dtm=2
    def SetPartialWindow(self, buffer_black, x, y, w, h, dtm):

        w = self.width
        h = self.height

        xe = (x+w-1) | 0x0007 # byte boundary inclusive (last byte)
        ye = y+h-1
        x &= 0xFFF8 # byte boundary

        self.send_command(PARTIAL_IN)
        self.send_command(PARTIAL_WINDOW)
        #self.send_data(x / 256)
        self.send_data(x % 256)
        #self.send_data(xe / 256)
        self.send_data(xe % 256)

        #self.send_data(y / 256)
        self.send_data(y % 256)
        #self.send_data(ye / 256)
        self.send_data(ye % 256)

        # self.send_data(x >> 8)
        # self.send_data(x & 0xf8)     # x should be the multiple of 8, the last 3 bit will always be ignored
        # self.send_data(((x & 0xf8) + w  - 1) >> 8)
        # self.send_data(((x & 0xf8) + w  - 1) | 0x07)

        # self.send_data(y >> 8)
        # self.send_data(y & 0xff)
        # self.send_data((y + l - 1) >> 8)
        # self.send_data((y + l - 1) & 0xff)
        self.send_data(0x00)         # Gates scan both inside and outside of the partial window. (default)
        #epdconfig.delay_ms(2)
        self.send_command(DATA_START_TRANSMISSION_1 if (dtm == 1) else DATA_START_TRANSMISSION_2)
        if (buffer_black != None):
            for i in range(0, int(w*h/8)):
                self.send_data(buffer_black[i])

        else:
            for i in range(0, int(w*h/8)):
                self.send_data(0x00)  #NOTE 0x00

        self.send_command(DISPLAY_REFRESH)
        #epdconfig.delay_ms(2)
        self.send_command(PARTIAL_OUT)



    # set the look-up table
    def SetLut(self):
        #vcom, ww --, bw r, wb w, bb b
        command_array = [LUT_FOR_VCOM, LUT_WHITE_TO_WHITE, LUT_BLACK_TO_WHITE, LUT_WHITE_TO_BLACK, LUT_BLACK_TO_BLACK]
        data_array = [lut_vcom0, lut_ww, lut_bw, lut_wb, lut_bb]

        for i in range(0, len(command_array)):
            self.send_command(command_array[i])
            for data in data_array[i]:
                self.send_data(data)



    # set the look-up table for quick display (partial refresh)
    def SetLutQuick(self):
        #vcom, ww --, bw r, wb w, bb b
        command_array = [LUT_FOR_VCOM, LUT_WHITE_TO_WHITE, LUT_BLACK_TO_WHITE, LUT_WHITE_TO_BLACK, LUT_BLACK_TO_BLACK]
        data_array = [lut_20_vcomDC_partial, lut_21_ww_partial, lut_22_bw_partial, lut_23_wb_partial, lut_24_bb_partial]
        #data_array = [VCOM_LUT_LUTC_PARTIAL, W2W_LUT_LUTWW_PARTIAL, B2W_LUT_LUTBW_LUTR_PARTIAL, W2B_LUT_LUTWB_LUTW_PARTIAL, B2B_LUT_LUTBB_LUTB_PARTIAL]
        for i in range(0, len(command_array)):
            self.send_command(command_array[i])
            for data in data_array[i]:
                self.send_data(data)

    # refresh and displays the frame
    def DisplayFrame(self, frame_buffer):
        #self.send_command(RESOLUTION_SETTING)

        #self.send_data(self.width >> 8)
        #self.send_data(self.width & 0xff)
        #self.send_data(self.height >> 8)
        #self.send_data(self.height & 0xff)

        self.send_command(VCOM_DC_SETTING)
        self.send_data(0x12)

        self.send_command(VCOM_AND_DATA_INTERVAL_SETTING)
        self.send_data(0x97)    #NOTE: Default 0x77 | listed 0x97     VBDF 17|D7 VBDW 97  VBDB 57  VBDF F7  VBDW 77  VBDB 37  VBDR B7

        if (frame_buffer != None):
            self.send_command(DATA_START_TRANSMISSION_1)
            for i in range(0, int(self.width * self.height / 8)):
                self.send_data(0xFF)      # bit set: white, bit reset: black

            epdconfig.delay_ms(2)

            self.send_command(DATA_START_TRANSMISSION_2)
            for i in range(0, int(self.width * self.height / 8)):
                self.send_data(frame_buffer[i])

            epdconfig.delay_ms(2)

        self.SetLut()

        self.send_command(DISPLAY_REFRESH)
        epdconfig.delay_ms(100)
        self.readBusy()





    # clear the frame data from the SRAM, this won't refresh the display

    def ClearFrame(self):
        #self.send_command(RESOLUTION_SETTING)
       # self.send_data(self.width >> 8)
       # self.send_data(self.width & 0xff)
       # self.send_data(self.height >> 8)
        #self.send_data(self.height & 0xff)

        self.send_command(DATA_START_TRANSMISSION_1)
        epdconfig.delay_ms(2)
        for i in range(0, int(self.width * self.height / 8)):
            self.send_data(0xFF)

        epdconfig.delay_ms(2)
        self.send_command(DATA_START_TRANSMISSION_2)
        epdconfig.delay_ms(2)
        for i in range(0, int(self.width * self.height / 8)):
            self.send_data(0xFF)

        epdconfig.delay_ms(2)




    # This displays the frame data from SRAM

    def DisplayFrame(self):
        self.SetLut()
        self.send_command(DISPLAY_REFRESH)
        epdconfig.delay_ms(100)
        self.readBusy()


    def DisplayFrameQuick(self):
        self.SetLutQuick()
        self.send_command(DISPLAY_REFRESH)
        #epdconfig.delay_ms(100)
        #self.readBusy()



lut_20_vcomDC_partial = [0x00, 0x1F, 0x01, 0x00, 0x00, 0x01]

lut_21_ww_partial = [0x00, 0x1F, 0x01, 0x00, 0x00, 0x01]

lut_22_bw_partial = [0x80, 0x1F, 0x01, 0x00, 0x00, 0x01]

lut_23_wb_partial = [0x40, 0x1F, 0x01, 0x00, 0x00, 0x01]

lut_24_bb_partial = [0x00, 0x1F, 0x01, 0x00, 0x00, 0x01]



# lut_vcom0 = [
# 0x40, 0x17, 0x00, 0x00, 0x00, 0x02,
# 0x00, 0x17, 0x17, 0x00, 0x00, 0x02,
# 0x00, 0x0A, 0x01, 0x00, 0x00, 0x01,
# 0x00, 0x0E, 0x0E, 0x00, 0x00, 0x02,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

# lut_vcom0_quick = [
# 0x00, 0x0E, 0x00, 0x00, 0x00, 0x01,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]



# lut_ww = [
# 0x40, 0x17, 0x00, 0x00, 0x00, 0x02,
# 0x90, 0x17, 0x17, 0x00, 0x00, 0x02,
# 0x40, 0x0A, 0x01, 0x00, 0x00, 0x01,
# 0xA0, 0x0E, 0x0E, 0x00, 0x00, 0x02,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

# lut_ww_quick = [
# 0xA0, 0x0E, 0x00, 0x00, 0x00, 0x01,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]


# lut_bw = [
# 0x40, 0x17, 0x00, 0x00, 0x00, 0x02,
# 0x90, 0x17, 0x17, 0x00, 0x00, 0x02,
# 0x40, 0x0A, 0x01, 0x00, 0x00, 0x01,
# 0xA0, 0x0E, 0x0E, 0x00, 0x00, 0x02,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]


# lut_bw_quick = [
# 0xA0, 0x0E, 0x00, 0x00, 0x00, 0x01,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

# lut_bb = [
# 0x80, 0x17, 0x00, 0x00, 0x00, 0x02,
# 0x90, 0x17, 0x17, 0x00, 0x00, 0x02,
# 0x80, 0x0A, 0x01, 0x00, 0x00, 0x01,
# 0x50, 0x0E, 0x0E, 0x00, 0x00, 0x02,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

# lut_bb_quick = [
# 0x50, 0x0E, 0x00, 0x00, 0x00, 0x01,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]


# lut_wb = [
# 0x80, 0x17, 0x00, 0x00, 0x00, 0x02,
# 0x90, 0x17, 0x17, 0x00, 0x00, 0x02,
# 0x80, 0x0A, 0x01, 0x00, 0x00, 0x01,
# 0x50, 0x0E, 0x0E, 0x00, 0x00, 0x02,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

# lut_wb_quick = [
# 0x50, 0x0E, 0x00, 0x00, 0x00, 0x01,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
# 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]


# ### END OF FILE ###

# VCOM_LUT_LUTC=[
# 0x00	,0x08	,0x00	,0x00	,0x00	,0x02,
# 0x60	,0x28	,0x28	,0x00	,0x00	,0x01,
# 0x00	,0x14	,0x00	,0x00	,0x00	,0x01,
# 0x00	,0x12	,0x12	,0x00	,0x00	,0x01,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00]
# W2W_LUT_LUTWW=[
# 0x40	,0x08	,0x00	,0x00	,0x00	,0x02,
# 0x90	,0x28	,0x28	,0x00	,0x00	,0x01,
# 0x40	,0x14	,0x00	,0x00	,0x00	,0x01,
# 0xA0	,0x12	,0x12	,0x00	,0x00	,0x01,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00]
# B2W_LUT_LUTBW_LUTR=[
# 0x40	,0x17	,0x00	,0x00	,0x00	,0x02,
# 0x90	,0x0F	,0x0F	,0x00	,0x00	,0x03,
# 0x40	,0x0A	,0x01	,0x00	,0x00	,0x01,
# 0xA0	,0x0E	,0x0E	,0x00	,0x00	,0x02,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00]
# W2B_LUT_LUTWB_LUTW=[
# 0x80	,0x08	,0x00	,0x00	,0x00	,0x02,
# 0x90	,0x28	,0x28	,0x00	,0x00	,0x01,
# 0x80	,0x14	,0x00	,0x00	,0x00	,0x01,
# 0x50	,0x12	,0x12	,0x00	,0x00	,0x01,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00]
# B2B_LUT_LUTBB_LUTB=[
# 0x80	,0x08	,0x00	,0x00	,0x00	,0x02,
# 0x90	,0x28	,0x28	,0x00	,0x00	,0x01,
# 0x80	,0x14	,0x00	,0x00	,0x00	,0x01,
# 0x50	,0x12	,0x12	,0x00	,0x00	,0x01,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00]


# VCOM_LUT_LUTC_PARTIAL=[
# 0x00	,0x19	,0x01	,0x00	,0x00	,0x01,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00]
# W2W_LUT_LUTWW_PARTIAL=[
# 0x00	,0x19	,0x01	,0x00	,0x00	,0x01,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00]
# B2W_LUT_LUTBW_LUTR_PARTIAL=[
# 0x80	,0x19	,0x01	,0x00	,0x00	,0x01,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00]
# W2B_LUT_LUTWB_LUTW_PARTIAL=[
# 0x40	,0x19	,0x01	,0x00	,0x00	,0x01,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00]
# B2B_LUT_LUTBB_LUTB_PARTIAL=[
# 0x00	,0x19	,0x01	,0x00	,0x00	,0x01,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00,
# 0x00	,0x00	,0x00	,0x00	,0x00	,0x00]
