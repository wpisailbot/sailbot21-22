






"""OUTDATED! Has been encorperated into epd_library.py"""
# # /*****************************************************************************
# # * | File        :	  epdconfig.py
# # * | Author      :   Waveshare team
# # * | Function    :   Hardware underlying interface
# # * | Info        :
# # *----------------
# # * | This version:   V1.0
# # * | Date        :   2019-06-21
# # * | Info        :   
# # ******************************************************************************
# # Permission is hereby granted, free of charge, to any person obtaining a copy
# # of this software and associated documnetation files (the "Software"), to deal
# # in the Software without restriction, including without limitation the rights
# # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# # copies of the Software, and to permit persons to  whom the Software is
# # furished to do so, subject to the following conditions:
# #
# # The above copyright notice and this permission notice shall be included in
# # all copies or substantial portions of the Software.
# #
# # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# # FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# # LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# # THE SOFTWARE.
# #

# import os
# import logging
# import sys
# import time

# class JetsonNano:

#     # Pin definition 
#     # NOTE: GPIO Lib in BOARD mode, ref. https://bit.ly/3F8oFtG
#     # Pin 21 and 26 off limits

#     RST_PIN         = 15 #  any digital pin
#     # VCC     -->     17    any 3.3V
#     BUSY_PIN        = 18 #  any digital pin
#     # DIN     -->     19    Don't change (SPI1_MOSI)
#     # GND     -->     20    any GND
#     DC_PIN          = 22 #  any digital pin
#     # CLK     -->     23    Don't change (SPI1_SCK)
#     CS_PIN          = 24 #  Don't change (SPI1_CS0)

#     def __init__(self, logger):

#         self.logger = logger
#         import ctypes
#         find_dirs = [
#             os.path.dirname(os.path.realpath(__file__)),
#             '/usr/local/lib',
#             '/usr/lib',
#         ]

#         # TODO: SWITCH TO NEW SPI LIBRARY!!
#         # Check func module_exit for reason
#         # replacements: python-spidev,
#         # or CircutPython board lib
#         # ^^^ example: https://bit.ly/3FXT92a
#         self.SPI = None
#         for find_dir in find_dirs:
#             so_filename = os.path.join(find_dir, 'sysfs_software_spi.so')
#             if os.path.exists(so_filename):
#                 self.SPI = ctypes.cdll.LoadLibrary(so_filename)
#                 break
#         if self.SPI is None:
#             raise RuntimeError('Cannot find sysfs_software_spi.so')

#         import Jetson.GPIO
#         self.GPIO = Jetson.GPIO

#     def digital_write(self, pin, value):
#         self.GPIO.output(pin, value)

#     # TODO: refactor out unused pin param
#     def read_busy(self, pin):
#         return self.GPIO.input(self.BUSY_PIN)

#     def delay_ms(self, delaytime):
#         time.sleep(delaytime / 1000.0)

#     def spi_writebyte(self, data):
#         self.SPI.SYSFS_software_spi_transfer(data[0]) # TODO: SWITCH TO NEW SPI LIBRARY!!

#     def module_init(self):
#         self.GPIO.setmode(self.GPIO.BOARD)  # BOARD:    Printed pin numbers
#                                             # BCM:      https://bit.ly/3F8oFtG
#         self.GPIO.setwarnings(False)
#         self.GPIO.setup(self.RST_PIN, self.GPIO.OUT)
#         self.GPIO.setup(self.DC_PIN, self.GPIO.OUT)
#         self.GPIO.setup(self.CS_PIN, self.GPIO.OUT)
#         self.GPIO.setup(self.BUSY_PIN, self.GPIO.IN)
#         self.SPI.SYSFS_software_spi_begin() # TODO: SWITCH TO NEW SPI LIBRARY!!
#         return 0

#     def module_exit(self):
#         self.logger.debug("      spi end")
#         #self.SPI.SYSFS_software_spi_end()              # TODO: ending custom SPI library locks 
#                                                         # TODO: ^SPI and needs reboot to re-enable
#                                                         # TODO: SWITCH TO NEW SPI LIBRARY!!
#         self.logger.debug("      close 5V, Module enters 0 power consumption ...")
#         self.GPIO.output(self.RST_PIN, 0)
#         self.GPIO.output(self.DC_PIN, 0)

#         self.GPIO.cleanup([self.RST_PIN, self.DC_PIN, self.CS_PIN, self.BUSY_PIN])

# ### END OF FILE ###
