#!/usr/bin/python
# -*- coding:utf-8 -*-
import sys
import os
import logging
from PIL import Image,ImageDraw,ImageFont

# local resources
from lib import epd_library, gompei

logging.basicConfig(level=logging.DEBUG)

import time
start_time = time.time()

try:
    print ("\n\n")
    logging.info("  ###### Boat Startup Screen ######\n")

    # Initialize
    epd = epd_library.EPaperDisplay()
    epd.InitQuick()
    logging.info("		  (OBD) Initialized")
    #epd.clear()
    epd.ClearFrame()
    epd.DisplayFrameQuick()       # This displays the data from the SRAM in e-Paper module

    logging.info("		  (OBD) Cleared")
    
    # Loading fonts
    font20b = ImageFont.truetype('/usr/share/fonts/truetype/msttcorefonts/Courier_New_Bold.ttf', 20)
    font20 = ImageFont.truetype('/usr/share/fonts/truetype/msttcorefonts/Courier_New.ttf', 20)
    logging.info("		  (OBD) Fonts loaded")

    # Making black buffer image with text
    HBlackimage = Image.new('1', (epd.height, epd.width), 255)  # 298*126
    drawblack = ImageDraw.Draw(HBlackimage)
    drawblack.text((120, 45), 'Boat booting', font = font20b, fill = 0)
    drawblack.text((120, 70), 'Please wait', font = font20, fill = 0)
    logging.info("		  (OBD) Images loaded")

    # Display black and red
    logging.info("		  (OBD) Printing")


    #epd.display(epd.getbuffer(HBlackimage), gompei.buffer)
    epd.SetPartialWindow(gompei.buffer, 0, 0, 20, 20, 2)
    epd.DisplayFrameQuick()
    epd.ClearFrame()



    # Clearing display
#    logging.info("Clear...")
#    epd.clear()
    

    # Put e-Paper in deep sleep
    logging.info("		  (OBD) Sleeeeeeep")
    epd.sleep()

    print("--- %s seconds ---" % (time.time() - start_time))

        
except IOError as e:
    logging.info(e)
    
except KeyboardInterrupt:    
    logging.info("		  EXIT: ctrl + c")
    epd_library.epdconfig.module_exit()
    exit()
