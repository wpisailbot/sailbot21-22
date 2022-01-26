# ROS Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# e-Paper Imports
import sys
import os
from PIL import Image,ImageDraw,ImageFont

import onBoardDisplay.lib.epd_library as epd_library
import onBoardDisplay.lib.gompei as gompei

shouldLogHelpers = True

class OBDController(Node):


    def __init__(self):
        super().__init__('obd_controller')

        ### NOTE: LOGGER LEVEL ###
        DEBUG,INFO,WARN,ERROR,FATAL,UNSET=10,20,30,40,50,00
        self.get_logger().set_level(DEBUG) 

        # Initialize E-Paper
        self.epd = epd_library.EPaperDisplay(self.get_logger())
        self.epd.setLog(3)
        self.epd.module_init()

        # Load fonts
        self.fontDir = '/usr/share/fonts/truetype/msttcorefonts/Courier_New.ttf'
        self.fontBoldDir = self.fontDir[:len(self.fontDir)-4] + '_Bold.ttf'

        self.subscription = self.create_subscription(String, 'test_OBD_string', self.updateDisplay, 10)
        self.subscription  # prevent unused variable warning

        # Show bootup screen 
        self.debuggingScreen()
        self.bool = True
        self.epd.sleep()


    #############################
    ###    Display Screens    ###
    #############################

    def bootingScreen(self):
        self.get_logger().info('######## Boat Startup Screen ########')
        self.epd.clear()

        # Load fonts
        font20 = ImageFont.truetype(self.fontDir, 20)
        font20b = ImageFont.truetype(self.fontBoldDir, 20)

        # Making an image and draw text for black render
        Blackimage = Image.new('1', (self.epd.height, self.epd.width), 255)  # 298*126
        drawblack = ImageDraw.Draw(Blackimage)
        drawblack.text((120, 45), 'Boat booting', font = font20b)
        drawblack.text((120, 70), 'Please wait', font = font20)

        # Display black and red
        self.epd.display(self.epd.getbuffer(Blackimage), gompei.buffer)


    def debuggingScreen(self):

        # Telem connected, base station wifi, trim tab wifi, trim tab voltage
        # 
        self.get_logger().info('######### Boat Debug Screen #########')

        connectedTuple = ('STATE:', 'TrimTab:', 'RC:', 'Wifi:', 'Jetson:', 'BT:')
        stateTuple = ('MODES:', 'Wing:', 'Ballast:', 'Wintch:', 'Rudder:', 'Tacker:')

        # PIL ImageDraw docs: https://bit.ly/3GZAuUO 
        Blackimage = Image.new('1', (self.epd.height, self.epd.width), 255)
        drawblack = ImageDraw.Draw(Blackimage)

        # Load font
        font24 = ImageFont.truetype(self.fontDir, 24)

        # Print both columns of strings and voltage
        #               (drawObj,        [col1, col2, ...],     xStart, colSpacing, yPadding, fontSize, bold)
        self.drawColumns(drawblack, (connectedTuple, stateTuple), 10,       120,        5,       16,    True)
        drawblack.text((225, 0), "14.8V", font = font24)

        # Display created image
        self.epd.display(self.epd.getbuffer(Blackimage), None)


    def oldDebuggingScreen(self):
        """
        Prints OG 17/18 Sailbot e-Paper screen layout

        NOTE: doesn't update, the boat has changed (good for testing layouts)
        PIL ImageDraw docs: https://bit.ly/3GZAuUO 

        """
        self.get_logger().info('###### 17/18 Boat Debug Screen ######')

        # \/\/\/\/ 17/18 Original debugging info \/\/\/\/
        connectedTuple = ('CONNECTED:', 'Wing:', 'Radio:', 'Wifi:', 'Jetson:', 'BT:')
        stateTuple = ('MODES:', 'Wing:', 'Ballast:', 'Wintch:', 'Rudder:', 'Tacker:')

        # PIL ImageDraw docs: https://bit.ly/3GZAuUO 
        Blackimage = Image.new('1', (self.epd.height, self.epd.width), 255)
        drawblack = ImageDraw.Draw(Blackimage)

        # Load font
        font24 = ImageFont.truetype(self.fontDir, 24)

        # Print both columns of strings and voltage
        #               (drawObj,        [col1, col2, ...],     xStart, colSpacing, yPadding, fontSize, bold)
        self.drawColumns(drawblack, (connectedTuple, stateTuple), 10,       120,        5,       16,    True)
        drawblack.text((225, 0), "14.8V", font = font24)

        # Display created image
        self.epd.display(self.epd.getbuffer(Blackimage), None)




    ##############################
    ###    Helper Functions    ###
    ##############################

    # NOTE: PIL LIBRARY ALREADY DOES THIS
    # this can be removed and replaced with ImageDraw.multiline_text()
    # i.e. drawblack.multiline_text()
    def drawListY(self, drawObj, stringList, xOffset, yOffset, spacing, fontSize=16, isBold=True):
        """
        Prints OG 17/18 Sailbot e-Paper screen layout

        """
        font = ImageFont.truetype(self.fontBoldDir if isBold else self.fontDir, fontSize)
        for i in range(0, len(stringList)):
            self.logForHelpers('Draw List Y: ' + str(round(yOffset+i*spacing)) + ', \"' + stringList[i]+ '\"')
            drawObj.text((xOffset, round(yOffset+i*spacing)), stringList[i], font = font)
        return 0

    # This fun takes a list of strings and draws them vertically, evenly spaced
    # yPadding is the amount of space 
    def drawListFitY(self, drawObj, stringList, xOffset, yPadding, fontSize=16, isBold=True):
        # Find spacing from bottom of each line of text (width is 128, height is 298)
            # take usable screen height and divide by number of lines 
            # (abs screen "height" - (top and bot padding) ) / # of lines
            # -3 used for top and bottom bounds of screen not centered
        spacing = (self.epd.width-3-(yPadding*2)) / len(stringList)
        
        # This makes the bottom of the last line of text flush with the padding
        spacing += (spacing-fontSize) / (len(stringList)-1)

        self.logForHelpers('Draw List Fit Spacing: ' + str(round(spacing,1))) 
        self.drawListY(drawObj, stringList, xOffset, yPadding, spacing, fontSize, isBold)

    def drawColumns(self, drawObj, columns, xStart, columnSpacing, yPadding, fontSize=16, isBold=True):
        for i in range(0, len(columns)):
            self.logForHelpers('Draw Columns X: ' + str(xStart+columnSpacing*i))
            self.drawListFitY(drawObj, columns[i], xStart+columnSpacing*i, yPadding, fontSize, isBold)    


    def logForHelpers(self, string):
        shouldLogHelpers and self.get_logger().debug('(Helper):     ' + string)



    #################################
    ###    Subscriber Callbacks   ###
    #################################

    def updateDisplay(self, msg):
        if self.bool:
            self.debuggingScreen()
            self.bool = False
        self.get_logger().info('Test String: ' + msg.data)





    

def main(args=None):
    rclpy.init(args=args)
    obd_controller = OBDController()    # Create on board display object
    rclpy.spin(obd_controller)
    obd_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
