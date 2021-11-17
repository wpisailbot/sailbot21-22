/**
 * @file Constants.h
 * @author Irina Lavryonova (ilavryonova@wpi.edu) - 2019/2020
 * @author Connor Burri (cjburri@wpi.edu) - 2020/2021
 * @brief File containing variables common to the entire system, centralizing the settings of the sailbot
 * @version 0.2
 * @date 2020-11-18
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef Constants_h
#define Constants_h

// necessary libraries for the constants
#include <IPAddress.h>
#include <Arduino.h>

/* Trim servo */
const int SERVO_CTR    = 115;
const int SERVO_LO_LIM = SERVO_CTR-55;
const int SERVO_HI_LIM = SERVO_CTR+55;

/* Wind vein*/
const int POT_HEADWIND = 463;

/* State angles */
const int MAX_LIFT_ANGLE = 30;

/* Pins */
const int potPin    = A0;             //A19
const int servoPin  = 16;             //Servo
const int led1Pin   = 4;              //white
const int led2Pin   = 5;              //white
const int wifiLED   = LED_BUILTIN;    //yellow
const int powerLED  = 7;              //red
const int RX2pin    = 9;              //ESP8266
const int TX2pin    = 10;             //ESP8266

/*  COMMS   */
const unsigned int port = 50000;
const IPAddress hullIP(10, 42, 0, 1);  // Hull's IP address
const char ssid[] = "sailbothot";                   // Name of the hull network
const char pass[] = "salad123";                     // Password to hull network

#endif
