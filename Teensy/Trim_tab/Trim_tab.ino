/**
 * @file Teensy.ino
 * @author Irina Lavryonova (ilavryonova@wpi.edu) - 2019/2020
 * @author Connor Burri (cjburri@wpi.edu) - 2020/2021
 * @brief File containing the execution code for the teensy embedded within the adjustable trim tab
 * @version 0.2
 * @date 2020-11-18
 * @copyright Copyright (c) 2020
 * @TODO: see if any of these variables can be moved into the constants file 
 */

/* Custom struct for the possible states that the trim tab can be in */
typedef enum _TrimState_TRIM_STATE {
    TrimState_TRIM_STATE_MAX_LIFT_PORT = 0,
    TrimState_TRIM_STATE_MAX_LIFT_STBD = 1,
    TrimState_TRIM_STATE_MAX_DRAG_PORT = 2,
    TrimState_TRIM_STATE_MAX_DRAG_STBD = 3,
    TrimState_TRIM_STATE_MIN_LIFT = 4,
    TrimState_TRIM_STATE_MANUAL = 5
} TrimState_TRIM_STATE;

/* File containing all constants for the trim tab to operate; mainly pins and comms */
#include "Constants.h"

/* Libraries */
#include <WiFiNINA.h>                         //driver code for ESP8266 module
//#include "SoftwareSerial.h"                 //driver code for converting ESP8266 to Serial output
#include <Servo.h>                            //driver code for operating servo
#include <ArduinoJson.h>                      //library to assist in the serialization and deserialization of JSON documents
#include <arduino-timer.h>                    //library to handle non-blocking function calls at a set interval

/* Receive and transmission buffers */
uint8_t rx_buffer[64];                        // receive buffer
unsigned char tx_buffer[64];                  // transmission buffer

/* Wifi variables */
//SoftwareSerial ESPSerial(RX2pin, TX2pin);     // initializing the serial output from the ESP8266 
int status;                                   // the Wifi radio's status
WiFiClient client;                            // communication between jetson server and teensy client
bool connection;                              // connection indicator
int bytesAvailable;                           // the number of bytes available to be received from the RX Buffer
size_t bytesSerialized;                       // the number of bytes that have been serialized into the JSON to be sent to the Jetson
DynamicJsonDocument responseJSON(256);        // the JSON document that will be written upon receiving a message from the Jetson
DynamicJsonDocument transmitJSON(256);        // the JSON document that will be written upon sending a message to the Jetson

/* Control variables */
volatile int ledState;                        // for controlling the state of an led
auto LEDTimer = timer_create_default();       // sets the led timer function to be called asynchronously on an interval
auto servoTimer = timer_create_default();;    // sets the servo timer function to be called asynchronously on an interval
Servo servo;                                  // servo object
volatile float windAngle;                     // mapped reading from wind direction sensor on the front of the sail
int control_angle;                            // the current angle that the servo is set to
bool readingNow;                              // boolean variable that gets flipped for repeatedly receiving and transmitting 
TrimState_TRIM_STATE state;                   // the variable responsible for knowing what state the trim tab is in

void setup()
{
  /* Setting the mode of the pins necessary */
  pinMode(powerLED, OUTPUT);                  // led on the button
  pinMode(wifiLED, OUTPUT);                   // led next to the button (2nd from front)
  pinMode(led1Pin, OUTPUT);                   // led next to the wifi led (3rd from front)
  pinMode(led2Pin, OUTPUT);                   // led next to led1Pin (4th from front)
  
  
  /* Initializing variables to initial conditions */
  status = WL_IDLE_STATUS;                    // setting the wifi module status to be idle at first
  connection = false;                         // by default we have not established a connection
  ledState = LOW;                             // led starts in off position
  control_angle = SERVO_CTR;                  // trim tab starts off centralized
  readingNow = true;                          // we start off not reading, the first step will be to read into RX buffer
  state = TrimState_TRIM_STATE_MIN_LIFT;      // the state is set to be in the center position or what we consider to be min lift
  
  /* Starting the serial monitor */
  Serial.begin(115200);
  
  /* Giving feedback that the power is on */
  digitalWrite(powerLED, HIGH);

  /* Setting up the ESP8266 */
  //ESPSerial.begin(115200);                    // initialize serial for ESP module
  //WiFi.init(&ESPSerial);                      // initialize ESP module

  /* Establishing a connection between the Teensy microcontroller and the Jetson Nano */
  Serial.println("Starting...");
  establishConnection();

  /* Initializing the servo and setting it to its initial condition */
  servo.attach(servoPin);
  servo.write(control_angle);

  /* Starting the asynchronous function calls */
  servoTimer.every(10, servoControl);
  LEDTimer.every(10, blinkState);
}

void loop()
{
  JetsonSendReceive();                        // repeatedly sends and receives messages between the Teensy and Jetson
  servoTimer.tick();
  LEDTimer.tick();
}

/** 
 * @author  Connor Burri
 * @brief   Reads incoming messages from Jetson Nano and transmits messages to Jetson Nano; uses JSON
 * @details Writes the relative wind angle direction to the sail to the Jetson Nano over a socket connection
 *          then it receives a command from the Jetson Nano. This write/read command cycles repeatedly as long
 *          as there is a connection over the socket. If no connection is found then there will be an attempt to 
 *          reconnect the Teensy to the Jetson Nano.
 */
void JetsonSendReceive()
{
   /* ensure that a connection is bridged before attempting to read or write */
  if (!connection)
  {
    client.stop();                            // terminate anything that may be in the client
    establishConnection();                    // attempt to reestablish the connection
  }
   
  /* alternate between writing data to the jetson and reading data from the jetson */
  else {
    if(!readingNow){
      writeJson();
    }
    if(readingNow){
      readJson();
    }
  }

  /* Setup for next call to JetsonSendReceive() */
  delay(1000);                              // delay so that the buffers don't get overfilled
}

/** 
 * @author  Connor Burri
 * @brief   Reads in data from ESP8266 and deserializes data into a JSON
 * @bug     JSON objects requires a lot of memory (relative); not a pressing issue but still something to keep in mind
 */
void readJson(){
  /* Make sure that we are connected to the server before performing read operations */
  if(client.connected()){
    Serial.println("reading...");
    /* Check if there is anything to be read from ESP8266 */
    bytesAvailable = client.available();
    
    /* Read in what is being received from the ESP8266 */
    if(bytesAvailable){

      /* Reads in all of the data from the server and stores it in the RX Buffer */
      for(int i = 0; i < bytesAvailable; i++){
        rx_buffer[i] = (uint8_t)client.read();
      }
    
      /* deserializing and making sure that deserialization performed as expected */
      if(!deserializeJson(responseJSON, rx_buffer, sizeof(rx_buffer))){
        handleResponse(responseJSON);
      }
    }

    readingNow = !readingNow;                   // flip so that we perform the alternate operation on the next pass
  }
  /* Otherwise we need to attempt a reconnect */
  else{
    Serial.println("Client disconnected");
    connection = false;
  }
}

/**
 * @author Connor Burri
 * @brief handles the response JSON and assignes instructions from the jetson into teensy controls
 */
void handleResponse(DynamicJsonDocument responseJSON){
  /* Read in the state coming from the Jetson */
  String stateString = responseJSON["state"];
  Serial.print("State: ");
  Serial.println(stateString);

  /* Assign the new state */
  if(stateString == "0"){
    state = TrimState_TRIM_STATE_MAX_LIFT_PORT;
  }
  else if(stateString == "1"){
    state = TrimState_TRIM_STATE_MAX_LIFT_STBD;
  }
  else if(stateString == "2"){
    state = TrimState_TRIM_STATE_MAX_DRAG_PORT;
  }
  else if(stateString == "3"){
    state = TrimState_TRIM_STATE_MAX_DRAG_STBD;
  }
  else if(stateString == "4"){
    state = TrimState_TRIM_STATE_MIN_LIFT;
  }
  else if(stateString == "5"){
    state = TrimState_TRIM_STATE_MANUAL;
    String angleString = responseJSON["angle"];
    if(angleString.toInt() != control_angle){
      Serial.print("new angle: ");
      Serial.println(angleString);
    }
    control_angle = angleString.toInt();
  }
}

/** 
 * @author  Connor Burri
 * @brief   writes data in JSON string format to Jetson using ESP8266 interface
 * @bug     JSON objects requires a lot of memory (relative); not a pressing issue but still something to keep in mind
 */
void writeJson(){
  Serial.println("writing...");
  if(client.connected()){
    
    //assigning the data
    transmitJSON["relative_wind_dir"] = analogRead(potPin); //windAngle; 
  
    //string representation of the JsonDocument
    String jsonString;
  
    //serializing the JsonDocument to a JsonString
    bytesSerialized = serializeJsonPretty(transmitJSON, jsonString);
  
    //send the message over the ESP8266
    for(int i = 0; i < (int)jsonString.length(); i++){
      tx_buffer[i] = jsonString.charAt(i);
    }

    //check to make sure the data was sent to the server
    if(!client.write(tx_buffer, bytesSerialized)){
      Serial.println("ESP8266 failed to send data to the Jetson Server");
    }
    else{
      Serial.println(jsonString);
    }
    
    readingNow = !readingNow;                   // flip so that we perform the alternate operation on the next pass
  }
  else{
    Serial.println("Client disconnected");
    connection = false;
  }
}

/**
 * @author Irina Lavryonova
 * @brief establishes a connection between the Teensy and Jetson Nano
 */
void establishConnection()
{
//  /* Checks to see if the teensy recognizes the ESP8266 */
//  if ( == WL_NO_SHIELD)
//  {
//    Serial.println("WiFi shield not present");
//    while (true)                              // don't continue
//      ;
//  }
  
  /* Attempt to connect to WiFi network */
  Serial.print("Attempting to connect to Jetson SSID: ");
  Serial.println(ssid);
  while (status != WL_CONNECTED)
  {
    status = WiFi.begin(ssid, pass);          // attempt connection
  }
  Serial.println("Connected to wifi\n\n");        // administer feedback that we have connected

  /* Attempt to connect to the server being hosted on the Jetson over the WiFi network */
  Serial.println("Starting connection to server");
  if (client.connect(hullIP, port))
  {
    connection = true;
    digitalWrite(wifiLED, HIGH);
    Serial.print("Connected to server: ");
    Serial.println(hullIP);
    Serial.print("Listening on port ");
    Serial.println(port);
  }
  else{
    Serial.println("Failed to connect to server");
    connection = false;
  }
}


/**
 * @author Irina Lavryonova
 * @brief Sets the angle of the servo based on the angle of attack read from the encoder at the front
 * @TODO: send this state to the telemetry interface
 */
bool servoControl(void *)
{
  // Read, format, and scale angle of attack reading from the encoder
  windAngle = analogRead(potPin) - POT_HEADWIND;                                            // reads angle of attack data and centers values on headwind
  windAngle = windAngle < 0 ? POT_HEADWIND + windAngle + (1023 - POT_HEADWIND) : windAngle; // wraps angle around
  windAngle = windAngle / 1023.0 * 360.0;                                             // Convert to degrees, positive when wind from 0-180, negative when wind 180-359
  if(windAngle > 180){
    windAngle = (360 - windAngle) * -1;
  }
  //Serial.println(windAngle);
  
  // Set debug LEDs to on to indicate servo control is active
  digitalWrite(led1Pin, HIGH);
  digitalWrite(led2Pin, HIGH);

  // Write servo position to one read from the Arduino
  //  servo.write(SERVO_CTR + control_angle - 200 - 90);

  switch(state){
    case TrimState_TRIM_STATE_MAX_LIFT_PORT:
      if (MAX_LIFT_ANGLE > windAngle) {
          control_angle+=2;
      }
      else if ((MAX_LIFT_ANGLE < windAngle)) {
          control_angle-=2;
      }
      control_angle = min(max(control_angle,(SERVO_CTR-55)), (SERVO_CTR+55));
      servo.write(control_angle);
      break;
    case TrimState_TRIM_STATE_MAX_LIFT_STBD:
      windAngle*=-1;
      if(MAX_LIFT_ANGLE > windAngle) {
          control_angle-=2;
      }
      else if ((MAX_LIFT_ANGLE < windAngle)) {
          control_angle+=2;
      }
      control_angle = min(max(control_angle,(SERVO_CTR-55)), (SERVO_CTR+55));
      servo.write(control_angle);
      break;
    case TrimState_TRIM_STATE_MAX_DRAG_PORT:
      servo.write(SERVO_CTR - 55);
      break;
    case TrimState_TRIM_STATE_MAX_DRAG_STBD:
      servo.write(SERVO_CTR + 55);
      break;
    case TrimState_TRIM_STATE_MIN_LIFT:
      servo.write(SERVO_CTR);
      break;
    case TrimState_TRIM_STATE_MANUAL:
      servo.write(control_angle);
      break;
    default:
      servo.write(control_angle);
      break;
  }

  return true;
}

/**
 * @author Irina Lavryonova
 * @brief controls the blinking operations within the LEDS
 * @TODO: send this state to the telemetry interface
 */ 
bool blinkState(void *)
{
  // Toggle state
  ledState = ledState == LOW ? HIGH : LOW;

  // Blink WiFi led
  if (!connection)
  {
    digitalWrite(wifiLED, ledState);
  }
  else
  {
    digitalWrite(wifiLED, HIGH);
  }

  switch(state){
    case TrimState_TRIM_STATE_MAX_LIFT_PORT:
      digitalWrite(led1Pin, HIGH);
      digitalWrite(led2Pin, LOW);
      break;
    case TrimState_TRIM_STATE_MAX_LIFT_STBD:
      digitalWrite(led1Pin, ledState);
      digitalWrite(led2Pin, LOW);
      break;
    case TrimState_TRIM_STATE_MAX_DRAG_PORT:
      digitalWrite(led1Pin, LOW);
      digitalWrite(led2Pin, HIGH);
      break;
    case TrimState_TRIM_STATE_MAX_DRAG_STBD:
      digitalWrite(led1Pin, LOW);
      digitalWrite(led2Pin, ledState);
      break;
    case TrimState_TRIM_STATE_MIN_LIFT:
      digitalWrite(led1Pin, LOW);
      digitalWrite(led2Pin, LOW);
      break;
    case TrimState_TRIM_STATE_MANUAL:
      digitalWrite(led1Pin, HIGH);
      digitalWrite(led2Pin, HIGH);
      break;
    default:
      digitalWrite(led1Pin, HIGH);
      digitalWrite(led2Pin, HIGH);
      break;
  }

  return true;
}

/**
 * @author Irina Lavryonova
 * @author Connor Burri
 * @brief Prints the status of the connection between the Teensy and Jetson Nano
 */
void printWifiStatus()
{
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  
  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  
  // print the received signal strength
  printSignalIndication(WiFi.RSSI());
}

/**
 * @author Connor Burri
 * @brief prints the signal strength in a human understandable way
 * @TODO: get this data over to the jetson for telemetry interface
 */
void printSignalIndication(long signalStrength){
  Serial.print("Signal strength (RSSI): ");
  Serial.print(signalStrength);
  if(signalStrength > -30){
    Serial.print(" (Amazing)");
  }
  else if(signalStrength > -67){
    Serial.print(" (Very Good)");
  }
  else if(signalStrength > -70){
    Serial.print(" (Okay)");
  }
  else if(signalStrength > -80){
    Serial.print(" (Not Good)");
  }
  else{
    Serial.print(" (Nearly Unusable)");
  }
  Serial.println(" dBm");
}
