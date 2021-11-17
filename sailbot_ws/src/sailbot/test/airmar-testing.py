import serial
import json
import time


class AirmarReader():

    def __init__(self):
        self.ser = serial.Serial('COM5')
        self.vals = {}

    def updateVals(self, update):
        for i in update:
            self.vals[i] = update[i]


    def readAndUpdate(self):
        new = self.readLineToJson()
        self.updateVals(new)
        
    def readLineToJson(self):

        try:
            line = self.ser.readline().decode()
            tag = line.split(',',1)[0]
            type_code = tag[-3:]
            args = line.split(',')
            args[len(args) - 1] = args[len(args) - 1].split('*')[0] #get rid of checksum

            if(type_code == 'ROT'): #rate of turn degrees per minute. negative is to port
                return {"rate-of-turn":args[1]}
            elif(type_code == 'GLL'):
                #convert from degree decimal minutes to decimal degrees
                #dd = d + m/60
                #lat = math.floor(float(args[1]) / 100) + (float(args[1]) % 100)/60.0
                #lon = math.floor(float(args[3]) / 100) + (float(args[3]) % 100)/60.0
                lat_raw = args[1]
                lat = float(lat_raw[:2]) + float(lat_raw[2:])/60.0
                lon_raw = args[3]
                lon = float(lon_raw[:3]) + float(lon_raw[3:])/60.0
                return {"Latitude":lat, 
                        "Latitude-direction":args[2],
                        "Longitude":lon,
                        "Longitude-direction":args[4]}
            elif(type_code == 'VTG'):
                return {"track-degrees-true":args[1],
                        "track-degrees-magnetic":args[3],
                        "speed-knots":args[5],
                        "speed-kmh":args[7]}
            elif(type_code == 'XDR'):
                ret = {}
                if(args[4] == "ENV_OUTSIDE_T"): #in celcius
                    ret["outside-temp"] = args[2]
                elif(args[4] == "ENV_ATMOS_P"): #in ???
                    ret["atmospheric-pressure"] = args[2]
                if(len(args) > 5):
                    if(args[8] == "ENV_OUTSIDE_T"): #in celcius
                        ret["outside-temp"] = args[6]
                    elif(args[8] == "ENV_ATMOS_P"): #in ???
                        ret["atmospheric-pressure"] = args[6]

                return ret

            elif(type_code == 'HDG'):
                return {"magnetic-sensor-heading":args[1], #degrees
                        "magnetic-deviation":args[2], #degrees
                        "magnetic-deviation-direction":args[3],
                        "magnetic-variation":args[4], #degrees
                        "magnetic-variation-direction":args[5]}

            elif(type_code == 'VHW'): #water speed and direction (speed not showing up)
                return {}  #not sure we need
            elif(type_code == 'GGA'): #GPS position, quality, elevation, # of satilites
                return {} #not sure needed, repeated gps from GLL
            elif(type_code == 'DTM'): #unreferenced in documentation except as datnum reference -- unusable
                return {}
            elif(type_code == 'GSV'): #GPS satilites in view
                return {} #no need for this data
            elif(type_code == 'GSA'): # GPS Dilution of Precision
                return {} #not sure if needed
            elif(type_code == 'GRS'): #"The GRS message is used to support the Receiver Autonomous Integrity Monitoring (RAIM)." -- unneeded
                return {}
            elif(type_code == 'MWD'): 
                return {"wind-angle-true":args[1], # in degrees
                        "wind-speed-true-knots":args[5],
                        "wind-speed-true-meters":args[7]} #in m/s
            elif(type_code == 'MWV'):
                return {"wind-angle-relative":args[1], # in degrees
                        "wind-speed-relative-meters":args[3]} #in m/s
            elif(type_code == 'ZDA'): #date & time
                return {} # unneeded
            elif(type_code == 'OUT'): #real key is 'PMAROUT', shortened to OUT, since all others are 3 letters
                #"PGN is translated to a Maretron proprietary NMEA 0183 sentence " -- used for pitch and roll
                return {"roll":args[2],
                        "pitch":args[3]}
            else:
                raise ValueError("Unknown NEMA code: " + type_code)
        except Exception as e:
            print(e);
            return({})

    def printAll(self):
        keys = """rate-of-turn
latitude
latitude-direction
longitude
longitude-direction
track-degrees-true
track-degrees-magnetic
speed-knots
speed-kmh
outside-temp
atmospheric-pressure
magnetic-sensor-heading
magnetic-deviation
magnetic-deviation-direction
magnetic-variation
magnetic-variation-direction
wind-angle-true
wind-speed-true-knots
wind-speed-true-meters
wind-angle-relative
wind-speed-relative-meters
roll
pitch"""

        toPrint = ""
        
        for i in keys.split("\n"):

            try:
                toPrint += i + ": " + self.vals[i] + "\n"
            except Exception:
                pass

        print(toPrint)
        #time.sleep(.2)
       
            

    

def main(args=None):
    print("begining test")

    airmar_reader = AirmarReader()
    
    while(True):
        airmar_reader.readAndUpdate()
        airmar_reader.printAll()

if __name__ == '__main__':
    main()


