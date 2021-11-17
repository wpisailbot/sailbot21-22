# Telemetry

This is the Land Code for the for the Telemetry

## Usage

### Server and Clients
```bash
npm start
```
Navigate to [localhost:3000](http://localhost:3000) for the client dashboard

For mock data to be sent to all clients, open a new tab to [localhost:3000/boat.html](http://localhost:3000/boat.html) and close this tab when mock data should be stopped

For the mock heroku app, navigate to http://sailbot2021.herokuapp.com/ for the normal dashboard and http://sailbot2021.herokuapp.com/boat.html to start sending data to the dashboards that are open

### Telemetry Networking Setup
#### Cabling
Both Devices must have ethernet power splitters, there is a 12 volt barrel jack plugged into one side and the ethernet into the other

#### Network Configuration
Must set all of the correct ip address
- Windows: Change using the [changer_digi program](http://ftp1.digi.com/support/utilities/changer_digi.zip) program for both to have a 255.255.255.0 subnet mask and the correct ip, and you must go to the network and sharing center, select the ethernet network interface and set all of the IPv4 addresses to 192.168.17.18 (there are 2 places to do this and you might have to do both and restart, I don't know why it worked like this)
- Jetson: set the ip and mask to the correct ones (mask the same ip is 192.168.17.17) "sudo ifconfig eth0 192.168.17.17 netmask 255.255.255.0 up"

##### Testing
- Just "ping 192.168.17.{17 or 18}" from bash or unix to see if they can hit eachother
- To test the server, you can just curl the host ip and port so: "curl 192.168.17.18:3000"


## Descriptions

The purpose of this is to display the Relevant statistics from the sailbot on a land Computer. The Sailbot and computer should be connected to same network, possibly through the telemetry modules, and then the Sailbot should send jsons to the server into the **data** socket, using **socket.io** in the format listed below:
```json
{
	"apparentWind": {"speed": 0 - 55, "direction": 0 - 360},
	"theoreticalWind": {"speed": 0 - 55, "direction": 0 0- 360},
	"compass": {"x": 0 - 360, "y": 0 - 360, "z": null}, 
	"airtemp": 0 - 35,
	"windchill": 0 - 35,
	"pressure": 950 - 1050,
	"groundspeed": 0 - 25,
	"gps": {"latitude": "float", "longitude": "float", "altitude": "float"},
	"pitchroll": {"pitch": (-20) - 20, "roll": (-90) - 90},
	"gyro": {"phi": "float", "theta": "float", "psi": "float"}
}
```


