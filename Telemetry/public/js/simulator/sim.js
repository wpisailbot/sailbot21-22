const socket = io(); // socket initialization

let counter = 0;
let dataInterval;

// let mockCoords = [{ lat: 42.849810669147935, lng: -70.98818573987138 }];
// const mockCoords = [{ lat: 42.849810669147935, lng: -70.98818573987138 }, { lat: 42.84892970142418, lng: -70.98767075574052 }, { lat: 42.8486465305588, lng: -70.98732743298662 }, { lat: 42.84828469923054, lng: -70.98694119488847 }, { lat: 42.848095916826544, lng: -70.98651204144609 }, { lat: 42.84770261829851, lng: -70.98599705731523 }, { lat: 42.84735651352237, lng: -70.98522458111894 }, { lat: 42.84730931726628, lng: -70.9846237662996 }, { lat: 42.84730931726628, lng: -70.98404440915239 }, { lat: 42.84759249426192, lng: -70.98376545941484 }, { lat: 42.84778127820445, lng: -70.98303589856279 }, { lat: 42.84797006157004, lng: -70.98232779538286 }, { lat: 42.84801725732128, lng: -70.98189864194048 }, { lat: 42.84817457623167, lng: -70.9811476234163 }, { lat: 42.848111648715594, lng: -70.98050389325273 }, { lat: 42.84768688630529, lng: -70.97986016308916 }, { lat: 42.84730931726628, lng: -70.98016057049882 }, { lat: 42.84707333544494, lng: -70.98061118161333 }, { lat: 42.84723065675932, lng: -70.9811476234163 }, { lat: 42.84771835028771, lng: -70.9817055228914 }, { lat: 42.848095916826544, lng: -70.98176989590776 }, { lat: 42.848851042980684, lng: -70.98187718426836 }, { lat: 42.8492443341965, lng: -70.98207030331743 }, { lat: 42.84955896536614, lng: -70.98217759167802 }, { lat: 42.85006237190399, lng: -70.98239216839922 }, { lat: 42.850298342306246, lng: -70.98307881390703 }, { lat: 42.84990505780166, lng: -70.9837225440706 }, { lat: 42.84951177079288, lng: -70.98355088269365 }, { lat: 42.849291528974135, lng: -70.982714033481 }, { lat: 42.84979493769163, lng: -70.98179135357988 }, { lat: 42.850298342306246, lng: -70.98196301495683 }, { lat: 42.85047138669493, lng: -70.98252091443193 }, { lat: 42.85040846151878, lng: -70.98340067898882 }, { lat: 42.85010956605655, lng: -70.9839371207918 }, { lat: 42.84914994453309, lng: -70.98413023984087 }, { lat: 42.84820603996566, lng: -70.98410878216875 }, { lat: 42.847844206057076, lng: -70.98404440915239 }, { lat: 42.847576762240664, lng: -70.9842804435457 }, { lat: 42.84708906759442, lng: -70.9852681344105 }, { lat: 42.847199192528485, lng: -70.98589040690196 }, { lat: 42.847623958292395, lng: -70.98640539103282 }, { lat: 42.84834762657032, lng: -70.98704912119639 }, { lat: 42.848851042980684, lng: -70.98745681696666 }, { lat: 42.849291528974135, lng: -70.98780013972056 }, { lat: 42.84971628034997, lng: -70.9881863778187 }];

// hardcoded waypoints
let waypoints = [
    { x: 3, y: 6, lat: 42.84780, lng: -70.9849 }, 
    { x: 7, y: 6, lat: 42.84780, lng: -70.9809 }, 
    { x: 5, y: 4, lat: 42.8499, lng: -70.9829 },
    { x: 0, y: 4, lat: 42.84964938719152, lng: -70.98783200038395 },
];

// hardcoded values for testing
let boatStart = [{ x: 0, y: 4, weight: 0, lat: 42.84964938719152, lng: -70.98783200038395 }];
const windDirection = 135;
const maxX = 12;
const maxY = 8;
let boatWaypoints = [];

while (waypoints.length > 0){
    let tempDest = waypoints.shift();
    console.log(boatStart[0], tempDest);
    let path = runAstar(boatStart, tempDest).reverse()
    waypoints.length > 1 ? path.pop() : undefined;
    boatWaypoints.push(...path);
    boatStart = [{ weight: 0, ...tempDest }]; // has to be in an array so it looks like a path
    console.log(boatWaypoints);
}

// getting intermediate points
// let mockCoords = [];
// for (let i = 0; i < boatWaypoints.length - 1; i++){
//     let start = boatWaypoints[i];
//     let end = boatWaypoints[i + 1];
//     for (let j = 0; j < 3; j++){
//         let latdiff = start.lat + j * (end.lat - start.lat);
//         let lngdiff = start.lng + j * (end.lng - start.lng);
//         mockCoords.push({x: end.x, y: end.y, lat: latdiff, lng: lngdiff});
//     }
// }
// console.log(mockCoords);


// sends data every 500ms
const sendMockMessages = () => {
	// timeout variable holds the timeout
	dataInterval = setInterval(() => {socket.emit('data', {
			apparentWind: {speed: 10 + Math.floor(Math.random() * 50), direction: Math.floor(Math.random() * 295)},
			theoreticalWind: {speed: 10 + Math.floor(Math.random() * 50), direction: Math.floor(Math.random() * 295)},
			compass: {x: Math.floor(Math.random() * 360), y: Math.floor(Math.random() * 360), z: 'garbo'}, 
			airtemp: Math.floor(Math.random() * 35),
			windchill: Math.floor(Math.random() * 35),
			pressure: 950 + Math.floor(Math.random() * 100),
			groundspeed: Math.floor(Math.random() * 25),
			gps: {latitude: boatWaypoints[counter].lat, longitude: boatWaypoints[counter].lng}, 
			pitchroll: {pitch: Math.floor(Math.random() * 20) - 20, roll: Math.floor(Math.random() * 180) - 90},
			gyro: {phi: Math.floor(Math.random() * 100000)/1000, theta: Math.floor(Math.random() * 100000)/1000, psi: Math.floor(Math.random() * 100000)/1000}
		});
			
		counter++;
		console.log(boatWaypoints[counter].lat +' '+ boatWaypoints[counter].lng);
	}, 1000);

	document.querySelector('#message').innerHTML = 'Sending...';
	document.querySelector('#but').innerHTML = 'Stop Sending';
	document.querySelector('#but').onclick = stopMessages;
}

const stopMessages = () => {
	clearInterval(dataInterval);
	document.querySelector('#but').onclick = sendMockMessages;
	document.querySelector('#message').innerHTML = 'Click the button to start sending messages!';
	document.querySelector('#but').innerHTML = 'Send';
}

const initSliders = () => {
	var sliderSimple = d3
		.sliderBottom()
		// .min()
		// .max(d3.max([50]))
		.width(300)
		// .tickFormat(d3.format(''))
		.ticks(5)
		.default(5)
		.on('onchange', val => {
		  d3.select('p#value-simple').text(d3.format('.2')(val));
		});

	var gSimple = d3
		.select('div#slider-simple')
		.append('svg')
		.attr('width', 500)
		.attr('height', 100)
		.append('g')
		.attr('transform', 'translate(30,30)');
		gSimple.call(sliderSimple);
		d3.select('p#value-simple').text(d3.format('.2')(sliderSimple.value()));
} 

window.onload = () => {
	initSliders();
};

