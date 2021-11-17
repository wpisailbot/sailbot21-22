const socket = io(); 

// inits the socket connection and joins the room for client in the server, also creates an event handler
// 	for when data is recieved

const socketInit = () => {
    const ease = d3.easeElasticOut;
    
	
    socket.emit('client');

    // Creates callback for when data is recieved from the server (updates all the page components)
	socket.on('updateAirmarDash', (data) => {
    /********** Apparent Wind **********/

	// 	let appSpeed = (data['wind-speed-relative-meters'] ? data['wind-speed-relative-meters'] : 60);
	// 	let appDirection = (data['wind-angle-relative'] ? data['wind-angle-relative'] : 0);
 //        let appOldAngle = document.querySelector('#apparentWindVectorLine').transform.baseVal[0].angle;
	// 	let appX = appSpeed * Math.cos(0 * (Math.PI / 180));
	// 	let appY = appSpeed * Math.sin(0 * (Math.PI / 180));
        
 //        // sets the correct Length of the Vector at the 0 angle
	//   	d3.select('#apparentWindVectorLine')
	//     	.attr('x1', (30 - appX/2).toString())
	//     	.attr('y1', (30 - appY/2).toString())
	//     	.attr('x2', (30 + appX/2).toString())
	//     	.attr('y2', (30 + appY/2).toString());

	//     // Sets the trnsition from the vector's oldAngle to the new angle
	//   	d3.select('#apparentWindVectorLine')
 //            .transition()
 //            .duration(1000)
 //            .ease(d3.easeElasticOut, 1, 0.9)
 //            .attrTween("transform", () => d3.interpolateString('rotate('+ appOldAngle +', 30, 30)', 'rotate('+ -appDirection +', 30, 30)'));

 //        // d3.select('#apparentWindAngle')
 //        //     .transition()
 //        //     .duration(1000)
 //        //     .ease(d3.easeElasticOut,1,0.9)
 //        //     .tween("text", (d) => {
 //        //         var i = d3.interpolateString(d3.select('#apparentWindAngle').text(), data.appDirection)
 //        //         return (t) => Math.round(i(ts));
 //        //     });

	// 	document.querySelector('#apparentWindAngle').innerHTML = appDirection;
	// 	document.querySelector('#apparentWindMag').innerHTML = appSpeed;

	// /********** Theoretical Wind **********/

 //        // let theoSpeed = (data.theoreticalWind.speed ? data.theoreticalWind.speed : 60);
 //        // let theoDirection = (data.theoreticalWind.direction ? data.theoreticalWind.direction : 0);
 //        // let theoOldAngle = document.querySelector('#theoreticalWindVectorLine').transform.baseVal[0].angle;
 //        // let theoX = theoSpeed * Math.cos(0 * (Math.PI / 180));
 //        // let theoY = theoSpeed * Math.sin(0 * (Math.PI / 180));
        
 //        // // sets the correct Length of the Vector at the 0 angle
 //        // d3.select('#theoreticalWindVectorLine')
 //        //     .attr('x1', (30 - theoX/2).toString())
 //        //     .attr('y1', (30 - theoY/2).toString())
 //        //     .attr('x2', (30 + theoX/2).toString())
 //        //     .attr('y2', (30 + theoY/2).toString());

 //        // // Sets the trnsition from the vector's oldAngle to the new angle
 //        // d3.select('#theoreticalWindVectorLine')
 //        //     .transition()
 //        //     .duration(1000)
 //        //     .ease(d3.easeElasticOut, 1, 0.9)
 //        //     .attrTween("transform", () => d3.interpolateString('rotate('+ theoOldAngle +', 30, 30)', 'rotate('+ -theoDirection +', 30, 30)'));

 //        // document.querySelector('#theoreticalWindAngle').innerHTML = theoDirection;
 //        // document.querySelector('#theoreticalWindMag').innerHTML = theoSpeed;

	// /********** Compass **********/ 

	// 	// .attr('transform', 'rotate(' + Math.atan2(data.compass.y, data.compass.x) * (180/Math.PI) + ', 50, 50) translate(17, 16) scale(0.30)');
	// // d3.select('#compassBoat')
 // //            .transition()
 // //            .duration(1000)
 // //            .ease(d3.easeElasticOut, 1, 0.9)
 // //            .attrTween("transform", () => d3.interpolateString('rotate('+ document.querySelector('#compassBoat').transform.baseVal[0].angle +', 50, 50) translate(17, 16) scale(0.30)', 'rotate('+ data['magnetic-sensor-heading'] * (180/Math.PI) +', 50, 50) translate(17, 16) scale(0.30)'));
 //            // .attrTween("transform", () => d3.interpolateString('rotate('+ document.querySelector('#compassBoat').transform.baseVal[0].angle +', 50, 50) translate(17, 16) scale(0.30)', 'rotate('+ -Math.atan2(data.compass.y, data.compass.x) * (180/Math.PI) +', 50, 50) translate(17, 16) scale(0.30)'));

	// /*** Air Temp **********/

	//    // airtemp.updateGauge(data['outside-temp'] ? data['outside-temp'] : 0);

	// /*** Wind Chill **********/

 //    	// windchill.updateGauge(data.windchill ? data.windchill : 0);

	// /********** Barometric Pressure **********/

	//    // pressure.updateGauge(data.pressure ? data.pressure : 950);
	

	// /********** Pitch and Roll **********/

	// 	// .attr('transform', 'rotate('+ (data.pitchroll.roll ? data.pitchroll.roll : 30) +' 65, 65)');
 //        // d3.select('#compassBoat')
 //    	d3.select('#rollIndicator')
 //            .transition()
 //            .duration(1000)
 //            .ease(d3.easeElasticOut, 1, 0.9)
 //            .attrTween("transform", () => d3.interpolateString('rotate('+ document.querySelector('#rollIndicator').transform.baseVal[0].angle +', 65, 65)', 'rotate('+ (data.roll ? data.roll : 30) +' 65, 65)')); 
 //        d3.select('#pitchIndicator')
 //            .transition()
 //            .duration(1000)
 //            .ease(d3.easeElasticOut, 1, 0.9)
 //            .attrTween("transform", () => d3.interpolateString('translate(0, '+ (document.querySelector('#pitchIndicator').transform.baseVal[0].matrix.f) +')', 'translate(0, '+ (data.pitch ? data.pitch : 0) +')'));
 //    	// d3.select('#pitchIndicator')
 //    	// 	.attr('transform', 'translate(0, '+ (data.pitchroll.pitch ? data.pitchroll.pitch : 0) +')');

	// /********** Ground Speed **********/

 //    	groundspeed.updateGauge(data['speed-kmh'] ? data['speed-kmh'] : 0);
    	
	// /********** Rate Gyro **********/

 //    	// document.querySelector('#phi').innerHTML = data.gyro.phi ? data.gyro.phi : 0;
 //    	// document.querySelector('#theta').innerHTML = data.gyro.theta ? data.gyro.theta : 0;
 //    	// document.querySelector('#psi').innerHTML = data.gyro.psi ? data.gyro.psi : 0;

	/********** Relative Humidity **********/

    	// document.querySelector('#humidityVal').innerHTML = (data.groundspeed ? data.groundspeed : 0) + '%';

	/********** GPS **********/

        if (data.hasOwnProperty('Longitude') || data.hasOwnProperty('Latitude'))
            // console.log(data.Latitude, data.Longitude);
            boatPath.getPath().push(new google.maps.LatLng(data['Latitude'], data['Longitude']));


	});
    
    socket.on('updateTrimDash', (data) => {
        
    });

    socket.on('updateSerialControls', (data) => {
        
    });
};
