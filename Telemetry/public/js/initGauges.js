// uses dependecy (d3gauge.js) to draw all of the gauges
const gaugeInit = () => {
    // default options
	const options = {gaugeRadius: 65,
					edgeWidth: .025, 
					tickLengthMin: 0, 
					needleLengthNeg: -0.25, 
					pivotRadius: 0,
					tickEdgeGap: -.1,
					tickFont: "'Lato', sans-serif", 
					unitsFont: "'Lato', sans-serif",
					labelFontSize: 22, 
					outerEdgeCol: 'black',
					tickColMaj: 'black',
					needleCol: 'red'};

	// airtemp = new drawGauge({divID: 'airtempDisp', 
	// 						minVal: 0, 
	// 						maxVal: 35, 
	// 						needleVal: 0,
	// 						tickSpaceMajVal: 5, 
	// 						gaugeUnits: 'Celcius', 
	// 						...options});
	// windchill = new drawGauge({divID: 'windchillDisp', 
	// 						minVal: 0, 
	// 						maxVal: 35, 
	// 						needleVal: 0,
	// 						tickSpaceMajVal: 5, 
	// 						gaugeUnits: 'Celcius', 
	// 						...options});
	// pressure = new drawGauge({divID: 'pressureDisp', 
	// 						minVal: 950, 
	// 						maxVal: 1050, 
	// 						needleVal: 950,
	// 						tickSpaceMajVal: 15, 
	// 						gaugeUnits: 'MiliBars', 
	// 						...options});

    coursetrack = new drawGauge({divID: 'coursetrackDisp', 
                            minVal: 0, 
                            maxVal: 360, 
                            needleVal: 0,
                            tickSpaceMajVal: 30, 
                            gaugeUnits: '°', 
                            ...options})

	groundspeed = new drawGauge({divID: 'speedDisp', 
							minVal: 0, 
							maxVal: 25, 
							needleVal: 0,
							tickSpaceMajVal: 5, 
							gaugeUnits: 'Knots', 
							...options});

};