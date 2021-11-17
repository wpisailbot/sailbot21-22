// creates the pitchroll svg component using d3 and inserts into the given div
const displayPitchRoll = (div, radius) => {
	d3.select('#' + div)
		.append('svg')
		.attr('id', 'svg-' + div)
        .attr('viewport', '0 0 350 350')
        .attr('height', '130px')
        .attr('width', '130px')
        .attr({'xmlns': 'http://www.w3.org/2000/svg','xmlns:xlink': 'http://www.w3.org/1999/xlink'});

    let svg = d3.select('#svg-' + div);

    let borders = svg.append('g')
        	.attr('id','borders');

    // Outer Circle
    borders.append('circle')
        .attr('cx', radius)
        .attr('cy', radius)
        .attr('r', radius - 1)
        .style('fill', 'white')
        .style('stroke', 'black');
    // Inner Circle
    borders.append('circle')
        .attr('cx', radius)
        .attr('cy', radius)
        .attr('r', radius * 0.7)
        .style('fill', 'white')
        .style('stroke', 'black');

    // Horizontal Ticks
    let horizonTicks = svg.append('g')
    	.attr('id', 'horizonTicks');

    /************************\

    TODO: TUNE THE locations of the lines and ticks SO ITS ACTUALLY CORRECT 
    (PROBABLY 0 horizon OR SOMETHING FOR A BOAT)

    \*************************/
    // long Horizon line
    horizonTicks.append('line')
    	.attr('id', 'horizon')
    	.attr('x1', radius - (radius * 0.92))
    	.attr('y1', radius * 1.3)
    	.attr('x2', radius + (radius * 0.92))
    	.attr('y2', radius * 1.3)
    	.attr('stroke', 'black')
    	.attr('stroke-width', 1);

    // normal pitch ticks
    horizonTicks.append('line')
    	.attr('id', '5degrees')
    	.attr('x1', radius - (radius * 0.2))
    	.attr('y1', radius * 1.11)
    	.attr('x2', radius + (radius * 0.2))
    	.attr('y2', radius * 1.11)
    	.attr('stroke', 'black')
    	.attr('stroke-width', 1);
    horizonTicks.append('line')
    	.attr('id', '10degrees')
    	.attr('x1', radius - (radius * 0.2))
    	.attr('y1', radius * 0.91)
    	.attr('x2', radius + (radius * 0.2))
    	.attr('y2', radius * 0.91)
    	.attr('stroke', 'black')
    	.attr('stroke-width', 1);
    horizonTicks.append('line')
    	.attr('id', '15degrees')
    	.attr('x1', radius - (radius * 0.2))
    	.attr('y1', radius * 0.71)
    	.attr('x2', radius + (radius * 0.2))
    	.attr('y2', radius * 0.71)
    	.attr('stroke', 'black')
    	.attr('stroke-width', 1);

    // Roll ticks around the edges (corrected to the angle)
    let rollTicks = svg.append('g')
    	.attr('id', 'rollTicks');
    let largeTickLen = (radius*0.3 - 1), smallTickLen = (radius*0.3 * 0.5), innerRad = radius * 0.7, center = radius;
    let angle, sin, cos;
    // A LOT OF TRIG WENT INTO FINDING THESE X'S AND Y'S

    angle = 15, sin = Math.sin(angle * Math.PI/180), cos = Math.cos(angle * Math.PI/180);
    rollTicks.append('line')
    	.attr('id', '15degrees')
    	.attr('x1', center + innerRad * sin)
    	.attr('y1', center - innerRad * cos)
    	.attr('x2', (center + innerRad * sin) + smallTickLen * sin)
    	.attr('y2', (center - innerRad * cos) - smallTickLen * cos)
    	.attr('stroke', 'black')
    	.attr('stroke-width', 1);
    angle = -15, sin = Math.sin(angle * Math.PI/180), cos = Math.cos(angle * Math.PI/180);
    rollTicks.append('line')
    	.attr('id', '-15degrees')
    	.attr('x1', center + innerRad * sin)
    	.attr('y1', center - innerRad * cos)
    	.attr('x2', (center + innerRad * sin) + smallTickLen * sin)
    	.attr('y2', (center - innerRad * cos) - smallTickLen * cos)
    	.attr('stroke', 'black')
    	.attr('stroke-width', 1);

    angle = 25, sin = Math.sin(angle * Math.PI/180), cos = Math.cos(angle * Math.PI/180);
    rollTicks.append('line')
    	.attr('id', '25degrees')
    	.attr('x1', center + innerRad * sin)
    	.attr('y1', center - innerRad * cos)
    	.attr('x2', (center + innerRad * sin) + smallTickLen * sin)
    	.attr('y2', (center - innerRad * cos) - smallTickLen * cos)
    	.attr('stroke', 'black')
    	.attr('stroke-width', 1);
    angle = -25, sin = Math.sin(angle * Math.PI/180), cos = Math.cos(angle * Math.PI/180);
    rollTicks.append('line')
    	.attr('id', '-25degrees')
    	.attr('x1', center + innerRad * sin)
    	.attr('y1', center - innerRad * cos)
    	.attr('x2', (center + innerRad * sin) + smallTickLen * sin)
    	.attr('y2', (center - innerRad * cos) - smallTickLen * cos)
    	.attr('stroke', 'black')
    	.attr('stroke-width', 1);

    angle = 45, sin = Math.sin(angle * Math.PI/180), cos = Math.cos(angle * Math.PI/180);
    rollTicks.append('line')
    	.attr('id', '45degrees')
    	.attr('x1', center + innerRad * sin)
    	.attr('y1', center - innerRad * cos)
    	.attr('x2', (center + innerRad * sin) + largeTickLen * sin)
    	.attr('y2', (center - innerRad * cos) - largeTickLen * cos)
    	.attr('stroke', 'black')
    	.attr('stroke-width', 3);
    angle = -45, sin = Math.sin(angle * Math.PI/180), cos = Math.cos(angle * Math.PI/180);
    rollTicks.append('line')
    	.attr('id', '-45degrees')
    	.attr('x1', center + innerRad * sin)
    	.attr('y1', center - innerRad * cos)
    	.attr('x2', (center + innerRad * sin) + largeTickLen * sin)
    	.attr('y2', (center - innerRad * cos) - largeTickLen * cos)
    	.attr('stroke', 'black')
    	.attr('stroke-width', 3);

    angle = 65, sin = Math.sin(angle * Math.PI/180), cos = Math.cos(angle * Math.PI/180);
    rollTicks.append('line')
    	.attr('id', '45degrees')
    	.attr('x1', center + innerRad * sin)
    	.attr('y1', center - innerRad * cos)
    	.attr('x2', (center + innerRad * sin) + largeTickLen * sin)
    	.attr('y2', (center - innerRad * cos) - largeTickLen * cos)
    	.attr('stroke', 'black')
    	.attr('stroke-width', 3);
    angle = -65, sin = Math.sin(angle * Math.PI/180), cos = Math.cos(angle * Math.PI/180);
    rollTicks.append('line')
    	.attr('id', '-45degrees')
    	.attr('x1', center + innerRad * sin)
    	.attr('y1', center - innerRad * cos)
    	.attr('x2', (center + innerRad * sin) + largeTickLen * sin)
    	.attr('y2', (center - innerRad * cos) - largeTickLen * cos)
    	.attr('stroke', 'black')
    	.attr('stroke-width', 3);

    angle = 90, sin = Math.sin(angle * Math.PI/180), cos = Math.cos(angle * Math.PI/180);
    rollTicks.append('line')
    	.attr('id', '45degrees')
    	.attr('x1', center + innerRad * sin)
    	.attr('y1', center - innerRad * cos)
    	.attr('x2', (center + innerRad * sin) + largeTickLen * sin)
    	.attr('y2', (center - innerRad * cos) - largeTickLen * cos)
    	.attr('stroke', 'black')
    	.attr('stroke-width', 3);
    angle = -90, sin = Math.sin(angle * Math.PI/180), cos = Math.cos(angle * Math.PI/180);
    rollTicks.append('line')
    	.attr('id', '-45degrees')
    	.attr('x1', center + innerRad * sin)
    	.attr('y1', center - innerRad * cos)
    	.attr('x2', (center + innerRad * sin) + largeTickLen * sin)
    	.attr('y2', (center - innerRad * cos) - largeTickLen * cos)
    	.attr('stroke', 'black')
    	.attr('stroke-width', 3);

    /**** TODO: ADD TICK LABELS TO THE PITCHROLL COMPONENT *****/
    // let tickLabels = svg.append('g')
    // 	.attr('id', 'tickLabels');
    // let angles = [15, 25, 45, 65, 90];

    // angles.forEach((angle) => {
	   //  let sin = Math.sin(angle * Math.PI/180), cos = Math.cos(angle * Math.PI/180);
	   //  tickLabels.append('text')
	   //  	.attr('x', 48 + 52*sin)
	   //  	.attr('y', 14 + 20/cos)
	   //      .style("font-weight", "300")
	   //      .style("font-size", "10px")
	   //      .attr("font-family", 'Lato')
	   //      .text(angle);

    // });

    // Top triangle thing in the roll part
    let centerRoll = svg.append('g')
    	.attr('id', 'centerRoll');

    centerRoll.append('polygon')
    	.attr('points', radius +','+ radius*0.3 +' '+ radius*1.15 +','+ radius*0.03 +' '+ radius*0.85 +','+ radius*0.03) //65,20 75,2 55,2 - rad 65
    	.attr('style', 'fill:DarkOrchid;');


    let movingParts = svg.append('g')
    	.attr('id', 'movingParts');

    // purple roll indicator (smaller triangle)
    movingParts.append('polygon')
    	.attr('id', 'rollIndicator')
    	.attr('points', radius +','+ radius*0.3 +' '+ radius*1.077 +','+ radius*0.46 +' '+ radius*0.92 +','+ radius*0.46) //65,20 70,30 60,30 - rad 65
    	.attr('style', 'fill:DarkOrchid;')
        .attr('transform', 'translate(0, 0)');

    let pitchIndicator = movingParts.append('g')
    	.attr('id', 'pitchIndicator')
        .attr('transform', 'translate(0, 0)');

    // path and circle code for the pitch indicator thingy
    pitchIndicator.append('path')
    	.attr('d', 'm'+ radius*0.46 +','+ radius*1.11 +' h'+ radius*0.23 +' a1,1 0 0,0 '+ radius*0.6 +',0 h'+ radius*0.23) //m30,72 h15 a1,1 0 0,0 39,0 h15 - rad65
    	.attr('fill', 'none')
    	.attr('stroke', 'black');
    pitchIndicator.append('circle')
        .attr('id', 'pitchCircle')
    	.attr('cx', radius)
    	.attr('cy', radius * 1.138)
    	.attr('r', 2)
    	.attr('stroke', 'black')
    	.attr('stroke-width', 1);
};