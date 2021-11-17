// Create the svg using d3 and inserts it into the given div
const displayCompass = (div) => {
	d3.select('#' + div)
		.append('svg')
		.attr('id', 'svg-' + div)
        .attr('viewport', '0 0 350 350')
        .attr('height', '100px')
        .attr('width', '100px')
        .attr({'xmlns': 'http://www.w3.org/2000/svg','xmlns:xlink': 'http://www.w3.org/1999/xlink'});
    
    let svg = d3.select('#svg-' + div);

    svg.append('circle')
    	.attr('cx', '50px')
    	.attr('cy', '50px')
    	.attr('r', '48')
    	.attr('stroke', 'black')
    	.attr('stroke-width', '1')
    	.attr('fill', 'white')

    svg.append('image')
    	.attr('id', 'compassBoat')
    	.attr('href', '../assets/boat.png')
    	.attr('transform', 'translate(17, 16) scale(0.30)');
};