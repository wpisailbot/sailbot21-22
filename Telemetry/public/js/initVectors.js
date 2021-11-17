// creates a vector component a nd places into the given div using d3
const displayVector = (div) => {
	d3.select('#' + div)
		.append('svg')
		.attr('id', 'svg-' + div)
        .attr('viewport', '0 0 350 350')
        .attr('height', '60px')
        .attr('width', '60px')
        .attr({'xmlns': 'http://www.w3.org/2000/svg','xmlns:xlink': 'http://www.w3.org/1999/xlink'});
    
    let svg = d3.select('#svg-' + div);

    // add marker definitions
    svg.append('defs')
    	.append('marker')
    	.attr('id', 'arrowhead')
    	.attr('markerWidth', '10')
    	.attr('markerHeight', '7')
    	.attr('refX', '0')
    	.attr('refY', '3.5')
    	.attr('orient', 'auto')
    	.append('polygon')
    	.attr('points', '0 0, 10 3.5, 0 7');

    // add vector line and stuff
    let appwind = svg.append('line')
    	.attr('id', div + 'Line')
    	.attr('x1', '30')
    	.attr('y1', '60')
    	.attr('x2', '30')
    	.attr('y2', '10')
    	.attr('transform', 'rotate(0, 30, 30)')
    	.attr('stroke', '#000')
    	.attr('stroke-width', '1')
    	.attr('marker-end', 'url(#arrowhead)');

    // add middle circle for vector
    svg.append('circle')
    	.attr('cx', '30px')
    	.attr('cy', '30px')
    	.attr('r', '2');
};