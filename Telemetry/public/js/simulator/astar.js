// values that must be tuned
const currentDirectionWeight = 20;
const nodeWeightMultiplier = 10000;

//globals
let destination;
let lowestPath = [];
let pathsToCheck = [];
let lowestPathWeight = 999999999;


/*
* Heuristic stuff:
* 	- node weight: will be total distance from current node to the destination
* 	- edge weight: will be current direction of travel + current wind direction vs edge direction
* 
* Algo Stuff:
* 	- F(n) = g(n) + h(n)
* 		- g(n) = edge weight
* 		- h(n) = dest node weight
* 	
* 	- Evaluate all pathsToCheck from a node, then start continuing on the lowest cost one
* 	- once you find the final node, keep going through your list until the value you have is lower than
* 		the top of the list
* 
*/
const runAstar = (start, dest) => {
	destination = dest; // pops the first element
	console.log(gridCenters); // holds all the points, and their coords
	resetPaths(start);
	let counter = 0
	// console.log(start, pathsToCheck);
	while (pathsToCheck.length > 0 && pathsToCheck[0][0].weight <= lowestPathWeight && counter < 500){
		findCheapestPath();
		counter += 1;
		// console.log(counter, pathsToCheck);
	}
	return lowestPath;
}

// reset paths variable with only our current node
const resetPaths = (start) => {
	pathsToCheck = [start]; // hardcoded start
	lowestPathWeight = 999999999;
	lowestPath = []
}


const findCheapestPath = () => {
	let curPath = pathsToCheck.shift();
	let pathNode = curPath[0];
	let pathWeight = pathNode.weight;
	let pathNodeNeighbors = findNeightbors(pathNode.x, pathNode.y);

	pathNodeNeighbors.forEach(neighbor => {
		let neighborWeight = findNodeWeight(neighbor, destination);
		let edgeWeight = findEdgeWeight(curPath, neighbor, windDirection);
		let totalWeight = pathWeight + neighborWeight + edgeWeight;
		let tempPath = [...curPath];
		// console.log(`neighbor: ${neighborWeight}, destination: ${destination}, edge: ${edgeWeight}, pathWeight: ${pathWeight}, x-y: ${neighbor.x}-${neighbor.y}`);
		tempPath.unshift({weight: totalWeight, ...neighbor});
		addPath(tempPath);
	});
}


const addPath = (path) => {
	let pathNode = path[0]
	if (pathNode.x === destination.x && pathNode.y === destination.y){ // if we get to the destination
		if (pathNode.weight < lowestPathWeight){ // keep this new path if its good
			lowestPathWeight = pathNode.weight;
			lowestPath = path;
		} // discard if we have a better one
	}
	// console.log('pre', pathsToCheck);
	pathsToCheck.push(path); // just put the path in teh global
	pathsToCheck.sort((a, b) => a[0].weight - b[0].weight); // sort it so we have the shortest path on top
	// console.log('post',pathsToCheck);
}

// calculate the distance from the destination for the current node
const findNodeWeight = (node, dest) => {
	// got the converter to meters here: https://www.movable-type.co.uk/scripts/latlong.html
	const R = 6371e3; // metres
	const φ1 = node.lat * Math.PI/180; // φ, λ in radians
	const φ2 = dest.lat * Math.PI/180;
	const Δφ = (dest.lat-node.lat) * Math.PI/180;
	const Δλ = (dest.lng-node.lng) * Math.PI/180;

	const a = Math.sin(Δφ/2) * Math.sin(Δφ/2) +
	          Math.cos(φ1) * Math.cos(φ2) *
	          Math.sin(Δλ/2) * Math.sin(Δλ/2);
	const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));

	const meters = R * c; // in metres
	return meters;
	// return nodeWeightMultiplier * Math.pow(node.lat - dest.lat, 2) + Math.pow(node.lng - dest.lng, 2);
	// return nodeWeightMultiplier * Math.max(Math.abs(node.lat - dest.lat) + Math.abs(node.lng - dest.lng));
}

// calculate the edge weight between the start node and dest node
// 	   So we calculate if it is staying the same direction, and direction with respect to wind
const findEdgeWeight = (prevPath, dest, windDirection) => {
	let weight = 0;

	if (prevPath.length > 1) {
		let nodeA = prevPath[1];
		let nodeB = prevPath[0];
		let nodeC = dest;
		let slopeAB = (nodeA.y - nodeB.y)/(nodeA.x - nodeB.x);
		let slopeBC = (nodeB.y - nodeC.y)/(nodeB.x - nodeC.x);
		if (slopeAB === slopeBC) // find out if nodes are collinear by seeing if slopes are equatl
			weight += currentDirectionWeight;
	}

	let startNode = prevPath[0];
	// TODO ADD DIRECTION BASED ON WIND
	return weight;
}

// find and returns an array of the neighbors of a given point
const findNeightbors = (x, y) => {
	let neighbors = [];
	if (x !== 0) // left
		neighbors.push(gridCenters[x - 1][y]);
	if (x !== maxX) // right
		neighbors.push(gridCenters[x + 1][y]);
	if (x !== 0 && y !== 0) //  top left
		neighbors.push(gridCenters[x - 1][y - 1]);
	if (x !== maxX && y !== 0) // top right
		neighbors.push(gridCenters[x + 1][y - 1]);
	if (x !== 0 && y !== maxY) //  bot left
		neighbors.push(gridCenters[x - 1][y + 1]);
	if (x !== maxX && y !== maxY) // bot right
		neighbors.push(gridCenters[x + 1][y + 1]);
	if (y !== 0) // top
		neighbors.push(gridCenters[x][y - 1]);
	if (y !== maxY) // bottom
		neighbors.push(gridCenters[x][y + 1]);

	return neighbors;
}


