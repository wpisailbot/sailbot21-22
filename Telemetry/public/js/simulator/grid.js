// generates grid of 13x10 at Lake attitash with a squareRad of 0.001
const gridCorners = {topleft: { lat: 42.85414938719151, lng: -70.98833200038395 }, botleft: { lat: 42.84417554052568, lng: -70.98833200038395 }, botright: { lat: 42.84401759443596, lng: -70.97532473965633 }, topright: { lat: 42.85367747491202, lng: -70.97532473965633 }};
let gridCenters = [];


// generate GridLines and their centers
const genGridlines = () => {
    const squareRad = 0.001; // 10 meters is 4th decimal places I think? (near the equator it is 11)

    let latLines = [];
    let lngLines = [];

    for (let i = gridCorners.topleft.lng; i <= gridCorners.topright.lng; i += squareRad) {
        latLines.push([{lat: gridCorners.topleft.lat, lng: i}, {lat: gridCorners.botleft.lat, lng: i}]);
    }

    for (let i = gridCorners.topleft.lat; i >= gridCorners.botleft.lat; i -= squareRad) {
        lngLines.push([{lat: i, lng:  gridCorners.topleft.lng}, {lat: i, lng:  gridCorners.topright.lng}]);
    }

    let gridIntersects = [];

    // generating all the points where the 2 gridlines intersect
    latLines.forEach((latLine, i) => {
        let temp = [];
        lngLines.forEach((lngLine, j) => {
            temp.push({x: i, y: j, lat: lngLine[0].lat, lng: latLine[0].lng});
        });
        gridIntersects.push(temp);
    });


    for (let i = 0; i < gridIntersects.length - 1; i++){
        let temp = []
        leftLngLine = gridIntersects[i]
        rightLngLine = gridIntersects[i + 1]

        for (let j = 0; j < leftLngLine.length - 1; j++){
            let topLeftCorner = leftLngLine[j]
            let topRightCorner = rightLngLine[j]
            let botLeftCorner = leftLngLine[j + 1]
            let center = { lat: (topLeftCorner.lat + botLeftCorner.lat)/2,lng: (topLeftCorner.lng + topRightCorner.lng)/2 };

            temp.push({x: i, y: j, lat: center.lat, lng: center.lng});
        }
        gridCenters.push(temp);
    }
}

genGridlines();