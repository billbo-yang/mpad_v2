function resetColors(){
    const carid = document.getElementById('car_id').innerText;
    const source_string = "/api/car/" + carid + "/reset/color";
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
           // Retrieve the sensor data from the car
           console.log("reset color channels")
        }
    };
    xhttp.open("GET", source_string, true);
    xhttp.send();
}

function sendCoordinates(x, y){
    const carid = document.getElementById('car_id').innerText;
    const source_string = "/api/car/" + carid + "/send/coordinates";
    var xhttp = new XMLHttpRequest();
    coordinates = JSON.stringify(
        {
            "x": x,
            "y": y
        })
    xhttp.open("POST", source_string, true);
    xhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
    xhttp.send(coordinates);
}

document.addEventListener("DOMContentLoaded", function(event) {
    const click_elem = document.getElementById("clickImage");
    const image_elem = document.getElementById("streamer-image");
    const carid = document.getElementById('car_id').innerText;
//    var socket = io.connect("https://ai-car.herokuapp.com/web", {
      var socket = io.connect('http://' + document.domain + ':' + location.port + '/web', {
      reconnection: false
    });

    let image2web_string = 'image2web/' + carid
    socket.on(image2web_string, (msg) => {
        click_elem.src = msg.image;
    });

    let filtered2web_string = 'filtered2web/' + carid
    socket.on(filtered2web_string, (msg) => {
        image_elem.src = msg.image;
    });
});

window.onload = function(){
    image = document.getElementById("clickImage");
    image.addEventListener("click", function(ev) {
        var x = ev.clientX;
        var y = ev.clientY;
        var canvas = document.createElement('canvas');
        canvas.width = image.width;
        canvas.height = image.height;
        canvas.getContext('2d').drawImage(image, 0, 0, image.width, image.height);
        sendCoordinates(ev.offsetX, ev.offsetY);
    });
}
