let imuContext = null;
let hefContext = null;
let batContext = null;
let tmpContext = null;
let hmdContext = null;
let uscContext = null;
let imuChart = null;
let hefChart = null;
let batChart = null;
let tmpChart = null;
let hmdChart = null;
let uscChart = null;
let sensorArr = [];
let chartArr = [];

function shiftSensor(configObj) {
  if (configObj.data.labels.length === 15) {
    configObj.data.labels.shift();
    configObj.data.datasets[0].data.shift();
  }
}

function shiftImuSensor() {
  if (imuConfig.data.labels.length === 15) {
    imuConfig.data.labels.shift();
    imuConfig.data.datasets[0].data.shift();
    imuConfig.data.datasets[1].data.shift();
    imuConfig.data.datasets[2].data.shift();
  }
}

function imuUpdate(data) {
  imuConfig.data.labels.push(data.timestamp);
  imuConfig.data.datasets[0].data.push(data.imu_data[0]);
  imuConfig.data.datasets[1].data.push(data.imu_data[1]);
  imuConfig.data.datasets[2].data.push(data.imu_data[2]);
}

function uscUpdate(data) {
  uscConfig.data.datasets[0].data = data.ultrasonic_data;
}

function hefUpdate(data) {
  hefConfig.data.labels.push(data.timestamp);
  hefConfig.data.datasets[0].data.push(data.hall_effect_data);
}

function batUpdate(data) {
  batConfig.data.labels.push(data.timestamp);
  batConfig.data.datasets[0].data.push(data.battery_data);
}

function tmpUpdate(data) {
  tmpConfig.data.labels.push(data.timestamp);
  tmpConfig.data.datasets[0].data.push(data.temperature_data);
}

function hmdUpdate(data) {
  hmdConfig.data.labels.push(data.timestamp);
  hmdConfig.data.datasets[0].data.push(data.humidity_data);
}

function downloadCSV(data, name) {
  let csvContent = "data:text/csv;charset=utf-8,";

  rows = [];
  rows.push([
    "timestamp",
    "cpu temp",
    "hall_effect",
    "humidity",
    "temperature",
    "imu_x",
    "imu_y",
    "imu_z",
    "ultrasonic 1",
    "ultrasonic 2",
    "ultrasonic 3",
    "ultrasonic 4",
    "ultrasonic 5",
    "ultrasonic 6",
    "ultrasonic 7",
    "lipo"
  ]);
  let i;
  sensorData = JSON.parse(data);
  for (i = 0; i < sensorData.temperature.length; i++) {
    timestamp = sensorData.timestamp[i];
    battery = sensorData.battery[i];
    hall_effect = sensorData.hall_effect[i];
    humidity = sensorData.humidity[i];
    temperature = sensorData.temperature[i];
    imu_x = sensorData.imu[i][0];
    imu_y = sensorData.imu[i][1];
    imu_z = sensorData.imu[i][2];
    ultrasonics = sensorData.ultrasonic[i];
    lipo = sensorData.lipo[i];
    rows.push([
      timestamp,
      battery,
      hall_effect,
      humidity,
      temperature,
      imu_x,
      imu_y,
      imu_z,
      ultrasonics,
      lipo
    ]);
  }

  console.log("FIRST Battery: " + battery);

  rows.forEach(function(rowArray) {
    let row = rowArray.join(",");
    csvContent += row + "\r\n";
  });

  var encodedUri = encodeURI(csvContent);
  var link = document.createElement("a");
  link.setAttribute("href", encodedUri);

  var dateObj = new Date();
  var month = dateObj.getUTCMonth() + 1;
  var day = dateObj.getUTCDate();
  var year = dateObj.getUTCFullYear();

  const download_string =
      name + " " + year + "-" + month + "-" + day + " data.csv";
  link.setAttribute("download", download_string);
  document.body.appendChild(link); // Required for FF

  link.click();
}

function terminate() {
  const carid = document.getElementById("car_id").innerText;

  const source_string = "/api/car/" + carid + "/terminate";
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      window.open("/", "_self");
    }
  };
  xhttp.open("GET", source_string, true);
  xhttp.send();
}

function startCar(){
  disableVideo();
  const carid = document.getElementById('car_id').innerText;
  document.getElementById("startCar").hidden = true;
  document.getElementById("stopCar").hidden = false;
  document.getElementById("selectColorButton").disabled = true;
  document.getElementById("autonomousDrivingSwitch").disabled = true;
  document.getElementById("servo-dropbtn").onmouseover = function()
  {
    document.getElementById("servo-content").style.visibility = "hidden";
  }
  document.getElementById("esc-dropbtn").onmouseover = function()
  {
    document.getElementById("esc-content").style.visibility = "hidden";
  }

  const source_string = "/api/car/" + carid + "/drive";
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      // Retrieve the speed value from the dashboard
      let output = JSON.parse(xhttp.response);
      console.log(output);
    }
  };
  xhttp.open("GET", source_string, true);
  xhttp.send();
}

function stopCar() {
  enableVideo();
  const carid = document.getElementById('car_id').innerText;
  document.getElementById("stopCar").hidden = true;
  document.getElementById("startCar").hidden = false;
  document.getElementById("selectColorButton").disabled = false;
  document.getElementById("autonomousDrivingSwitch").disabled = false;
  document.getElementById("servo-dropbtn").onmouseover = function()
  {
    document.getElementById("servo-content").style.visibility = "visible";
  }
  document.getElementById("esc-dropbtn").onmouseover = function()
  {
    document.getElementById("esc-content").style.visibility = "visible";
  }

  const stop_string = "/api/car/" + carid + "/set/speed/0";
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      updateSpeedSlider(carid);
    }
  };
  xhttp.open("POST", stop_string, true);
  xhttp.send();

  const stop2_string = "/api/car/" + carid + "/stop/drive";
  var xhttp2 = new XMLHttpRequest();
  xhttp2.open("GET", stop2_string, true);
  xhttp2.send();
}

function exportSensorData() {
  const carid = document.getElementById("car_id").innerText;
  const friendly_name = document.getElementById("friendly_name").innerText;
  const source_string = "/api/client/" + carid + "/export/data";
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      // Retrieve the sensor data from the car
      let sensorData = xhttp.response;
      downloadCSV(sensorData, friendly_name);
    }
  };
  xhttp.open("GET", source_string, true);
  xhttp.send();
}

document.addEventListener("DOMContentLoaded", function(event) {
  const image_elem = document.getElementById("streamer-image");
  const carid = document.getElementById("car_id").innerText;

  //    var socket = io.connect("https://ai-car.herokuapp.com/web", {
  var socket = io.connect(
      "http://" + document.domain + ":" + location.port + "/web",
      {
        reconnection: false
      }
  );

  socket.on("connect", () => {
    console.log("Connected");
  });

  socket.on("disconnect", () => {
    console.log("Disconnected");
  });

  socket.on("connect_error", error => {
    console.log("Connect error! " + error);
  });

  socket.on("connect_timeout", error => {
    console.log("Connect timeout! " + error);
  });

  socket.on("error", error => {
    console.log("Error! " + error);
  });

  let image2web_string = "image2web/" + carid;
  socket.on(image2web_string, msg => {
    image_elem.src = msg.image;
  });

  let data2web_string = "data2web/" + carid;
  socket.on(data2web_string, sensor_readings => {
    const data = JSON.parse(sensor_readings);
    for (let i = 0; i < sensorArr.length; i++) {
      shiftSensor(sensorArr[i]);
    }
    shiftImuSensor();
    imuUpdate(data);
    uscUpdate(data);
    hefUpdate(data);
    batUpdate(data);
    tmpUpdate(data);
    hmdUpdate(data);
    updateBatteryLevel(data);

    for (let i = 0; i < chartArr.length; i++) {
      chartArr[i].update();
    }
  });
});

function updateBatteryLevel(data){
  if(data.lipo_data <= 6.5){
    document.getElementById("batterylevel").style.background = "#FF0000";
  }
  else {
    document.getElementById("batterylevel").style.background = "#ffffff";
  }
  document.getElementById("batterylevel-text").innerHTML = data.lipo_data + " V";
}

function updateSpeedSlider(carid) {
  let speedSlider = document.getElementById("speedRange");
  let speed = document.getElementById("speed");
  const speed_string = "/api/car/" + carid + "/get/speed";
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      // Retrieve the speed value from the dashboard
      let speedValue = JSON.parse(xhttp.response);
      speed.innerHTML = speedValue;
      speedSlider.value = speedValue / 1;
    }
  };
  xhttp.open("GET", speed_string, true);
  xhttp.send();
}

function updateSteeringSlider(carid) {
  let steeringSlider = document.getElementById("steeringRange");
  let steering = document.getElementById("steering");
  const steering_string = "/api/car/" + carid + "/get/steering";
  var xhttp2 = new XMLHttpRequest();
  xhttp2.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      // Retrieve the speed value from the dashboard
      let steeringValue = JSON.parse(xhttp2.response);
      steering.innerHTML = steeringValue;
      steeringSlider.value = steeringValue / 5;
    }
  };
  xhttp2.open("GET", steering_string, true);
  xhttp2.send();
}

function updateServoAdjustment(carid) {
  let servoSlider = document.getElementById("servoRange");
  let servoAdjustment = document.getElementById("servo_adjustment");
  const servo_string = "/api/car/" + carid + "/get/servo_adjustment";
  var xhttp2 = new XMLHttpRequest();
  xhttp2.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      // Retrieve the speed value from the dashboard
      let servoValue = JSON.parse(xhttp2.response);
      servoAdjustment.innerHTML = servoValue;
      servoSlider.value = servoValue / 1;
    }
  };
  xhttp2.open("GET", servo_string, true);
  xhttp2.send();
}

function updateToggleDirection(carid) {
  let directionSwitch = document.getElementById("directionSwitch");
  const direction_string = "/api/car/" + carid + "/get/direction";
  var xhttp3 = new XMLHttpRequest();
  xhttp3.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      // Retrieve the speed value from the dashboard
      let directionValue = JSON.parse(xhttp3.response);
      if (directionValue == 1) {
        directionSwitch.checked = true;
      } else {
        directionSwitch.checked = false;
      }
    }
  };
  xhttp3.open("GET", direction_string, true);
  xhttp3.send();
}

function updateServoSwitch(carid) {
  let servoSwitch = document.getElementById("servoSwitch");
  const servo_string = "/api/car/" + carid + "/get/servo_direction";
  var xhttp3 = new XMLHttpRequest();
  xhttp3.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      // Retrieve the speed value from the dashboard
      let servoValue = JSON.parse(xhttp3.response);
      if (servoValue == 1) {
        servoSwitch.checked = true;
      } else {
        servoSwitch.checked = false;
      }
    }
  };
  xhttp3.open("GET", servo_string, true);
  xhttp3.send();
}

function enableVideo() {
  const carid = document.getElementById("car_id").innerText;
  document.getElementById("enableVideo").hidden = true;
  document.getElementById("disableVideo").hidden = false;
  document.getElementById("selectColorButton").disabled = false;

  const source_string = "/api/car/" + carid + "/enable/video";
  var xhttp = new XMLHttpRequest();
  xhttp.open("GET", source_string, true);
  xhttp.send();
}

function disableVideo() {
  const carid = document.getElementById("car_id").innerText;
  document.getElementById("enableVideo").hidden = false;
  document.getElementById("disableVideo").hidden = true;
  document.getElementById("selectColorButton").disabled = true;

  const source_string = "/api/car/" + carid + "/disable/video";
  var xhttp = new XMLHttpRequest();
  xhttp.open("GET", source_string, true);
  xhttp.send();
}

function updateVideo(carid) {
  const video_string = "/api/car/" + carid + "/get/stream";
  var xhttp4 = new XMLHttpRequest();
  xhttp4.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      // Retrieve the speed value from the dashboard
      let is_streaming = JSON.parse(xhttp4.response);
      if (is_streaming) {
        enableVideo();
      } else {
        disableVideo();
      }
    }
  };
  xhttp4.open("GET", video_string, true);
  xhttp4.send();
}

function changeServoButtonText(value) {
  var oldText = document.getElementById("servo-dropbtn").innerText;
  document.getElementById("servo-dropbtn").innerText = value;
  if (
      document.getElementById("servo-dropbtn").innerText ===
      document.getElementById("esc-dropbtn").innerText
  ) {
    document.getElementById("servo-dropbtn").innerText = oldText;
  } else {
    document.getElementById("esc-dropbtn").style.background = "#27fcff";
    document.getElementById("servo-dropbtn").style.background = "#27fcff";

    if(value.length === 10){
      var channel = value.substring(value.length - 2, value.length);

    }
    else if(value.length === 9){
      var channel = value.substring(value.length - 1, value.length);
    }
    changeServoChannel(channel);
  }
}

function changeServoChannel(value){
  var channel = value * 1;

  const servo_channel =
      "/api/car/" + carid + "/set/servo_channel/" + channel;
  var xhttp = new XMLHttpRequest();
  xhttp.open("POST", servo_channel, true);
  xhttp.send();
}

function changeESCButtonText(value) {
  var oldText = document.getElementById("esc-dropbtn").innerText;
  document.getElementById("esc-dropbtn").innerText = value;
  if (
      document.getElementById("esc-dropbtn").innerText ===
      document.getElementById("servo-dropbtn").innerText
  ) {
    document.getElementById("esc-dropbtn").innerText = oldText;
  } else {
    document.getElementById("esc-dropbtn").style.background = "#27fcff";
    document.getElementById("servo-dropbtn").style.background = "#27fcff";
    if(value.length === 10){
      var channel = value.substring(value.length - 2, value.length);
    }
    else if(value.length === 9){
      var channel = value.substring(value.length - 1, value.length);
    }
    changeESCChannel(channel);
  }
}

function changeESCChannel(value){
  var channel = value * 1;

  const esc_channel =
      "/api/car/" + carid + "/set/esc_channel/" + channel;
  var xhttp = new XMLHttpRequest();
  xhttp.open("POST", esc_channel, true);
  xhttp.send();
}

function createTutorial() {
  var intro = introJs();
  intro
      .setOptions({
        steps: [
          {
            title: "1) Enable/Disable Video",
            intro: "Press here to enable/disable the video feed",
            element: document.querySelector(".smallControlButton")
          },
          {
            title: "2) Speed",
            intro:
                "Select what percentage of top speed you would like the vehicle to travel at.",
            element: document.getElementById("speedRange")
          },
          {
            title: "3) Steering Aggressiveness",
            intro: "Select the vehicle's steering angle.",
            element: document.getElementById("steeringRange")
          },
          {
            title: "4) Servo Shield Adjustment",
            intro:
                "If needed, select how much to offset the vehicle's steering angle by.",
            element: document.getElementById("servoRange")
          },
          {
            title: "5) Servo Motor Channel Selection",
            intro:
                "This dropdown provides the option to change the channel for the Servo Shield (steering), in case a channel on the servo shield is malfunctioning.",
            element: document.getElementById("servo-dropbtn")
          },
          {
            title: "6) ESC Motor Channel Selection",
            intro:
                "This dropdown provides the option to change channel for the ESC motor (power), in case a channel on the ESC motor is malfunctioning.",
            element: document.getElementById("esc-dropbtn")
          },
          {
            title: "7) Invert Throttle Direction",
            intro:
                "If needed, invert the direction of the vehicle's ESC motor.",
            element: document.getElementById("directionSwitchLabel")
          },
          {
            title: "8) Invert Steering Direction",
            intro:
                "If needed, invert the direction of the vehicle's servo shield.",
            element: document.getElementById("servoSwitchLabel")
          },
          {
            title: "9) Enable/Disable Autonomous Driving",
            intro:
                "Toggling this switch will disable autonomous driving and enable manual driving with the arrow keys on the keyboard.",
            element: document.getElementById("autonomousDrivingSwitchLabel")
          },
          {
            title: "10) Lane Color Picker",
            intro:
                "Select the color of the vehicle's lanes. Continue selecting until the lanes are filled white on the camera feed.",
            element: document.getElementById("selectColorButton")
          },
          {
            title: "11) Start/Stop Button",
            intro: "Press here to start/stop the autonomous driving.",
            element: document.querySelector(".controlButton")
          },
          {
            title: "12) Show/Hide Charts Button",
            intro:
                "This button will show/hide the data charts.",
            element: document.getElementById("hideShowButton")
          },
          {
            title: "13) Data Charts",
            intro:
                "These charts show data for the CPU temperature, hall effect sensors, ultrasonic sensors, IMU sensor, the temperature and humidity. Click on each chart to learn more!",
            element: document.getElementById("ultrasonic")
          },
          {
            title: "14) Show/Hide Testing Table Button",
            intro:
                "This button will show/hide the testing and diagnostics buttons.",
            element: document.getElementById("hideShowTestsButton")
          },
          {
            title: "15) Testing/Diagnostics Table",
            intro:
                "These tests allow the user to ensure proper hardware is working as desired.",
            element: document.getElementById("testtable")
          },
          {
            title: "16) LiPo Battery Level",
            intro:
                "The current level of the LiPo battery will be displayed here. The battery should be charged or replaced once it reaches below 6.5V.",
            element: document.getElementById("batterylevel-outer")
          },
          {
            title: "17) Terminate Button",
            intro:
                "This button will terminate the program in case of any malfunctions.",
            element: document.getElementById("terminateButton")
          },
          {
            title: "18) Tutorial Button",
            intro:
                "Pressing this button will take you back through this tutorial.",
            element: document.getElementById("tutorialButton")
          }
        ],
        showProgress: true,
        showBullets: false,
        disableInteraction: true
      });
  intro.onchange(function () {
    if(this._currentStep === 3) {
      document.getElementById("advanced-settings").style.display = "inline";
      document.getElementById("arrow").style.transform = "rotate(225deg)";
    }
    if(this._currentStep === 6){
      document.getElementById("advanced-settings").style.display = "none";
      document.getElementById("arrow").style.transform = "rotate(45deg)";
    }
  });
  intro.start();
}

function hideShowCharts() {
  var x = document.getElementById("charts");
  if (x.style.display === "none") {
    x.style.display = "flex";
    document.getElementById("hideShowButton").innerText = "Hide Charts";
  } else {
    x.style.display = "none";
    document.getElementById("hideShowButton").innerText = "Show Charts";
  }
}

function hideShowTests() {
  var x = document.getElementById("tests");
  if (x.style.display === "none") {
    x.style.display = "flex";
    document.getElementById("hideShowTestsButton").innerText = "Hide Testing Table";
  } else {
    x.style.display = "none";
    document.getElementById("hideShowTestsButton").innerText = "Show Testing Table";
  }
}

function hideShowAdvancedSettings() {
  var advancedSettings = document.getElementById("advanced-settings");
  var arrow = document.getElementById("arrow");

  if(advancedSettings.style.display === "none"){
    advancedSettings.style.display = "inline";
    arrow.style.transform = "rotate(225deg)";
  }
  else {
    advancedSettings.style.display = "none";
    arrow.style.transform = "rotate(45deg)";
  }
}

/* Servo angle test function */
function testServoAngle() {
  const carid = document.getElementById("car_id").innerText;
  const source_string = "/api/car/" + carid + "/test/servo_angle";
  var xhttp = new XMLHttpRequest();
  xhttp.open("GET", source_string, true);
  xhttp.send();
}

/* Servo angle test function */
function testThrottle() {
  const carid = document.getElementById("car_id").innerText;
  const source_string = "/api/car/" + carid + "/test/throttle";
  var xhttp = new XMLHttpRequest();
  xhttp.open("GET", source_string, true);
  xhttp.send();
}

function switchDrivingMode() {
  const startCar = document.getElementById("startCar");
  const isChecked = document.getElementById("autonomousDrivingSwitch").checked;
  // check if autonomous is checked or not
  // if not checked
  if (!isChecked) {
    console.log("manual driving enabled");
    startCar.disabled = true;
    document.getElementById("servo-dropbtn").onmouseover = function()
    {
      document.getElementById("servo-content").style.visibility = "hidden";
    }
    document.getElementById("esc-dropbtn").onmouseover = function()
    {
      document.getElementById("esc-content").style.visibility = "hidden";
    }
  }
  // if checked
  else {
    console.log("autonomous driving enabled");
    startCar.disabled = false;

    document.getElementById("servo-dropbtn").onmouseover = function()
    {
      document.getElementById("servo-content").style.visibility = "visible";
    }
    document.getElementById("esc-dropbtn").onmouseover = function()
    {
      document.getElementById("esc-content").style.visibility = "visible";
    }
  }
}

window.onload = function() {
  imuContext = document.getElementById("imuChart").getContext("2d");
  uscContext = document.getElementById("uscChart").getContext("2d");
  hefContext = document.getElementById("hefChart").getContext("2d");
  batContext = document.getElementById("batChart").getContext("2d");
  tmpContext = document.getElementById("tmpChart").getContext("2d");
  hmdContext = document.getElementById("hmdChart").getContext("2d");

  Chart.defaults.global.defaultFontColor = "#aeaeae";
  imuChart = new Chart(imuContext, imuConfig);
  uscChart = new Chart(uscContext, uscConfig);
  hefChart = new Chart(hefContext, hefConfig);
  batChart = new Chart(batContext, batConfig);
  tmpChart = new Chart(tmpContext, tmpConfig);
  hmdChart = new Chart(hmdContext, hmdConfig);

  sensorArr = [hefConfig, batConfig, tmpConfig, hmdConfig];
  chartArr = [imuChart, uscChart, hefChart, batChart, tmpChart, hmdChart];

  const carid = document.getElementById("car_id").innerText;
  updateVideo(carid);
  updateSpeedSlider(carid);
  updateSteeringSlider(carid);
  updateServoAdjustment(carid);
  updateToggleDirection(carid);
  updateServoSwitch(carid);
};

// add event listeners for manual driving
window.addEventListener("keyup", manualDrive);
window.addEventListener("keydown", manualDrive);

function manualDrive(event) {
  // only do this if autonomous is unchecked
  const isChecked = document.getElementById("autonomousDrivingSwitch").checked;
  if (isChecked) return;

  // prevent scrolling
  if(["ArrowUp","ArrowDown","ArrowLeft","ArrowRight"].indexOf(event.code) > -1) {
    event.preventDefault();
  }

  var debug_str = ""
  var source_string = ""
  switch(event.keyCode) {
    case 37:
      debug_str = "left arrow";
      if(event.type == "keydown") {
        // send GET request to server signalling servo left
        source_string = "/api/car/" + carid + "/drive/turn/left";
      } else {
        // send GET request to server signalling servo 0
        source_string = "/api/car/" + carid + "/drive/turn/left_stop";
      }
      break;
    case 38:
      debug_str = "up arrow";
      if(event.type == "keydown") {
        // send GET request to server signalling throttle up
        source_string = "/api/car/" + carid + "/drive/throttle/up";
      } else {
        // send GET request to server signalling throttle 0
        source_string = "/api/car/" + carid + "/drive/throttle/up_stop";
      }
      break;
    case 39:
      debug_str = "right arrow";
      if(event.type == "keydown") {
        // send GET request to server signalling servo right
        source_string = "/api/car/" + carid + "/drive/turn/right";
      } else {
        // send GET request to server signalling servo right
        source_string = "/api/car/" + carid + "/drive/turn/right_stop";
      }
      break;
    case 40:
      debug_str = "down arrow";
      if(event.type == "keydown") {
        // send GET request to server signalling throttle back
        source_string = "/api/car/" + carid + "/drive/throttle/back";
      } else {
        // send GET request to server signalling throttle 0
        source_string = "/api/car/" + carid + "/drive/throttle/back_stop";
      }
      break;
    default:
      return;
  }
  var xhttp = new XMLHttpRequest();
  xhttp.open("GET", source_string, true);
  xhttp.send();
}
