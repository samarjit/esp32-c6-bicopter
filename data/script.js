
let scene, camera, rendered, cube;

function parentWidth(elem) {
  return elem.parentElement.clientWidth;
}

function parentHeight(elem) {
  return elem.parentElement.clientHeight;
}

function init3D(){
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0xffffff);

  camera = new THREE.PerspectiveCamera(75, parentWidth(document.getElementById("3Dcube")) / parentHeight(document.getElementById("3Dcube")), 0.1, 1000);

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(parentWidth(document.getElementById("3Dcube")), parentHeight(document.getElementById("3Dcube")));

  document.getElementById('3Dcube').appendChild(renderer.domElement);

  // Create a geometry
  const geometry = new THREE.BoxGeometry(5, 1, 4);

  // Materials of each face
  var cubeMaterials = [
    new THREE.MeshBasicMaterial({color:0x03045e}),
    new THREE.MeshBasicMaterial({color:0x023e8a}),
    new THREE.MeshBasicMaterial({color:0x0077b6}),
    new THREE.MeshBasicMaterial({color:0x03045e}),
    new THREE.MeshBasicMaterial({color:0x023e8a}),
    new THREE.MeshBasicMaterial({color:0x0077b6}),
  ];

  const material = new THREE.MeshFaceMaterial(cubeMaterials);

  cube = new THREE.Mesh(geometry, material);
  scene.add(cube);
  camera.position.z = 5;
  renderer.render(scene, camera);
}

// Resize the 3D object when the browser window changes size
function onWindowResize(){
  camera.aspect = parentWidth(document.getElementById("3Dcube")) / parentHeight(document.getElementById("3Dcube"));
  //camera.aspect = window.innerWidth /  window.innerHeight;
  camera.updateProjectionMatrix();
  //renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setSize(parentWidth(document.getElementById("3Dcube")), parentHeight(document.getElementById("3Dcube")));

}

window.addEventListener('resize', onWindowResize, false);

// Create the 3D representation
init3D();

// Create events for the sensor readings
if (!!window.EventSource) {
  var source = new EventSource('/events');

  source.addEventListener('open', function(e) {
    console.log("Events Connected");
  }, false);

  source.addEventListener('error', function(e) {
    if (e.target.readyState != EventSource.OPEN) {
      console.log("Events Disconnected");
    }
  }, false);

  // source.addEventListener('gyro_readings', function(e) {
  //   //console.log("gyro_readings", e.data);
  //   var obj = JSON.parse(e.data);
  //   document.getElementById("gyroX").innerHTML = obj.gyroX;
  //   document.getElementById("gyroY").innerHTML = obj.gyroY;
  //   document.getElementById("gyroZ").innerHTML = obj.gyroZ;

  //   // Change cube rotation after receiving the readinds
  //   cube.rotation.x = obj.gyroY;
  //   cube.rotation.z = obj.gyroX;
  //   cube.rotation.y = obj.gyroZ;
  //   renderer.render(scene, camera);
  // }, false);

  // source.addEventListener('temperature_reading', function(e) {
  //   console.log("temperature_reading", e.data);
  //   document.getElementById("temp").innerHTML = e.data;
  // }, false);

  // source.addEventListener('accelerometer_readings', function(e) {
  //   console.log("accelerometer_readings", e.data);
  //   var obj = JSON.parse(e.data);
  //   document.getElementById("accX").innerHTML = obj.accX;
  //   document.getElementById("accY").innerHTML = obj.accY;
  //   document.getElementById("accZ").innerHTML = obj.accZ;
  // }, false);


  // source.addEventListener('rollpitchyaw_readings', function(e) {
  //   console.log("rollpitchyaw_readings", e.data);
  //   var obj = JSON.parse(e.data);
  //   document.getElementById("roll_IMU").innerHTML = obj.roll_IMU;
  //   document.getElementById("pitch_IMU").innerHTML = obj.pitch_IMU;
  //   document.getElementById("yaw_IMU").innerHTML = obj.yaw_IMU;
  // }, false);

  // source.addEventListener('PID_readings', function(e) {
  //   console.log("PID_readings", e.data);
  //   var obj = JSON.parse(e.data);
  //   document.getElementById("roll_PID").innerHTML = obj.roll_PID;
  //   document.getElementById("pitch_PID").innerHTML = obj.pitch_PID;
  //   document.getElementById("yaw_PID").innerHTML = obj.yaw_PID;
  // }, false);

   source.addEventListener('motor_readings', function(e) {
    console.log("motor_readings", e.data);
    var obj = JSON.parse(e.data);
    
    document.getElementById("gyroX").innerHTML = obj.gyroX;
    document.getElementById("gyroY").innerHTML = obj.gyroY;
    document.getElementById("gyroZ").innerHTML = obj.gyroZ;

    // Change cube rotation after receiving the readinds
    cube.rotation.x = obj.gyroY;
    cube.rotation.z = obj.gyroX;
    cube.rotation.y = obj.gyroZ;
    renderer.render(scene, camera);

    // document.getElementById("accX").innerHTML = obj.accX;
    // document.getElementById("accY").innerHTML = obj.accY;
    // document.getElementById("accZ").innerHTML = obj.accZ;

    document.getElementById("channel_1_pwm").innerHTML = obj.channel_1_pwm;
    document.getElementById("channel_2_pwm").innerHTML = obj.channel_2_pwm;
    document.getElementById("channel_3_pwm").innerHTML = obj.channel_3_pwm;
    document.getElementById("channel_4_pwm").innerHTML = obj.channel_4_pwm;

    document.getElementById("roll_IMU").innerHTML = obj.roll_IMU;
    document.getElementById("pitch_IMU").innerHTML = obj.pitch_IMU;
    document.getElementById("yaw_IMU").innerHTML = obj.yaw_IMU;

    document.getElementById("roll_PID").innerHTML = obj.roll_PID;
    document.getElementById("pitch_PID").innerHTML = obj.pitch_PID;
    document.getElementById("yaw_PID").innerHTML = obj.yaw_PID;

    document.getElementById("thro_des").innerHTML = obj.thro_des;
    document.getElementById("roll_des").innerHTML = obj.roll_des;
    document.getElementById("pitch_des").innerHTML = obj.pitch_des;
    document.getElementById("yaw_des").innerHTML = obj.yaw_des;

    document.getElementById("MLeft").innerHTML = obj.MLeft;
    document.getElementById("MRight").innerHTML = obj.MRight;
    document.getElementById("SLeft").innerHTML = obj.SLeft;
    document.getElementById("SRight").innerHTML = obj.SRight;
    // send to simulator
    try {
        fetch('http://localhost:5173/data', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(obj),
      });
    } catch(error) {
      console.error('Error sending data to simulator:', error);
    }
}, false);

  
}

function resetPosition(element){
  var xhr = new XMLHttpRequest();
  // xhr.open("GET", "/"+element.id, true);
  // console.log(element.id);
  xhr.open("GET", '/calibratefHigherLimit', true);
  console.log('calibrate joystick');
  xhr.send();
}