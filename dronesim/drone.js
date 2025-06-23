// import * as THREE from 'three';
// import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js';
// import { degToRad } from 'three/src/math/MathUtils.js';

// Scene setup
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(75, window.innerWidth/window.innerHeight, 0.1, 1000);
const renderer = new THREE.WebGLRenderer();
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

// Lighting
const light = new THREE.DirectionalLight(0xffffff, 1);
light.position.set(1, 1, 1).normalize();
scene.add(light);

// Drone model (basic box for now)
// const geometry = new THREE.BoxGeometry(1, 0.2, 1);
// const material = new THREE.MeshPhongMaterial({ color: 0x00ffcc });
// const drone = new THREE.Mesh(geometry, material);
//////// drone geometry

// Drone Body
const droneGeometry = new THREE.BoxGeometry(4, 0.5, 4);
const droneMaterial = new THREE.MeshPhongMaterial({ color: 0xff00ff });
const drone = new THREE.Mesh(droneGeometry, droneMaterial);
// Raise the drone above the ground to prevent clipping
drone.position.y = 1;
scene.add(drone);

// Propeller Geometry
const propellerGeometry = new THREE.CircleGeometry(0.5, 32);
const propellerMaterial = new THREE.MeshBasicMaterial({ color: 0x0000ff, side: THREE.DoubleSide });

// Create Propellers

const propellers = [];
for (let i = 0; i < 4; i++) {
  const propeller = new THREE.Mesh(propellerGeometry, propellerMaterial);
  propeller.rotation.x = Math.PI / 2;
  // Raise propellers higher above the drone body
  propeller.position.set(i < 2 ? -1.5 : 1.5, 0.25, i % 2 === 0 ? -1.5 : 1.5);
  drone.add(propeller);
  propellers.push({ mesh: propeller, speed: 0 });
}

//////// drone geometry end ///
 

// Ground
const groundGeo = new THREE.PlaneGeometry(100, 100);
const groundMat = new THREE.MeshStandardMaterial({ color: 0x333333 });
const ground = new THREE.Mesh(groundGeo, groundMat);
ground.rotation.x = -Math.PI / 2;
scene.add(ground);

// Camera position
camera.position.set(0, 5, 8);
camera.lookAt(0, 0, 0);

// PID Controller
function PID(kp, ki, kd) {
  let prevError = 0, integral = 0;
  return function (target, current, dt) {
    const error = target - current;
    integral += error * dt;
    const derivative = (error - prevError) / dt;
    // derivative = current;
    prevError = error;
    return 0.1*(kp * error + ki * integral + kd * derivative);
  };
}

// Create separate PID controllers
const pitchPID = PID(1.2, 0.3, 0.05);
const rollPID = PID(1.2, 0.3, 0.05);

// Simulated drone state
let rotation = { pitch: 0, roll: 0 };
let joystickInput = { pitch: 0, roll: 0 };

// Joystick

const joystick = nipplejs.create({
  zone: document.getElementById('joystick'),
  mode: 'static',
  position: { left: '50%', top: '50%' },
  color: 'blue'
});



joystick.on('move', (evt, data) => {
  if (data && data.angle) {
    
    const rad = data.angle.radian;
    joystickInput.pitch = -Math.sin(rad) * data.distance / 50;
    joystickInput.roll = Math.cos(rad) * data.distance / 50;
  }
});

joystick.on('end', () => {
  joystickInput.pitch = 0;
  joystickInput.roll = 0;
});

document.getElementById('imu-pitch')
.addEventListener('change', (event) => {
 
  pitch_IMU = parseFloat(event.target.value);
})
document.getElementById('imu-roll')
.addEventListener('change', (event) => {
  roll_IMU = parseFloat(event.target.value);
})
document.getElementById('imu-yaw')
.addEventListener('change', (event) => {
  yaw_IMU = -parseFloat(event.target.value);
})
/*
const joystick = new VirtualJoystick({
    container: document.getElementById('joystick'),
    mouseSupport: true, // Allows desktop use
    limitStickTravel: true,
    stickRadius: 75,
    strokeStyle: 'blue'
  });
 

  function updateJoystick() {
    const dx = joystick.deltaX();
    const dy = joystick.deltaY();

    // Normalize distance based on max radius
    const maxDistance = 75; // same as stickRadius
    const distance = Math.min(Math.sqrt(dx * dx + dy * dy), maxDistance);

    const angle = Math.atan2(dy, dx);

    joystickInput.pitch = -Math.sin(angle) * distance / maxDistance;
    joystickInput.roll = Math.cos(angle) * distance / maxDistance;

    requestAnimationFrame(updateJoystick);
  }

  // Reset joystick values on release
  setInterval(() => {
    if (!joystick._pressed) {
      joystickInput.pitch = 0;
      joystickInput.roll = 0;
    }
  }, 100);

  updateJoystick();
*/




// Variables for controlANGLE()
let roll_des = 0, pitch_des = 0, yaw_des = 0, thro_des = 0.75;
let roll_IMU = 0, pitch_IMU = 0, yaw_IMU=0, GyroX = 0, GyroY = 0, GyroZ = 0;
let channel_1_pwm = 1100; // Simulated throttle value
let i_limit = 10;

let error_roll = 0, error_pitch = 0, error_yaw = 0;
let integral_roll = 0, integral_pitch = 0, integral_yaw = 0;
let integral_roll_prev = 0, integral_pitch_prev = 0, integral_yaw_prev = 0;
let derivative_roll = 0, derivative_pitch = 0, derivative_yaw = 0;
let error_yaw_prev = 0;

let roll_PID = 0, pitch_PID = 0, yaw_PID = 0;

// let Kp_roll_angle = 1.2, Ki_roll_angle = 0.3, Kd_roll_angle = 0.05;
// let Kp_pitch_angle = 1.2, Ki_pitch_angle = 0.3, Kd_pitch_angle = 0.05;
// let Kp_yaw = 1.0, Ki_yaw = 0.0, Kd_yaw = 0.0;

const Kp_roll_angle = 0.2;    //Roll P-gain - angle mode 
const Ki_roll_angle = 0.3;    //Roll I-gain - angle mode
const Kd_roll_angle = 0.05;   //Roll D-gain - angle mode (if using controlANGLE2, MUST be 0.0. Suggested default for controlANGLE is 0.05)
const Kp_pitch_angle = 0.2;   //Pitch P-gain - angle mode
const Ki_pitch_angle = 0.3;   //Pitch I-gain - angle mode
const Kd_pitch_angle = 0.05;  //Pitch D-gain - angle mode (if using controlANGLE2, MUST be 0.0. Suggested default for controlANGLE is 0.05)

const Kp_yaw = 0.3;      //Yaw P-gain
const Ki_yaw = 0.05;     //Yaw I-gain
const Kd_yaw = 0.00015;   

function constrain(val, min, max) {
  return Math.max(min, Math.min(max, val));
}

function controlANGLE(dt) {
  GyroX = roll_IMU; GyroY = pitch_IMU; GyroZ = yaw_IMU; // Simulated gyro values
  //DESCRIPTION: Computes control commands based on state error (angle)
  /*
   * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in 
   * getDesState(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features
   * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent 
   * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
   * they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
   * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I 
   * terms will always start from 0 on takeoff. This function updates the variables roll_PID, pitch_PID, and yaw_PID which
   * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in controlMixer().
   */
  
  //Roll
  error_roll = roll_des - roll_IMU;
  integral_roll = integral_roll_prev + error_roll*dt;
  if (channel_1_pwm < 1060) {   //don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_roll = GyroX;
  roll_PID = 0.01*(Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll - Kd_roll_angle*derivative_roll); //scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (channel_1_pwm < 1060) {   //don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_pitch = GyroY;
  pitch_PID = .01*(Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - Kd_pitch_angle*derivative_pitch); //scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (channel_1_pwm < 1060) {   //don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  integral_roll_prev = integral_roll;
  //Update pitch variables
  integral_pitch_prev = integral_pitch;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

let m1_command_scaled,m2_command_scaled,m3_command_scaled,m4_command_scaled,m5_command_scaled,m6_command_scaled,s1_command_scaled,s2_command_scaled,s3_command_scaled,s4_command_scaled,s5_command_scaled,s6_command_scaled,s7_command_scaled = 0;
let m1_command_PWM,m2_command_PWM,m3_command_PWM,m4_command_PWM,m5_command_PWM,m6_command_PWM,s1_command_PWM,s2_command_PWM,s3_command_PWM,s4_command_PWM,s5_command_PWM,s6_command_PWM,s7_command_PWM = 0;
function controlMixer() {
  //DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
  /*
   * Takes roll_PID, pitch_PID, and yaw_PID computed from the PID controller and appropriately mixes them for the desired
   * vehicle configuration. For example on a quadcopter, the left two motors should have +roll_PID while the right two motors
   * should have -roll_PID. Front two should have -pitch_PID and the back two should have +pitch_PID etc... every motor has
   * normalized (0 - 1) thro_des command for throttle control. Can also apply direct unstabilized commands from the transmitter with 
   * roll_passthru, pitch_passthru, and yaw_passthu. mX_command_scaled and sX_command scaled variables are used in scaleCommands() 
   * in preparation to be sent to the motors and servos.
   */
  //Quad mixing
  //m1 = front left, m2 = front right, m3 = back right, m4 = back left
  m1_command_scaled = thro_des - pitch_PID + roll_PID + yaw_PID;
  m2_command_scaled = thro_des - pitch_PID - roll_PID - yaw_PID;
  m3_command_scaled = thro_des + pitch_PID - roll_PID + yaw_PID;
  m4_command_scaled = thro_des + pitch_PID + roll_PID - yaw_PID;
  m5_command_scaled = 0;
  m6_command_scaled = 0;

  //0.5 is centered servo, 0 is zero throttle if connecting to ESC for conventional PWM, 1 is max throttle
  s1_command_scaled = 0;
  s2_command_scaled = 0;
  s3_command_scaled = 0;
  s4_command_scaled = 0;
  s5_command_scaled = 0;
  s6_command_scaled = 0;
  s7_command_scaled = 0;

  //my---tailsitter
  //hover
  // m1_command_scaled = thro_des - yaw_PID; // left motor
  // m2_command_scaled = thro_des + yaw_PID; // right motor
  // s1_command_scaled = /*servo_left_trim */  pitch_PID + roll_PID ; //left servo
  // s2_command_scaled = /*servo_right_trim */  pitch_PID - roll_PID; //right servo
  // forward
  // m2_command_scaled = thro_des + yaw_PID;
  // m1_command_scaled = thro_des - yaw_PID;
  // s1_command_scaled = /*servo_left_trim */ roll_PID + pitch_PID;  //left servo
  // s2_command_scaled = /*servo_right_trim */ roll_PID - pitch_PID; //right servo
  // m1_command_scaled = thro_des  - pitch_PID + roll_PID;
  // m2_command_scaled = thro_des + yaw_PID - roll_PID;
  // m3_command_scaled = thro_des  + pitch_PID + roll_PID;
  // m4_command_scaled = thro_des - yaw_PID - roll_PID;
  m1_command_scaled = thro_des   + yaw_PID + roll_PID;
  m2_command_scaled = thro_des + pitch_PID - roll_PID;
  m3_command_scaled = thro_des  - yaw_PID - roll_PID;
  m4_command_scaled = thro_des - pitch_PID + roll_PID;
  //my---tailsitter-end
  //Example use of the linear fader for float type variables. Linearly interpolates between minimum and maximum values for Kp_pitch_rate variable based on state of channel 6
  /*
  if (channel_6_pwm > 1500){
    Kp_pitch_rate = floatFaderLinear(Kp_pitch_rate, 0.1, 0.3, 5.5, 1, 2000); //parameter, minimum value, maximum value, fadeTime (seconds), state (0 min or 1 max), loop frequency
  }
  else if (channel_6_pwm < 1500) {
    Kp_pitch_rate = floatFaderLinear(Kp_pitch_rate, 0.1, 0.3, 2.5, 0, 2000); //parameter, minimum value, maximum value, fadeTime, state (0 min or 1 max), loop frequency
  }
  */
 const pwmVals = [
    m1_command_PWM,
    m2_command_PWM,
    m3_command_PWM,
    m4_command_PWM
 ];

  // Find the max PWM value
  let maxPWM = pwmVals[0];
  for (let i = 1; i < 4; ++i) {
    if (pwmVals[i] > maxPWM) maxPWM = pwmVals[i];
  }

  // If maxPWM exceeds 250, scale all accordingly
  if (maxPWM > 250) {
    let scale = 250.0 / maxPWM;
    for (let i = 0; i < 4; ++i) {
      pwmVals[i] = Math.floor(pwmVals[i] * scale);
    }
    m1_command_PWM = pwmVals[0];
    m2_command_PWM = pwmVals[1];
    m3_command_PWM = pwmVals[2];
    m4_command_PWM = pwmVals[3];
  }
}

function scaleCommands() {
  //DESCRIPTION: Scale normalized actuator commands to values for ESC protocol
  /*
   * mX_command_scaled variables from the mixer function are scaled to 125-250us for OneShot125 protocol. sX_command_scaled variables from
   * the mixer function are scaled to 0-180 for the servo library using standard PWM.
   * mX_command_PWM are updated here which are used to command the motors in commandMotors(). sX_command_PWM are updated 
   * which are used to command the servos.
   */
  //Scaled to 125us - 250us for oneshot125 protocol
  m1_command_PWM = m1_command_scaled*125 + 125;
  m2_command_PWM = m2_command_scaled*125 + 125;
  m3_command_PWM = m3_command_scaled*125 + 125;
  m4_command_PWM = m4_command_scaled*125 + 125;
  m5_command_PWM = m5_command_scaled*125 + 125;
  m6_command_PWM = m6_command_scaled*125 + 125;
  //Constrain commands to motors within oneshot125 bounds
  // m1_command_PWM = constrain(m1_command_PWM, 125, 250);
  // m2_command_PWM = constrain(m2_command_PWM, 125, 250);
  // m3_command_PWM = constrain(m3_command_PWM, 125, 250);
  // m4_command_PWM = constrain(m4_command_PWM, 125, 250);
  // m5_command_PWM = constrain(m5_command_PWM, 125, 250);
  // m6_command_PWM = constrain(m6_command_PWM, 125, 250);

  //Scaled to 0-180 for servo library
  s1_command_PWM = s1_command_scaled*180;
  s2_command_PWM = s2_command_scaled*180;
  s3_command_PWM = s3_command_scaled*180;
  s4_command_PWM = s4_command_scaled*180;
  s5_command_PWM = s5_command_scaled*180;
  s6_command_PWM = s6_command_scaled*180;
  s7_command_PWM = s7_command_scaled*180;
  //Constrain commands to servos within servo library bounds
  // s1_command_PWM = constrain(s1_command_PWM, 0, 180);
  // s2_command_PWM = constrain(s2_command_PWM, 0, 180);
  // s3_command_PWM = constrain(s3_command_PWM, 0, 180);
  // s4_command_PWM = constrain(s4_command_PWM, 0, 180);
  // s5_command_PWM = constrain(s5_command_PWM, 0, 180);
  // s6_command_PWM = constrain(s6_command_PWM, 0, 180);
  // s7_command_PWM = constrain(s7_command_PWM, 0, 180);

}

// create a 3D rectangle with stick handles to drag and set angle values
 

function updateTable() {
  document.getElementById('gyro-values').textContent = `${gyro.pitch.toFixed(2)}, ${gyro.roll.toFixed(2)}`;
  document.getElementById('pid-values').textContent = `${pidOutput.pitch.toFixed(2)}, ${pidOutput.roll.toFixed(2)}`;
  document.getElementById('joystick-values').textContent = `${joystickInput.pitch.toFixed(2)}, ${joystickInput.roll.toFixed(2)}`;
  document.getElementById('imu-values').textContent = `${roll_IMU.toFixed(2)},${pitch_IMU.toFixed(2)}, ${yaw_IMU.toFixed(2)}`;
  document.getElementById('pid-values2').textContent = `${roll_PID.toFixed(2)}, ${pitch_PID.toFixed(2)}, ${yaw_PID.toFixed(2)}`;
  document.getElementById('motor-values').innerHTML = `${m1_command_PWM.toFixed(2)} <br/> ${m2_command_PWM.toFixed(2)} <br/> ${m3_command_PWM.toFixed(2)} <br/> ${m4_command_PWM.toFixed(2)}`;
  document.getElementById('imu-values-derived').textContent = `${pitch_IMUderived.toFixed(2)}, ${roll_IMUderived.toFixed(2)}, ${yaw_IMUderived.toFixed(2)}`;
}

//// start of motorRectangle
// draw a thin rectangle by using m1_command_PWM, m2_command_PWM , m3_command_PWM , m4_command_PWM as corner heights
// show vertical bars at each corner representing motor PWM values

// Helper to create a vertical bar at a given (x, z) position
function createMotorBar(motorForceMesh, x, z, color = 0xff0000) {
  const geometry = new THREE.BoxGeometry(0.2, 1, 0.2);
  const material = new THREE.MeshBasicMaterial({ color });
  const mesh = new THREE.Mesh(geometry, material);
  mesh.position.set(x, 0.6, z); // y will be updated dynamically
  scene.add(mesh);
  return mesh;
}

// Positions for the 4 corners (relative to drone center)
const barPositions = [
  [-2, 0],  // left
  [0, -2],  // back
  [2, 0],   // right
  [0, 2],   // front
];



// Update bar heights each frame
function updateMotorBars() {
  // Map PWM (125-250) to bar height (e.g. 0.2 to 2.0)
  function pwmToHeight(pwm) {
    return 0.2 + (pwm - 125) / 100;
  }
  const pwms = [m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM];
  for (let i = 0; i < 4; ++i) {
    const height = pwmToHeight(pwms[i]);
    motorBars[i].scale.y = height;
    motorBars[i].position.y = height / 2 + 1.5; // keep base at y=0.1
  }
}

// Call updateMotorBars in the animation loop
// (add this to the animate() function: updateMotorBars();)
function createMotorRectangle() {
  // Create geometry for the thin rectangle (quad)
  const quadWidth = 4, quadDepth = 4, baseY = 0.1;
  const quadGeometry = new THREE.BufferGeometry();
  const quadVertices = new Float32Array([
    -quadWidth/2, baseY, -quadDepth/2, // front-left
    quadWidth/2, baseY, -quadDepth/2, // front-right
    quadWidth/2, baseY,  quadDepth/2, // back-right
    -quadWidth/2, baseY,  quadDepth/2  // back-left
  ]);
  quadGeometry.setAttribute('position', new THREE.BufferAttribute(quadVertices, 3));
  quadGeometry.setIndex([0, 1, 2, 0, 2, 3]);
  quadGeometry.computeVertexNormals();

  const quadMaterial = new THREE.MeshBasicMaterial({ color: 0xffff00, side: THREE.DoubleSide, transparent: true, opacity: 0.4 });
  const quadMesh = new THREE.Mesh(quadGeometry, quadMaterial);
  quadMesh.position.y = 1;
  quadMesh.rotation.y = THREE.MathUtils.degToRad(45) ; // Rotate to lie flat
  scene.add(quadMesh);

  return quadGeometry;
}
const motorRectGeometry = createMotorRectangle();

  // Create bars and store references
const motorBars = [
  createMotorBar(motorRectGeometry, barPositions[0][0], barPositions[0][1], 0xff0000),
  createMotorBar(motorRectGeometry, barPositions[1][0], barPositions[1][1], 0x00ff00),
  createMotorBar(motorRectGeometry, barPositions[2][0], barPositions[2][1], 0x0000ff),
  createMotorBar(motorRectGeometry, barPositions[3][0], barPositions[3][1], 0xffff00),
];

// Update the quad's corner heights each frame
function updateQuadHeights() {
  const baseY = 0.1; // Base height for the quad
  const positions = motorRectGeometry.attributes.position.array;
  // Map PWM (typically 125-250) to a visible height range, e.g. 0.1 to 1.0
  function pwmToHeight(pwm) {
    return baseY + (pwm - 125) / 200; // adjust divisor for scale
  }
  positions[1]  = pwmToHeight(m1_command_PWM); // front-left y
  positions[4]  = pwmToHeight(m2_command_PWM); // front-right y
  positions[7]  = pwmToHeight(m3_command_PWM); // back-right y
  positions[10] = pwmToHeight(m4_command_PWM); // back-left y
  motorRectGeometry.attributes.position.needsUpdate = true;
  motorRectGeometry.computeVertexNormals();
  updateMotorBars();
}
//// end of motorRectangle


//// derive IMU from motorPWM values
let pitch_IMUderived = 0, roll_IMUderived = 0, yaw_IMUderived = 0;
function deriveIMUFromMotorPWM() {
  // Simple mapping from motor PWM to IMU angles
  pitch_IMUderived = (m1_command_PWM - m2_command_PWM) / 1; // Simplified example
  roll_IMUderived = (m3_command_PWM - m4_command_PWM) / 1;
  yaw_IMUderived = (m5_command_PWM - m6_command_PWM) / 1;
}

//// end derive IMU from motorPWM values


//// start IMU box creation and update functions
/**
 * Draw a thin box (rectangle) above the drone to visualize IMU values (pitch, roll, yaw).
 * The box tilts according to pitch_IMU and roll_IMU, and can be colored by yaw_IMU.
 */
function createIMUBox() {
  const boxWidth = 4, boxDepth = 4, boxHeight = 0.1;
  const geometry = new THREE.BoxGeometry(boxWidth, boxHeight, boxDepth);
  const material = new THREE.MeshBasicMaterial({ color: 0x00ff00, opacity: 0.5, transparent: true });
  const mesh = new THREE.Mesh(geometry, material);
  mesh.position.y = 2.2; // Above the drone
  scene.add(mesh);
  return mesh;
}
const imuBox = createIMUBox();

function updateIMUBox() {
  // Apply IMU angles (in radians) to the box
  imuBox.rotation.x = THREE.MathUtils.degToRad(pitch_IMU);
  imuBox.rotation.z = -THREE.MathUtils.degToRad(yaw_IMU);
  // Optionally, color by yaw
  const hue = ((roll_IMU % 360) + 360) % 360 / 360;
  imuBox.material.color.setHSL(hue, 1, 0.5);
}
////  end of IMU box creation and update functions


///// start getting IMU data from device
/**
 * Receive IMU values from a real device via HTTP events (e.g., using Server-Sent Events).
 * Assumes your device streams IMU data as text/event-stream at /imu-stream endpoint.
 */
function setupIMUEventSource() {
  // Replace with your device's actual URL/endpoint
  // setInterval(() => {
  fetch('/data')
  .then(res => res.json())
  .then(data =>   {
    // Expecting JSON: { pitch: <deg>, roll: <deg>, yaw: <deg> }
    try {
      // if (typeof data.gyroY === 'number') document.getElementById('imu-pitch').value = parseInt(data.pitch_IMU);
      // if (typeof data.gyroX === 'number') document.getElementById('imu-roll').value = parseInt(data.roll_IMU);
      // if (typeof data.gyroZ === 'number') document.getElementById('imu-yaw').value = parseFloat(data.yaw_IMU);
      document.getElementById('imu-pitch').value = pitch_IMU = parseFloat(data.pitch_IMU);
      document.getElementById('imu-roll').value = roll_IMU = parseFloat(data.roll_IMU);
      document.getElementById('imu-yaw').value = yaw_IMU = parseFloat(data.yaw_IMU);
    } catch (e) {
      console.error('Error parsing IMU data:', e);
    }
  });
  // }, 1000);
}
 
///// end getting IMU data from device



// Animation loop
let gyro = { pitch: 0, roll: 0 };
let pidOutput = { pitch: 0, roll: 0 };
let lastTime = performance.now();
let imuRenderTime = lastTime; // Track last IMU update time

function animate() {
  requestAnimationFrame(animate);

  const now = performance.now();
  const dt = (now - lastTime) / 1000;
  lastTime = now;
  if (imuRenderTime + 300 < now) { // Update IMU every 300ms
    imuRenderTime = now;
    if (document.getElementById('syncRealtimeIMU').checked) {
      setupIMUEventSource();
    }
  }
  // Update simulated gyro (with gravity acting to zero out rotation)
  gyro.pitch += pidOutput.pitch * dt;
  gyro.roll += pidOutput.roll * dt;

  // Apply PID correction
  pidOutput.pitch = pitchPID(joystickInput.pitch, gyro.pitch, dt);
  pidOutput.roll = rollPID(joystickInput.roll, gyro.roll, dt);
  controlANGLE(dt);
  controlMixer();
  scaleCommands();
  // Apply rotation
  drone.rotation.x = gyro.pitch;
  drone.rotation.z = -gyro.roll;

  // Update propeller speeds and colors
  propellers.forEach((propeller, index) => {
    const speed = Math.abs(pidOutput.pitch) + Math.abs(pidOutput.roll);
    propeller.speed = speed;
    propeller.mesh.material.color.setHSL(speed / 10, 1, 0.5);
  });

// Update table
  updateTable();updateQuadHeights();updateIMUBox();deriveIMUFromMotorPWM();
  // const colr = document.getElementById('colorPicker').value
  // drone.material.color.set(colr);
  renderer.render(scene, camera);
}
animate();
