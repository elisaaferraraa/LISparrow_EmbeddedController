/*
 * =============================================================================
 * MoCap-Pixhawk MAVLink Bridge with AgileCascadedPID
 *
 * This firmware runs on an ESP32 and performs the following tasks:
 *
 * 1. Connects to a WiFi network ("WIsa-Fi") to receive UDP MoCap data
 * 2. Parses MoCap data (position + orientation quaternion)
 * 3. Sends the data to Pixhawk as MAVLink VISION_POSITION_ESTIMATE messages
 *
 * 4. Receives attitude (quaternion + angular velocity) from Pixhawk
 *    via MAVLink ATTITUDE_QUATERNION messages over Serial (UART)
 *
 * 5. Feeds orientation + angular velocity into an AgileCascadedPID controller
 *    to compute 5 control outputs: thrust, sweep left, sweep right, elevator, rudder
 *
 * 6. Sends these outputs to the Pixhawk as RC_CHANNELS_OVERRIDE commands,
 *    simulating RC stick inputs to control the actuators (PWM outputs)
 *
 * Requirements:
 * - PX4 firmware configured to accept RC overrides
 * - Actuators mapped to RC channels 1–5 (or as configured in PX4 params)
 * - UDP MoCap data streamed to ESP32 (7 floats: [x, y, z, qx, qy, qz, qw])
 * - MAVLink C headers (common message set) included in the /lib directory
 *
 * Author: Elisa Ferrara
 * Contact: elisa.ferrara@epfl.ch
 * =============================================================================
 */
#include <WiFi.h>
#include <WiFiUdp.h>
#include "lib/mavlink/common/mavlink.h"
#include <math.h>
#include "AgileCascadedPID.h"


// ====== WiFi Configuration ======
const char* ssid = "WiFli"; //"WIsa-Fi";
const char* password = "crazyflie";//"87654321";
WiFiUDP Udp;
unsigned int localPort = 8888;

const char* laptopIP = "192.168.194.243"; //"192.168.194.114"; my with WiFli //"192.168.209.213" my with WisaFi;  // <-- Your laptop IP on same network as ESP32
const int laptopPort = 9999;          // <-- Choose an unused UDP port for feedback
WiFiUDP laptopUdp;

const char* ElisalaptopIP = "192.168.194.114";
const int ElisalaptopPort = 7777;          // <-- Choose an unused UDP port for feedback
WiFiUDP ElisalaptopUdp;

const int QGCport = 14550;          // <-- Choose an unused UDP port for feedback
WiFiUDP QGC;


// ====== MAVLink UART Configuration ======
#define RX_PIN D7     // Pixhawk TX → Xiao RX
#define TX_PIN D6     // Pixhawk RX ← Xiao TX
#define BAUD 230400 //460800// //115200 //921600 



// ====== Globals ======
// MavLink
mavlink_message_t msg;
mavlink_attitude_quaternion_t att_q; //TODO eliminate thus
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
uint64_t start_time = 0;
float mocap_data[7] = {0}; //cancel initial values TODO
Eigen::VectorXf actuators(5); // TODO remove it
Eigen::Vector3f ekf_pos(0.0f, 0.0f, 0.0f);
bool is_armed = false; // Flag to request arming
constexpr int UDP_BUF_LEN = 300; // Maximum UDP packet size we expect (MAVLink max is ~280 bytes)
uint8_t udpBuf[UDP_BUF_LEN];

// Controller
AgileCascadedPID controller(0.04f);  // 25 Hz loop
Eigen::Quaternionf q_meas = Eigen::Quaternionf::Identity();
Eigen::Vector3f omega_meas = Eigen::Vector3f::Zero();
Eigen::Vector3f v_body = Eigen::Vector3f::Zero();
float u_sw_sym = -0.25f;
float last_u_ele = 0.0f;
float last_u_rud = 0.0f;
float roll_rad = 0.0f; // Roll angle in radians read after EKF
float pitch_rad = 0.0f;
float yaw_rad = 0.0f;
const unsigned long update_interval_ms = 40; //frequency at which controller runs (25 Hz)
// WARNING: if you want to change it you need also to change accordingly the controller update rate in AgileCascadedPID.h in PID.h


// Debug
uint16_t last_servo_raw[5] = {0};
float last_rc_override[5] = {0.0f};
//--frequency analysis
unsigned long last_update = 0;




void setup() {
  pinMode(LED_BUILTIN, OUTPUT);  // Initialize LED pin as output
  WiFi.setSleep(WIFI_PS_NONE);
  Serial.begin(115200);

  digitalWrite(LED_BUILTIN, HIGH);  // LED OFF before WiFi is connected
  //  WIFI SETUP
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  digitalWrite(LED_BUILTIN, LOW);  // LED ON when WiFi is connected
  Serial.println("\nWiFi connected");

  // UDP CONNECTIONS SETUP
  Udp.begin(localPort); // for reading MoCap
  laptopUdp.begin(laptopPort); //for sending to laptop specific info
  QGC.begin(QGCport); //for connecting to 
  ElisalaptopUdp.begin(ElisalaptopPort); //for sending to laptop specific info
  Serial.println("UDP listening...");

  // UART CONNECTION SETUP
  Serial1.begin(BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  if (Serial1.available()){
    Serial.println("MAVLink UART started");
    }
  actuators.setZero(); 

  // references FIXED for controller initialization
  // controller.setAttitudeReference(0.2f, 0.0f, 0.0f, 0.0f);
  // controller.setThrustReference(0.0f, 0.0f);               // Half thrust
  start_time = millis();

  // Run one loop before arming
  read_Pixhawk();
  float roll_cmd = 0.0f; 
  float pitch_cmd = 0.0f;
  controller.setAttitudeReference(roll_cmd, pitch_cmd, roll_rad, v_body[0]);
  controller.update(Eigen::Vector3f::Zero(), q_meas, omega_meas, v_body, u_sw_sym, last_u_ele, last_u_rud);
  actuators = controller.getActuatorOutputs();
  send_MAVLink_SERVO(actuators);

}


// --------- Main Loop -------------

void loop() {
  // 0. ARMING (you can send it through UDP now)
  // if (!is_armed){
  // armingSequence();}

  // 1) Read any incoming UDP packet (could be MoCap or ARM)
  int len = readUDPtoBuf();
  if (len > 0) {
    uint8_t hdr = udpBuf[0];
    // 2a) If it’s your ARM command, forward it:
    if (hdr == MAVLINK_STX   /*0xFE*/ || hdr == 0xFD /*0xFD = MAVLink v2 start byte*/) {    
        send_Arming_to_Pixhawk(udpBuf, len);}
    // 2b) If it’s MoCap:
    else{ 
      send_MoCAP_to_Pixhawk(reinterpret_cast<float*>(udpBuf));    }
  }


  // // 2. Always check for incoming MAVLink messages
  read_Pixhawk();

  // 3. Run control loop at fixed rate
  if (millis() - last_update >= update_interval_ms) {
      last_update = millis();
      //float t = millis() / 1000.0f;  // time in seconds for dynamic attitude reference
      float roll_cmd = 0.0f; //* sin(2 * M_PI * 0.2f * t);   // 0.2 rad amplitude at 0.2 Hz
      float pitch_cmd = 0.0f;
      controller.setAttitudeReference(roll_cmd, pitch_cmd, roll_rad, v_body[0]);
      controller.update(Eigen::Vector3f::Zero(), q_meas, omega_meas, v_body, u_sw_sym, last_u_ele, last_u_rud);
      actuators = controller.getActuatorOutputs();
    }
      // 4. Send RC override to Pixhawk
      send_MAVLink_SERVO(actuators);
      // 5. Debugging output to laptop (optional)
      // laptop_debug(actuators);
      // laptop_debug_QGC();

}

// --------functions called in loop-----------------
void armingSequence() {
  //[ref. https://mavlink.io/en/messages/common.html#HEARTBEAT]
  mavlink_message_t msg;
  mavlink_status_t  status;
  while (Serial1.available()) {
    uint8_t c = Serial1.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(&msg, &hb);
        bool nowArmed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
        if (nowArmed != is_armed) {
          is_armed = nowArmed;
          Serial.println("Vehicle is now ARMED");
        }
        else{ arm();}
      }
      Serial1.write(c); // forward raw MAVLink byte onto USB not to get stuck
    }
  }
}

void arm(){
  mavlink_command_long_t cmd;
  cmd.target_system = 1;
  cmd.target_component = 1;
  cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
  cmd.confirmation = 0;
  cmd.param1 = 1.0f;     // 1 to arm, 0 to disarm
  cmd.param2 = 21196.0f; // force arm (magic number)
  cmd.param3 = 0;
  cmd.param4 = 0;
  cmd.param5 = 0;
  cmd.param6 = 0;
  cmd.param7 = 0;

  mavlink_message_t arm_msg;
  mavlink_msg_command_long_encode(1, MAV_COMP_ID_ONBOARD_COMPUTER, &arm_msg, &cmd);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &arm_msg);
  Serial1.write(buf, len);
  Serial.println("ARM command sent from arm()");
  delay(1000); // Wait for the command to be processed
}


int readUDPtoBuf() {
  //returns the number of bytes read, or 0 if no packet
  int packetSize = Udp.parsePacket();
  if (packetSize > 0 && packetSize <= UDP_BUF_LEN) {
    return Udp.read((char*)udpBuf, packetSize);
  }
  return 0;
}

void send_Arming_to_Pixhawk(const uint8_t* buf, int len) {
  // Just mirror the exact MAVLink bytes onto Serial1
  Serial1.write(buf, len);
  Serial1.flush();
  Serial.printf("Forwarded ARM packet (%d bytes) to Pixhawk\n", len);
}


void send_MoCAP_to_Pixhawk(const float* data) {
  float posX = data[0], posY = data[1], posZ = data[2];
  float qx = data[3], qy = data[4], qz = data[5], qw = data[6];
  // Convert quaternion to roll, pitch, yaw  because PX4 expects these angles in VISION_POSITION_ESTIMATE
  float roll = atan2(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy));
  float pitch = asin(2.0f * (qw * qy - qz * qx));
  float yaw = atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));

  uint64_t timestamp_us = (millis() - start_time) * 1000ULL;

  mavlink_msg_vision_position_estimate_pack(
    2, 42, &msg,
    timestamp_us,
    posX, posY, posZ,
    roll, pitch, yaw,
    nullptr,
    0
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}


void read_Pixhawk() {
  /*  Read MAVLink messages from Pixhawk over Serial1 (UART). Generatethe full state of the drone usedby the controller:
  - Attitude quaternion 
  - Attitude in RPY angles (roll, pitch, yaw)
  - Angular velocity (rollspeed, pitchspeed, yawspeed)
  - Position (x, y, z)
  - Velocity in body frame (v_body)
  - last_u_ele and last_u_rud (elevator and rudder control outputs)
  */

  mavlink_status_t status;
  while (Serial1.available() > 0) {
    uint8_t c = Serial1.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

      if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE_QUATERNION) {
        mavlink_msg_attitude_quaternion_decode(&msg, &att_q);
        q_meas = Eigen::Quaternionf(att_q.q1, att_q.q2, att_q.q3, att_q.q4);
        omega_meas = Eigen::Vector3f(att_q.rollspeed, att_q.pitchspeed, att_q.yawspeed);
      }

      else if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
        mavlink_attitude_t att;
        mavlink_msg_attitude_decode(&msg, &att);
        roll_rad = att.roll;
        pitch_rad = att.pitch;
        yaw_rad = att.yaw;
    
      }

      else if (msg.msgid == MAVLINK_MSG_ID_ODOMETRY) {
        mavlink_odometry_t odom;
        mavlink_msg_odometry_decode(&msg, &odom);
    
        Eigen::Vector3f v_ned(odom.vx, odom.vy, odom.vz);
    
        ekf_pos[0] = odom.x;
        ekf_pos[1] = odom.y;
        ekf_pos[2] = odom.z;
    
        //Orientation update (if you trust ODOMETRY more than ATTITUDE_QUATERNION, uncomment the following)
        // q_meas = Eigen::Quaternionf(odom.q[0], odom.q[1], odom.q[2], odom.q[3]);
    
        Eigen::Matrix3f R = q_meas.toRotationMatrix();  // world-to-body
        v_body = R.transpose() * v_ned;    
      }
    

      // 3. Servo PWM outputs
      else if (msg.msgid == MAVLINK_MSG_ID_SERVO_OUTPUT_RAW) {
        mavlink_servo_output_raw_t servo;
        mavlink_msg_servo_output_raw_decode(&msg, &servo);

        // Read channels 1–5 (assuming actuators are on MAIN outputs)
        auto normalize_pwm = [](uint16_t pwm) -> float {
          float v = static_cast<float>(pwm); 
          return clamp((pwm - 1500.0f) / 500.0f, -1.0f, 1.0f);
        };

        last_u_ele    = normalize_pwm(servo.servo4_raw);  // Channel 4
        last_u_rud    = normalize_pwm(servo.servo5_raw);  // Channel 5
      
      }
    }
  }
}


void send_MAVLink_SERVO(Eigen::VectorXf& actuators) {
  uint16_t rc_override[18] = {0};

  auto mapToPWM = [](float cmd) -> uint16_t {
    float pwm = 1500.0f + cmd * 500.0f;
    pwm = clamp(pwm, 1000.0f, 2000.0f);
    return static_cast<uint16_t>(pwm);
  };

  for (int i = 0; i < 5 && i < actuators.size(); ++i) {
    if (i==1) {  // left sweep inversion 
      rc_override[i] = mapToPWM(-actuators[i]);
    } else {
      rc_override[i] = mapToPWM(actuators[i]);
    }
    last_rc_override[i] = actuators[i];  // Store the raw [-1,1] value for debug
  }
  // Serial.printf("RC_CHANNELS_OVERRIDE sent: ");
  // for (int i = 0; i < 5; ++i) {
  //   Serial.printf("%.2f ", last_rc_override[i]);
  // }
  // Serial.println();
  // Serial.printf("SERVO raw values sent: ");
  // for (int i = 0; i < 5; ++i) {
  //   Serial.printf("%f", actuators[i]);
  // }
  // Serial.println();

  mavlink_message_t servo_msg;
  mavlink_msg_rc_channels_override_pack(
    2, 42, &servo_msg,
    1, 1,
    rc_override[0], rc_override[1], rc_override[2], rc_override[3],
    rc_override[4], rc_override[5], rc_override[6], rc_override[7],
    rc_override[8], rc_override[9], rc_override[10], rc_override[11],
    rc_override[12], rc_override[13], rc_override[14], rc_override[15],
    rc_override[16], rc_override[17]
  );

  uint8_t temp_buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(temp_buf, &servo_msg);
  Serial1.write(temp_buf, len);
}






void laptop_debug(const Eigen::VectorXf& actuators) {
  // Build packet: [roll, pitch, yaw, p, q, r, x, y, z, vx, vy, vz, u0, u1, u2, u3, u4]
  float packet[18] = {
    roll_rad, // from ATTITUDE msg
    pitch_rad,
    yaw_rad,

    omega_meas[0], // from ATTITUDE_QUATERNION msg
    omega_meas[1],
    omega_meas[2],

    ekf_pos[0], // from LOCAL_POSITION_NED msg
    ekf_pos[1],
    ekf_pos[2],

    v_body[0], //from LOCAL_POSITION_NED msg transformed to body frame
    v_body[1],
    v_body[2],

    actuators[0], // TODO: this is the command sent to RC before mapping, change it to the one read on the actuators
    actuators[1],
    actuators[2],
    actuators[3],
    actuators[4],
    0.0f  // Padding or future use
  };

  // Send 18 floats = 72 bytes
  ElisalaptopUdp.beginPacket(ElisalaptopIP, ElisalaptopPort);
  ElisalaptopUdp.write((uint8_t*)packet, sizeof(packet));
  ElisalaptopUdp.endPacket();
}

// void laptop_debug_QGC() {
//   // 1) pack into 'msg' — note the extra nullptr at the end
//   mavlink_msg_attitude_quaternion_pack(
//       /* system_id */      1,
//       /* component_id */   MAV_COMP_ID_ONBOARD_COMPUTER,
//       /* message out */    &msg,
//       /* time_boot_ms */   micros()/1000,
//       /* quaternion */     att_q.q1, att_q.q2, att_q.q3, att_q.q4,
//       /* rates */          att_q.rollspeed, att_q.pitchspeed, att_q.yawspeed,
//       /* extra fields */   nullptr
//   );

//   // 2) serialize
//   uint8_t buf[MAVLINK_MAX_PACKET_LEN];
//   uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

//   // 3) send
//   QGC.beginPacket(laptopIP, QGCport);
//   QGC.write(buf, len);
//   QGC.endPacket();
// }

void laptop_debug_QGC() {
  // 1) Send attitude_quaternion (already present)
  mavlink_msg_attitude_quaternion_pack(
      1, MAV_COMP_ID_ONBOARD_COMPUTER, &msg,
      micros()/1000,
      att_q.q1, att_q.q2, att_q.q3, att_q.q4,
      att_q.rollspeed, att_q.pitchspeed, att_q.yawspeed,
      nullptr
  );
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  QGC.beginPacket(laptopIP, QGCport);
  QGC.write(buf, len);
  QGC.endPacket();

  // 2) Send RC override values as a custom float array
  float rc_debug[5];
  for (int i = 0; i < 5; ++i)
    rc_debug[i] = last_rc_override[i];  // values in [-1, 1]

  QGC.beginPacket(laptopIP, QGCport);
  QGC.write((uint8_t*)rc_debug, sizeof(rc_debug));
  QGC.endPacket();

  // 3) Send servo output raw as uint16_t values
  QGC.beginPacket(ElisalaptopIP, QGCport);
  QGC.write((uint8_t*)last_servo_raw, sizeof(last_servo_raw));
  QGC.endPacket();
}

