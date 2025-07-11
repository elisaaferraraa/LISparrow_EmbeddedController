/*
 * =============================================================================
 * Low Level control for LISparrow UAV, with connection to MoCap and Pixhawk, and laptop debug.
 *
 * This firmware runs on an ESP32 and performs the following tasks:
 *
 * 1. Connects to a WiFi network
 * 2. Parses MoCap data or arming command
 * 3. Sends the data to Pixhawk as MAVLink VISION_POSITION_ESTIMATE messages
 * 4. Receives state (quaternion attitude, angular velocity, position, velocity, previous servo commands)
 * from Pixhawk over Serial (UART)
 * 5. Feeds state into an AgileCascadedPID controller
 *    to compute 5 control outputs: thrust, sweep left, sweep right, elevator, rudder
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
const char *ssid = "WiFli";
const char *password = "crazyflie";
WiFiUDP Udp;
unsigned int localPort = 8888;

const char *laptopIP = "192.168.194.114"; // <-- Your laptop IP on same network as ESP32
const int laptopPort = 9999;              // <-- Choose an unused UDP port for feedback
WiFiUDP laptopUdp;

const char *ElisalaptopIP = "192.168.194.114"; // <-- Your laptop IP on same network as ESP32
const int ElisalaptopPort = 9998;              // <-- Choose an unused UDP port for feedback
WiFiUDP ElisalaptopUdp;                        // used for debug stream sent over UDP to laptop and retrieved with telemetry:UPD_receive.py
const int QGCport = 14550;                     // <-- Choose an unused UDP port for feedback
WiFiUDP QGC;

// ====== MAVLink UART Configuration ======
#define RX_PIN D7 // Pixhawk TX → Xiao RX
#define TX_PIN D6 // Pixhawk RX ← Xiao TX
#define BAUD 230400

// ====== Globals ======
const int led = D10; // for ESP32C3, for ESP32S3 use LED_BUILTIN
// MavLink
mavlink_message_t msg;
mavlink_attitude_quaternion_t att_q;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
uint64_t start_time = 0;
float mocap_data[7] = {0};
Eigen::VectorXf actuators(5);
Eigen::Vector3f ekf_pos(0.0f, 0.0f, 0.0f);
bool is_armed = false;           // Flag to request arming
constexpr int UDP_BUF_LEN = 300; // Maximum UDP packet size we expect (MAVLink max is ~280 bytes)
uint8_t udpBuf[UDP_BUF_LEN];

// Controller
AgileCascadedPID controller(0.02f); // 50 Hz loop
Eigen::Quaternionf q_meas = Eigen::Quaternionf::Identity();
Eigen::Vector3f omega_meas = Eigen::Vector3f::Zero();
Eigen::Vector3f v_body = Eigen::Vector3f::Zero();
float u_sw_sym = -0.25f;
float last_u_ele = 0.0f;
float last_u_rud = 0.0f;
float roll_rad = 0.0f; // Roll angle in radians read after EKF
float pitch_rad = 0.0f;
float yaw_rad = 0.0f;
const unsigned long update_interval_ms = 20; // frequency at which controller runs (50 Hz)
// WARNING: if you want to change it you need also to change accordingly the controller update rate in AgileCascadedPID when you generate the controller object
unsigned long last_update = 0;

// Debug
uint16_t last_servo_raw[5] = {0};
float last_rc_override[5] = {0.0f};

// =============================================================================
void setup()
{
  pinMode(led, OUTPUT); //(ESP32C3)
  // pinMode(LED_BUILTIN, OUTPUT);  // Initialize LED pin as output (ESP32S3)
  Serial.begin(115200);
  // digitalWrite(LED_BUILTIN, HIGH);  // LED OFF before WiFi is connected (ESP32S3)
  digitalWrite(led, HIGH); // LED OFF before WiFi is connected (ESP32C3)

  WiFi.setSleep(WIFI_PS_NONE); // full battery
  initWiFi();

  // UDP CONNECTIONS SETUP
  Udp.begin(localPort);        // for reading MoCap and arm command
  laptopUdp.begin(laptopPort); // for sending to laptop specific info
  QGC.begin(QGCport);          // for connecting to QGC
  Serial.println("UDP listening...");

  // UART CONNECTION SETUP
  Serial1.begin(BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  if (Serial1.available())
  {
    Serial.println("MAVLink UART started");
  }
  actuators.setZero();

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

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    WiFireconnect();
  }
  // 0. ARMING (you can send it through UDP now)
  // if (!is_armed){
  // armingSequence();}

  // 1) Read any incoming UDP packet (could be MoCap or ARM)
  int len = readUDPtoBuf();
  if (len > 0)
  {
    uint8_t hdr = udpBuf[0];
    // 2a) If it’s your ARM command, forward it:
    if (hdr == MAVLINK_STX /*0xFE*/ || hdr == 0xFD /*0xFD = MAVLink v2 start byte*/)
    {
      send_Arming_to_Pixhawk(udpBuf, len);
    }
    // 2b) If it’s MoCap:
    else
    {
      send_MoCAP_to_Pixhawk(reinterpret_cast<float *>(udpBuf));
    }
  }

  // // 2. Always check for incoming MAVLink messages
  read_Pixhawk();

  // 3. Run control loop at fixed rate
  if (millis() - last_update >= update_interval_ms)
  {
    last_update = millis();
    // float t = millis() / 1000.0f;  // time in seconds for dynamic attitude reference
    float roll_cmd = 0.0f; //* sin(2 * M_PI * 0.2f * t);   // 0.2 rad amplitude at 0.2 Hz
    float pitch_cmd = 0.0f;
    controller.setAttitudeReference(roll_cmd, pitch_cmd, roll_rad, v_body[0]);
    controller.update(Eigen::Vector3f::Zero(), q_meas, omega_meas, v_body, u_sw_sym, last_u_ele, last_u_rud);
    actuators = controller.getActuatorOutputs();
  }
  // 4. Send RC override to Pixhawk
  send_MAVLink_SERVO(actuators);
  // 5. Debugging output to laptop (optional) - uncomment them to run faster
  laptop_debug(actuators);
  laptop_debug_QGC();

  delay(10); // 100 Hz
}

// =============================================================================
// --------Functions called in loop-----------------
void initWiFi()
{
  //  WIFI SETUP
  // if wifi connection does not work try to shortcut the RST button (i.e. press it) to reset the ESP32
  WiFi.mode(WIFI_STA); // Set WiFi mode to Station (client)
  WiFi.disconnect();
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    Serial.println(WiFi.status()); // When it prints "0" it means that it is going to connect in a few seconds"
    delay(500);
  }
  // digitalWrite(LED_BUILTIN, LOW);  // LED ON when WiFi is connected (ESP32S3)
  digitalWrite(led, LOW); // LED ON when WiFi is connected (ESP32C3)
  Serial.println("\nWiFi connected");
}

void WiFireconnect()
{
  // if WiFi is down, try reconnecting
  while ((WiFi.status() != WL_CONNECTED))
  {
    Serial.println(".");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    delay(30000); // 30 seconds before retrying
  }
}
void armingSequence()
{
  //[ref. https://mavlink.io/en/messages/common.html#HEARTBEAT]
  mavlink_message_t msg;
  mavlink_status_t status;
  while (Serial1.available())
  {
    uint8_t c = Serial1.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
      if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT)
      {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(&msg, &hb);
        bool nowArmed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
        if (nowArmed != is_armed)
        {
          is_armed = nowArmed;
          Serial.println("Vehicle is now ARMED");
        }
        else
        {
          arm();
        }
      }
      Serial1.write(c); // forward raw MAVLink byte onto USB not to get stuck
    }
  }
}

void arm()
{
  mavlink_command_long_t cmd;
  cmd.target_system = 1;
  cmd.target_component = 1;
  cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
  cmd.confirmation = 0;
  cmd.param1 = 1.0f;     // 1 to arm, 0 to disarm
  cmd.param2 = 21196.0f; // force arm
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
  delay(1000);
}

int readUDPtoBuf()
{
  // returns the number of bytes read, or 0 if no packet
  int packetSize = Udp.parsePacket();
  if (packetSize > 0 && packetSize <= UDP_BUF_LEN)
  {
    return Udp.read((char *)udpBuf, packetSize);
  }
  return 0;
}

void send_Arming_to_Pixhawk(const uint8_t *buf, int len)
{
  // Just mirror the exact MAVLink bytes onto Serial1
  Serial1.write(buf, len);
  Serial1.flush();
  Serial.printf("Forwarded ARM packet (%d bytes) to Pixhawk\n", len);
}

void send_MoCAP_to_Pixhawk(const float *data)
{
  float posX = data[0], posY = data[1], posZ = data[2];
  float qx = data[3], qy = data[4], qz = data[5], qw = data[6];
  // Convert quaternion to roll, pitch, yaw  because PX4 expects these angles in VISION_POSITION_ESTIMATE message
  // ref: https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
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
      0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

void read_Pixhawk()
{
  /*  Read MAVLink messages from Pixhawk over Serial1 (UART). Generatethe full state of the drone usedby the controller:
  - Attitude quaternion
  - Attitude in RPY angles (roll, pitch, yaw)
  - Angular velocity (rollspeed, pitchspeed, yawspeed)
  - Position (x, y, z)
  - Velocity in body frame (v_body)
  - last_u_ele and last_u_rud (elevator and rudder control outputs)
  */

  mavlink_status_t status;
  while (Serial1.available() > 0)
  {
    uint8_t c = Serial1.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {

      if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE_QUATERNION)
      {
        mavlink_msg_attitude_quaternion_decode(&msg, &att_q);
        q_meas = Eigen::Quaternionf(att_q.q1, att_q.q2, att_q.q3, att_q.q4);
        omega_meas = Eigen::Vector3f(att_q.rollspeed, att_q.pitchspeed, att_q.yawspeed);
      }

      else if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE)
      {
        mavlink_attitude_t att;
        mavlink_msg_attitude_decode(&msg, &att);
        roll_rad = att.roll;
        pitch_rad = att.pitch;
        yaw_rad = att.yaw;
      }

      else if (msg.msgid == MAVLINK_MSG_ID_ODOMETRY)
      {
        mavlink_odometry_t odom;
        mavlink_msg_odometry_decode(&msg, &odom);

        Eigen::Vector3f v_ned(odom.vx, odom.vy, odom.vz);

        ekf_pos[0] = odom.x;
        ekf_pos[1] = odom.y;
        ekf_pos[2] = odom.z;

        // Orientation update (if you trust ODOMETRY more than ATTITUDE_QUATERNION, uncomment the following)
        //  q_meas = Eigen::Quaternionf(odom.q[0], odom.q[1], odom.q[2], odom.q[3]);

        Eigen::Matrix3f R = q_meas.toRotationMatrix(); // world-to-body
        v_body = R.transpose() * v_ned;
      }

      // 3. Servo PWM outputs
      else if (msg.msgid == MAVLINK_MSG_ID_SERVO_OUTPUT_RAW)
      {
        mavlink_servo_output_raw_t servo;
        mavlink_msg_servo_output_raw_decode(&msg, &servo);

        // Read channels 1–5 (assuming actuators are on MAIN outputs)
        auto normalize_pwm = [](uint16_t pwm) -> float
        {
          float v = static_cast<float>(pwm);
          return clamp((pwm - 1500.0f) / 500.0f, -1.0f, 1.0f);
        };

        last_u_ele = normalize_pwm(servo.servo4_raw); // Channel 4
        last_u_rud = normalize_pwm(servo.servo5_raw); // Channel 5
      }
    }
  }
}

void send_MAVLink_SERVO(Eigen::VectorXf &actuators)
{
  uint16_t rc_override[18] = {0};

  auto mapToPWM = [](float cmd) -> uint16_t
  {
    float pwm = 1500.0f + cmd * 500.0f;
    pwm = clamp(pwm, 1000.0f, 2000.0f);
    return static_cast<uint16_t>(pwm);
  };

  for (int i = 0; i < 5 && i < actuators.size(); ++i)
  {
    if (i == 1)
    { // left sweep inversion
      rc_override[i] = mapToPWM(-(actuators[i] + 0.25f) - 0.25f);
    }
    else
    {
      rc_override[i] = mapToPWM(actuators[i]);
    }
    last_rc_override[i] = actuators[i]; // Store the raw [-1,1] value for debug
  }

  mavlink_message_t servo_msg;
  mavlink_msg_rc_channels_override_pack(
      2, 42, &servo_msg,
      1, 1,
      rc_override[0], rc_override[1], rc_override[2], rc_override[3],
      rc_override[4], rc_override[5], rc_override[6], rc_override[7],
      rc_override[8], rc_override[9], rc_override[10], rc_override[11],
      rc_override[12], rc_override[13], rc_override[14], rc_override[15],
      rc_override[16], rc_override[17]);

  uint8_t temp_buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(temp_buf, &servo_msg);
  Serial1.write(temp_buf, len);
}

// -------------------Debugging functions-------------------
void laptop_debug(const Eigen::VectorXf &actuators)
{
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

      v_body[0], // from LOCAL_POSITION_NED msg transformed to body frame
      v_body[1],
      v_body[2],

      actuators[0], // This is the command sent to RC before mapping, change it to the one read on the actuators, if you want
      actuators[1],
      actuators[2],
      actuators[3],
      actuators[4],
      0.0f // Padding or future use
  };

  ElisalaptopUdp.beginPacket(ElisalaptopIP, ElisalaptopPort);
  ElisalaptopUdp.write((uint8_t *)packet, sizeof(packet));
  ElisalaptopUdp.endPacket();
}

void laptop_debug_QGC()
{
  send_quaternion_to_QGC();
  send_RC_channels_to_QGC(last_rc_override, 5, 0);
  // last_servo_raw must be uint16_t[<=16]
  send_servo_outputs_to_QGC(last_servo_raw, 6);
}

void send_quaternion_to_QGC()
{
  uint32_t t_ms = (uint32_t)(micros() / 1000);
  float thrust_val = 0.0f; // if you don’t have thrust data

  mavlink_msg_attitude_quaternion_pack(
      /*sysid*/ 1,
      /*compid*/ MAV_COMP_ID_ONBOARD_COMPUTER,
      /*msg*/ &msg,
      /*time_boot_ms*/ t_ms,
      /*q1..q4*/ att_q.q1, att_q.q2, att_q.q3, att_q.q4,
      /*rates*/ att_q.rollspeed, att_q.pitchspeed, att_q.yawspeed,
      /*thrust ptr*/ &thrust_val);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  QGC.beginPacket(laptopIP, QGCport);
  QGC.write(buf, len);
  QGC.endPacket();
}

void send_RC_channels_to_QGC(const float *rc_in_f, uint8_t chan_count, uint8_t rssi)
{
  uint16_t rc[8] = {0};
  for (uint8_t i = 0; i < chan_count && i < 8; i++)
  {
    rc[i] = (uint16_t)rc_in_f[i];
  }

  uint32_t now_ms = (uint32_t)(micros() / 1000);

  mavlink_msg_rc_channels_raw_pack(
      /*sysid*/ 1,
      /*compid*/ MAV_COMP_ID_ONBOARD_COMPUTER,
      /*msg*/ &msg,
      /*time_boot_ms*/ now_ms,
      /*port*/ 0,
      /*chan1*/ rc[0], /*chan2*/ rc[1],
      /*chan3*/ rc[2], /*chan4*/ rc[3],
      /*chan5*/ rc[4], /*chan6*/ rc[5],
      /*chan7*/ rc[6], /*chan8*/ rc[7],
      /*rssi*/ rssi);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  QGC.beginPacket(laptopIP, QGCport);
  QGC.write(buf, len);
  QGC.endPacket();
}

void send_servo_outputs_to_QGC(const uint16_t *servo_pwm, uint8_t servo_count)
{
  uint16_t pwm[16] = {0};
  for (uint8_t i = 0; i < servo_count && i < 16; i++)
  {
    pwm[i] = servo_pwm[i];
  }

  uint32_t now_us = (uint32_t)micros();

  mavlink_msg_servo_output_raw_pack(
      /*sysid*/ 1,
      /*compid*/ MAV_COMP_ID_ONBOARD_COMPUTER,
      /*msg*/ &msg,
      /*time_usec*/ now_us,
      /*port*/ 0,
      /*servo1*/ pwm[0], /*2*/ pwm[1], /*3*/ pwm[2], /*4*/ pwm[3],
      /*5*/ pwm[4], /*6*/ pwm[5], /*7*/ pwm[6], /*8*/ pwm[7],
      /*9*/ pwm[8], /*10*/ pwm[9], /*11*/ pwm[10], /*12*/ pwm[11],
      /*13*/ pwm[12], /*14*/ pwm[13], /*15*/ pwm[14], /*16*/ pwm[15]);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  QGC.beginPacket(laptopIP, QGCport);
  QGC.write(buf, len);
  QGC.endPacket();
}
