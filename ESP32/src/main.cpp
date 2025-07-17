#include <Arduino.h>
#include <WiFi.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define MAX_RPM 98
#include <std_msgs/UInt8.h>

#include "HX711.h"

#include <math.h>

#include <Arduino.h>
#include <ESP32Servo.h>

Servo myServo;           // Create a Servo object
//const int servoPin = 14; // Use GPIO14 for servo signal
const int stepDegrees = 20;
const int maxAngle = 180;

const int delayTime = 1000; // 1 second delay

unsigned long last_action_time_3 = 0;   // Global or static variable
unsigned long delay_duration_3 = 3000;  // 2000 ms = 2 seconds

unsigned long last_action_time_4 = 0;   // Global or static variable
unsigned long delay_duration_4 = 3000;  // 2000 ms = 2 seconds

float counter = 0;

// MPU6050
Adafruit_MPU6050 mpu;
float yaw = 0;
float gyro_z_bias = 0;

unsigned long last_time_obst = millis();
float x = 4;
float y = 2;
float angular_theta = 0;
unsigned long last_time = 0;

float last_linear_x = 0;
bool cmd_received = 0;
bool motor_command_received = 0;
bool keyboard_command_received = 0;

double x_pose = 0;
double y_pose = 0;

// Motor Pins
#define BR_IN1 33
#define BR_IN2 25
#define BR_PWM 32
//#define BR_MENCG 34
//#define BR_MENCY 35

#define FR_IN1 26
#define FR_IN2 27
#define FR_PWM 12
//#define FR_MENG 12
//#define FR_MENY 15

#define BL_IN1 5
#define BL_IN2 18
#define BL_PWM 19
//#define BL_MENG 14
//#define BL_MENCY 23

#define FL_IN1 16
#define FL_IN2 17
#define FL_PWM 4
//#define FL_MENG 19
//#define FL_MENCY 32

#define WHEELBASE 0.24
#define WHEEL_RADIUS 0.06

#define WHEEL_DIAMETER_MM 130
#define ENCODER_TICKS_PER_REV 2000
#define PI 3.14159265359

#define LEFT_ENC_A 23
#define LEFT_ENC_B 15

#define RIGHT_ENC_A 35
#define RIGHT_ENC_B 34

// HX711 Load Cell
//#define LOADCELL_DOUT_PIN  21
//#define LOADCELL_SCK_PIN   22

// --- Encoder counters ---
volatile long left_ticks = 0;
volatile long right_ticks = 0;

float m_command = 6;
float key_command = 6;

float linear_x = 0;
float angular_z = 0;
float obstacle = 0;

float last_angular_theta = 0;
float last_ticks = 0;

float compare_linear = 180;
float compare_angular = 180;

bool last_rotated_is_right = 0;
bool last_rotated_is_left = 0;

bool rotating_time = 0;

//Linear
int forward_en = 0;

float PWM_teleop = 150;

// HX711 Load Cell
#define LOADCELL_DOUT_PIN  13
#define LOADCELL_SCK_PIN   14
HX711 scale;
float load_cell_threshold = 30; // 1kg threshold

// WiFi credentials
const char* ssid = "Ahmed";
const char* password = "kokamido12";
IPAddress server(192,168,43,214); // ROS master IP 192.168.43.79
uint16_t serverPort = 11411; // ROS master port 192.168.43.214

// ROS node handle and messages
ros::NodeHandle nh;
geometry_msgs::Pose2D pose_msg;
ros::Publisher pose_pub("robot_pose", &pose_msg);

// --- Function Declarations ---
void updateIMUData();
void publishPose(float x_pos, float y_pos, float theta);
void setAllMotors(int fl_pwm, int bl_pwm, int fr_pwm, int br_pwm);
void handleObstacle(int obstacle_type, unsigned long duration);
void maintainConnection(unsigned long duration);
void handleMotorCommands();
void handleRotation();
void setupMotorPins();
void setupIMU();
void connectToWiFi();

void cmdVelCallback(const geometry_msgs::Twist& cmd_vel) {
    linear_x = (cmd_vel.linear.x) * 1000;    
    obstacle = cmd_vel.linear.y;
    angular_z = cmd_vel.angular.z; 

    if (linear_x != last_linear_x) {
        rotating_time = 0;
    }

    if (linear_x != 0 || angular_z != 0) {
        cmd_received = true;
        motor_command_received = false;
        keyboard_command_received = false;
    }
}

void cmdVelCallback_teleop(const std_msgs::Int32& motor_command) {
    m_command = motor_command.data;

    if (m_command != 6) {
        motor_command_received = true;
        keyboard_command_received = false;
        cmd_received = false;
    }
}

void cmdVelCallback_keyboard_teleop(const std_msgs::Int32& keyboard_input) {
    key_command = keyboard_input.data;

    if (key_command != 6) {
        keyboard_command_received = true;
        motor_command_received = false;
        cmd_received = false;
    }
}

// --- Encoder ISRs ---
void IRAM_ATTR leftISR() {
    if (digitalRead(LEFT_ENC_B) == HIGH && forward_en == 1)
        left_ticks++;
}

void IRAM_ATTR rightISR() {
    if (digitalRead(RIGHT_ENC_B) == HIGH && forward_en == 1)
        right_ticks++;
}

void rotate_right_pins() {
    digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
    digitalWrite(BL_IN1, LOW); digitalWrite(BL_IN2, HIGH);
    digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
    digitalWrite(BR_IN1, HIGH); digitalWrite(BR_IN2, LOW);
}

void rotate_left_pins() {
    digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
    digitalWrite(BL_IN1, HIGH); digitalWrite(BL_IN2, LOW);
    digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
    digitalWrite(BR_IN1, LOW); digitalWrite(BR_IN2, HIGH);
}

void forward_pins() {
    digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
    digitalWrite(BL_IN1, LOW); digitalWrite(BL_IN2, HIGH);
    digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
    digitalWrite(BR_IN1, LOW); digitalWrite(BR_IN2, HIGH);
}

void backward_pins() {
    digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
    digitalWrite(BL_IN1, HIGH); digitalWrite(BL_IN2, LOW);
    digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
    digitalWrite(BR_IN1, HIGH); digitalWrite(BR_IN2, LOW);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmdVelCallback);
ros::Subscriber<std_msgs::Int32> sub1("motor_command", &cmdVelCallback_teleop);
ros::Subscriber<std_msgs::Int32> sub2("keyboard_input", &cmdVelCallback_keyboard_teleop);

void setup() {
    Serial.begin(9600);
    connectToWiFi();
    setupIMU();
    setupMotorPins();

    // ROS setup
    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();
    delay(10);
    nh.subscribe(sub);
    nh.subscribe(sub1);
    nh.subscribe(sub2);
    nh.advertise(pose_pub);
    last_time = millis();

    // Encoder pin setup
    pinMode(LEFT_ENC_A, INPUT_PULLUP);
    pinMode(LEFT_ENC_B, INPUT_PULLUP);
    pinMode(RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftISR, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightISR, RISING);
      // Initialize load cell
      scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
      scale.set_scale(425); // You need to set your calibration factor
      scale.tare(); // Reset the scale to 0

    //myServo.attach(servoPin);  // Attach the servo to GPIO14
}

// [Previous includes and #define directives remain exactly the same...]

void loop() {
  nh.spinOnce();
  updateIMUData();

      // Check load cell
    float weight = scale.get_units(10); // Get weight in kg

  pose_msg.x = weight;
  pose_msg.y = load_cell_threshold;
  pose_pub.publish(&pose_msg);
  
  delay(10); // Small delay between load cell readings

  last_linear_x = linear_x;

  long target_ticks = ((linear_x / (PI * WHEEL_DIAMETER_MM)) * ENCODER_TICKS_PER_REV) / 2.35;
  last_ticks = target_ticks;

  if(rotating_time == 0) {
      angular_theta = 90 - (angular_z * (180.0 / PI));
      last_angular_theta = angular_theta;
  }
  else if(rotating_time == 1) {
      angular_theta = last_angular_theta - 180;
      target_ticks = 2 * last_ticks;

      if (counter == 2) {
          angular_theta = 0;
      }
  }

  if (motor_command_received == true || keyboard_command_received == true) {
      handleMotorCommands();
  }

  if(cmd_received == true && motor_command_received == false) {
      if (obstacle == 2) {  
          handleObstacle(2, 4000);
          last_rotated_is_right = true;
          last_rotated_is_left = false;
      }  
      else if (obstacle == 1) {
          handleObstacle(1, 4000);
          last_rotated_is_left = true;
          last_rotated_is_right = false;
      }

      if (abs(yaw - angular_theta) > 5.0) {
          handleRotation();
      } 
      else {
          forward_en = 1;
          if ((abs(left_ticks) + abs(right_ticks)) / 2 < target_ticks) {
              x_pose = linear_x * cos(angular_z);
              y_pose = linear_x * sin(angular_z);
              publishPose(x_pose, y_pose, yaw);
              
              forward_pins();
              setAllMotors(255, 255, 255, 255);
          }
          else {
              // Goal reached - original return-to-home logic
              forward_pins();
              setAllMotors(0, 0, 0, 0);
              
              maintainConnection(7000);
              
              rotating_time = 1;
              counter++;
              compare_linear = linear_x;
              compare_angular = angular_z;

              // Original return-to-home logic
              while(counter == 3) {
                  nh.spinOnce();
                  updateIMUData();
                  publishPose(0, 0, yaw);
                  
                  if(compare_linear != linear_x || compare_angular != angular_z) {
                      counter = 0;
                      rotating_time = 0;
                      left_ticks = 0;
                      right_ticks = 0;
                  }
              }

              // Original servo control logic (commented out in original)
              /*
              if(target_ticks == 5599.0) {
                  myServo.write(0);
                  for (int angle = 0; angle <= maxAngle; angle += stepDegrees) {
                      myServo.write(angle);
                      Serial.print("Moved to angle: ");
                      Serial.println(angle);

                      unsigned long t_start_rotate_shaft = millis();
                      while ((millis() - t_start_rotate_shaft) < 1000) {
                          nh.spinOnce();   // Keep ROS alive
                          delay(1);        // Allow ESP32 WiFi background tasks
                      }
                  }
              }
              */
          }
      }
  }
  delay(10);
}

// [Rest of the function implementations remain the same...]

void updateIMUData() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time) / 1000.0;
    last_time = current_time;

    float gyro_z = g.gyro.z - gyro_z_bias;
    yaw += gyro_z * delta_time * (180.0 / PI);
    
    // Normalize yaw to -180 to 180 degrees
    if (yaw > 180) yaw -= 360;
    if (yaw < -180) yaw += 360;


}

void publishPose(float x_pos, float y_pos, float theta) {
    /*
    pose_msg.x = x_pos;
    pose_msg.y = y_pos;
    pose_msg.theta = theta;
    pose_pub.publish(&pose_msg);
    */

}

void setAllMotors(int fl_pwm, int bl_pwm, int fr_pwm, int br_pwm) {
    analogWrite(FL_PWM, fl_pwm);
    analogWrite(BL_PWM, bl_pwm);
    analogWrite(FR_PWM, fr_pwm);
    analogWrite(BR_PWM, br_pwm);
}

void handleObstacle(int obstacle_type, unsigned long duration) {
    forward_en = 1;
    forward_pins();
    
    if(obstacle_type == 2) {
        setAllMotors(240, 240, 0, 70);
    } else {
        setAllMotors(0, 70, 255, 255);
    }

    unsigned long t_start = millis();
    while (millis() - t_start < duration) {
        nh.spinOnce();
        updateIMUData();
        publishPose(x, y, yaw);
        delay(1);
    }
}

void maintainConnection(unsigned long duration) {
  unsigned long t_start = millis();
  while ((millis() - t_start) < duration) {
      nh.spinOnce();      // Keep processing ROS messages
      updateIMUData();    // Keep updating sensor data
      
      
      // Check load cell
      float weight = scale.get_units(10); // Get weight in kg
      if (weight >= load_cell_threshold) {
          Serial.print("Weight detected: ");
          Serial.print(weight);
          Serial.println(" kg - Ending wait period");
          return; // Exit the function early if weight threshold is met
      }

      pose_msg.x = weight;
      pose_msg.y = load_cell_threshold;
      pose_pub.publish(&pose_msg);
      
      delay(10); // Small delay between load cell readings
  }
}

void handleMotorCommands() {
    if(m_command == 0 || key_command == 0) { // STOP
        setAllMotors(0, 0, 0, 0);
    }
    else if(m_command == 1 || key_command == 1) { // FORWARD
        forward_pins();
        setAllMotors(150, 150, 150, 150);
    }
    else if(m_command == 2 || key_command == 2) { // RIGHT
        digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
        digitalWrite(BL_IN1, LOW); digitalWrite(BL_IN2, HIGH);
        digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
        digitalWrite(BR_IN1, LOW); digitalWrite(BR_IN2, HIGH);
        setAllMotors(240, 240, 240, 70);
    }
    else if(m_command == 3 || key_command == 3) { // LEFT
        digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
        digitalWrite(BL_IN1, LOW); digitalWrite(BL_IN2, HIGH);
        digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
        digitalWrite(BR_IN1, LOW); digitalWrite(BR_IN2, HIGH);
        setAllMotors(240, 70, 240, 240);
    }
    else if(m_command == 4 || key_command == 4) { // SPEED UP
        forward_pins();
        PWM_teleop = PWM_teleop + 20;
        if(PWM_teleop >= 240) PWM_teleop = 140;
        setAllMotors(PWM_teleop, PWM_teleop, PWM_teleop, PWM_teleop);
    }
    else if(m_command == 5 || key_command == 5) { // REVERSE
        backward_pins();
        setAllMotors(150, 150, 150, 150);
    }
}

void handleRotation() {
    forward_en = 0;
    if(last_rotated_is_right == true && last_rotated_is_left == false) {
        rotate_left_pins();
    }
    else if(last_rotated_is_left == true && last_rotated_is_right == false) {
        rotate_right_pins();
    }
    else {
        rotate_left_pins();
    }
    setAllMotors(255, 255, 255, 255);
    publishPose(0, 0, yaw);
}

void setupMotorPins() {
    pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT);
    pinMode(BL_IN1, OUTPUT); pinMode(BL_IN2, OUTPUT);
    pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT);
    pinMode(BR_IN1, OUTPUT); pinMode(BR_IN2, OUTPUT);

    digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
    digitalWrite(BL_IN1, LOW); digitalWrite(BL_IN2, HIGH);
    digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
    digitalWrite(BR_IN1, LOW); digitalWrite(BR_IN2, HIGH);
}

void setupIMU() {
    if (!mpu.begin()) {

        while (1) {
            delay(10);
        }
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Calculate gyro bias
    float total = 0;
    for (int i = 0; i < 100; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        total += g.gyro.z;
        delay(10);
    }
    gyro_z_bias = total / 100.0;
}

void connectToWiFi() {
    WiFi.begin(ssid, password);
    WiFi.setSleep(false);  
    WiFi.setAutoReconnect(true); 
    WiFi.persistent(true);       

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);

        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {

    } else {

        ESP.restart();
    }
}
