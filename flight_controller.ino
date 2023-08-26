#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#define I2C_CLK_FREQ 400000
const u_int8_t IMUAddress = 0x68;

#ifndef APSSID
#define APSSID "W"
#define APPSW "password"
#endif
#define UDP_PKT_MAX_SIZE 16

unsigned int localPort = 8888;
char packetBuffer[UDP_PKT_MAX_SIZE + 1];

WiFiUDP Udp;

#define MOT_TOP_LEFT 18
#define MOT_TOP_RIGHT 13
#define MOT_BOTTOM_LEFT 28
#define MOT_BOTTOM_RIGHT 1
#define sensitivity 2

float pid_p_gain_roll = 1.3;
float pid_i_gain_roll = 0.013;
float pid_d_gain_roll = 14.0;
int pid_max_roll = 300;

float pid_p_gain_pitch = pid_p_gain_roll;
float pid_i_gain_pitch = pid_i_gain_roll;
float pid_d_gain_pitch = pid_d_gain_roll;
int pid_max_pitch = pid_max_roll;

float pid_p_gain_yaw = 8.5;
float pid_i_gain_yaw = 0.005;
float pid_d_gain_yaw = 0.0;
int pid_max_yaw = 300;

boolean auto_level = true;

byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte highByte, lowByte;
volatile int receiver_input_channel_1 = 0, receiver_input_channel_2 = 0, receiver_input_channel_3 = 0, receiver_input_channel_4 = 0;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
int cal_int, start, gyro_address;
int receiver_input[5];
int temperature;
int16_t acc_axis[4], gyro_axis[4];
float roll_level_adjust, pitch_level_adjust;

long acc_x, acc_y, acc_z, acc_total_vector;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[4];
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
boolean gyro_angles_set;

void setup(){

  analogWriteFreq(500);
  analogWriteRange(1000);

  Serial.begin(115200);
  gyro_address = IMUAddress;
  Wire.setClock(I2C_CLK_FREQ);
  Wire.begin();

  pinMode(MOT_TOP_LEFT, OUTPUT);
  pinMode(MOT_TOP_RIGHT, OUTPUT);
  pinMode(MOT_BOTTOM_LEFT, OUTPUT);
  pinMode(MOT_BOTTOM_RIGHT, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  analogWrite(MOT_TOP_LEFT, 0);
  analogWrite(MOT_TOP_RIGHT, 0);
  analogWrite(MOT_BOTTOM_LEFT, 0);
  analogWrite(MOT_BOTTOM_RIGHT, 0);

  digitalWrite(LED_BUILTIN,HIGH);

  set_gyro_registers();

  for (cal_int = 0; cal_int < 2000 ; cal_int ++){
    if(cal_int % 15 == 0)
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    gyro_signalen();
    gyro_axis_cal[1] += gyro_axis[1];
    gyro_axis_cal[2] += gyro_axis[2];
    gyro_axis_cal[3] += gyro_axis[3];
    delay(4);
  }

  gyro_axis_cal[1] /= 2000;
  gyro_axis_cal[2] /= 2000;
  gyro_axis_cal[3] /= 2000;

  WiFi.mode(WIFI_AP);
  WiFi.begin(APSSID, APPSW);
  while(WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  Udp.begin(localPort);

  while(receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400){
    int packetSize = Udp.parsePacket();
    if(packetSize) {
      int n = Udp.read(packetBuffer, UDP_PKT_MAX_SIZE);
      packetBuffer[n] = '\0';
      char ch1[5], ch2[5], ch3[5], ch4[5];
      ch1[4] = '\0'; ch2[4] = '\0'; ch3[4] = '\0'; ch4[4] = '\0';
      for(int i=0; i<4; i++) {
        ch4[i] = packetBuffer[i];
        ch3[i] = packetBuffer[i+4];
        ch1[i] = packetBuffer[i+8];
        ch2[i] = packetBuffer[i+12];
      }
      receiver_input_channel_1 = atoi(ch1);
      receiver_input_channel_2 = atoi(ch2);
      receiver_input_channel_3 = atoi(ch3);
      receiver_input_channel_4 = atoi(ch4);
    }
    start ++;
    delay(3);
    if(start == 125){
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      start = 0;
    }
  }
  start = 0;

  loop_timer = micros();

  digitalWrite(LED_BUILTIN,LOW);
}

void loop(){
  int packetSize = Udp.parsePacket();
  if(packetSize) {
      int n = Udp.read(packetBuffer, UDP_PKT_MAX_SIZE);
      packetBuffer[n] = '\0';
      char ch1[5], ch2[5], ch3[5], ch4[5];
      ch1[4] = '\0'; ch2[4] = '\0'; ch3[4] = '\0'; ch4[4] = '\0';
      for(int i=0; i<4; i++) {
        ch4[i] = packetBuffer[i];
        ch3[i] = packetBuffer[i+4];
        ch1[i] = packetBuffer[i+8];
        ch2[i] = packetBuffer[i+12];
      }
      receiver_input_channel_1 = atoi(ch1);
      receiver_input_channel_2 = atoi(ch2);
      receiver_input_channel_3 = atoi(ch3);
      receiver_input_channel_4 = atoi(ch4);
    }

  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);

  angle_pitch += gyro_pitch * 0.0000611;
  angle_roll += gyro_roll * 0.0000611;

  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);
  angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);

  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));

  if(abs(acc_y) < acc_total_vector){
    angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;
  }
  if(abs(acc_x) < acc_total_vector){
    angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;
  }

  angle_pitch_acc -= 0.0;
  angle_roll_acc -= 0.0;

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;

  pitch_level_adjust = angle_pitch * 15;
  roll_level_adjust = angle_roll * 15;

  if(!auto_level){
    pitch_level_adjust = 0;
    roll_level_adjust = 0;
  }

  if(receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050)start = 1;

  if(start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450){
    start = 2;

    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;
    gyro_angles_set = true;

    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }

  if(start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950)start = 0;

  pid_roll_setpoint = 0;

  if(receiver_input_channel_1 > 1508)pid_roll_setpoint = receiver_input_channel_1 - 1508;
  else if(receiver_input_channel_1 < 1492)pid_roll_setpoint = receiver_input_channel_1 - 1492;

  pid_roll_setpoint -= roll_level_adjust;
  pid_roll_setpoint /= 3.0;

  pid_pitch_setpoint = 0;

  if(receiver_input_channel_2 > 1508)pid_pitch_setpoint = receiver_input_channel_2 - 1508;
  else if(receiver_input_channel_2 < 1492)pid_pitch_setpoint = receiver_input_channel_2 - 1492;

  pid_pitch_setpoint -= pitch_level_adjust;
  pid_pitch_setpoint /= 3.0;

  pid_yaw_setpoint = 0;

  if(receiver_input_channel_3 > 1050){
    if(receiver_input_channel_4 > 1508)pid_yaw_setpoint = (receiver_input_channel_4 - 1508)/3.0;
    else if(receiver_input_channel_4 < 1492)pid_yaw_setpoint = (receiver_input_channel_4 - 1492)/3.0;
  }

  calculate_pid();

  throttle = receiver_input_channel_3;

  if (start == 2){
    if (throttle > 1800) throttle = 1800;
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;

    if (esc_1 < 1100) esc_1 = 1100;
    if (esc_2 < 1100) esc_2 = 1100;
    if (esc_3 < 1100) esc_3 = 1100;
    if (esc_4 < 1100) esc_4 = 1100;

    if(esc_1 > 2000)esc_1 = 2000;
    if(esc_2 > 2000)esc_2 = 2000;
    if(esc_3 > 2000)esc_3 = 2000;
    if(esc_4 > 2000)esc_4 = 2000;
  }

  else{
    esc_1 = 1000;
    esc_2 = 1000;
    esc_3 = 1000;
    esc_4 = 1000;
  }

  if(micros() - loop_timer > 4050)digitalWrite(LED_BUILTIN, HIGH);

  while(micros() - loop_timer < 4000);
  loop_timer = micros();

  esc_1 = map(esc_1, 1000, 2000, 0, 1000);
  esc_2 = map(esc_2, 1000, 2000, 0, 1000);
  esc_3 = map(esc_3, 1000, 2000, 0, 1000);
  esc_4 = map(esc_4, 1000, 2000, 0, 1000);

  analogWrite(MOT_TOP_RIGHT, esc_1);
  analogWrite(MOT_BOTTOM_RIGHT, esc_2);
  analogWrite(MOT_BOTTOM_LEFT, esc_3);
  analogWrite(MOT_TOP_LEFT, esc_4);

  gyro_signalen();
}

void gyro_signalen(){

  Wire.beginTransmission(gyro_address);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(gyro_address,14);

  int packetSize = Udp.parsePacket();
  if(packetSize) {
    int n = Udp.read(packetBuffer, UDP_PKT_MAX_SIZE);
    packetBuffer[n] = '\0';
    char ch1[5], ch2[5], ch3[5], ch4[5];
    ch1[4] = '\0'; ch2[4] = '\0'; ch3[4] = '\0'; ch4[4] = '\0';
    for(int i=0; i<4; i++) {
      ch4[i] = packetBuffer[i];
      ch3[i] = packetBuffer[i+4];
      ch1[i] = packetBuffer[i+8];
      ch2[i] = packetBuffer[i+12];
    }
    receiver_input_channel_1 = atoi(ch1);
    receiver_input_channel_2 = atoi(ch2);
    receiver_input_channel_3 = atoi(ch3);
    receiver_input_channel_4 = atoi(ch4);
  }

  while(Wire.available() < 14);
  acc_axis[1] = Wire.read()<<8|Wire.read();
  acc_axis[2] = Wire.read()<<8|Wire.read();
  acc_axis[3] = Wire.read()<<8|Wire.read();
  temperature = Wire.read()<<8|Wire.read();
  gyro_axis[1] = Wire.read()<<8|Wire.read();
  gyro_axis[2] = Wire.read()<<8|Wire.read();
  gyro_axis[3] = Wire.read()<<8|Wire.read();

  if(cal_int == 2000){
    gyro_axis[1] -= gyro_axis_cal[1];
    gyro_axis[2] -= gyro_axis_cal[2];
    gyro_axis[3] -= gyro_axis_cal[3];
  }
  gyro_roll = gyro_axis[0b00000010];
  gyro_pitch = gyro_axis[0b00000001];
  gyro_yaw = gyro_axis[0b00000011];
  gyro_yaw *= -1;
  acc_x = acc_axis[0b00000001];
  acc_y = acc_axis[0b00000010];
  acc_z = acc_axis[0b00000011];
  acc_z *= -1;
}

void calculate_pid(){

  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

void set_gyro_registers(){

  Wire.beginTransmission(gyro_address);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(gyro_address);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  Wire.beginTransmission(gyro_address);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(gyro_address);
  Wire.write(0x1B);
  Wire.endTransmission();
  Wire.requestFrom(gyro_address, 1);
  while(Wire.available() < 1);
  if(Wire.read() != 0x08){
    digitalWrite(12,HIGH);
    while(1)delay(10);
  }

  Wire.beginTransmission(gyro_address);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();
}