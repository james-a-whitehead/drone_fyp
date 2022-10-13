#include <Arduino.h>

// globals


// ota remember last flash
#include <EEPROM.h>
#define EEPROM_SIZE 1

// wifi
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

WiFiUDP udp;
const char * udpAddress = "0.0.0.0"; // broadcast on all ipv4 addresses
const int udpPort = 3333;


// commands
char udp_buff_rx [64];

#define NUM_COMMANDS 13

#define CMD_INCREASE_THRUST 0
#define CMD_DECREASE_THRUST 1
#define CMD_BATTERY_STATUS 2
#define CMD_REBOOT 3
#define CMD_DISPLAY_IMU 4
#define CMD_BEGIN_BALANCE 5
#define CMD_CHANGE_K 6
#define CMD_PRINT_K 7
#define CMD_EMERGENCY_OVERRIDE 8
#define CMD_TOGGLE_OTA_NEXT 9
#define CMD_COARSE_FINE_THRUST 10
#define CMD_DISTANCE 11
#define CMD_CONTROLLER 12

int command[NUM_COMMANDS];

void send_message_udp(const char msg[]) {
  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  udp.write((uint8_t*)msg, sizeof(char) * strlen(msg));
  udp.endPacket();
}

void serial_and_udp(const char msg[]) {
  Serial.println(msg);
  send_message_udp(msg);
}


// adc
#define BAT_VOL_PIN 19
float analogue_read_val = 0;
const int bat_vol_len = 4; // X.XX
char bat_vol_buf[bat_vol_len];

const int ADC_ARR_NUM_SAMPLES = 5;
float rolling_avg_adc_arr[ADC_ARR_NUM_SAMPLES] = {0};
int rolling_avg_adc_ptr = 0;
float rolling_avg_adc_val = 0.0;

// motors
#define MOT_1_PIN 5
#define MOT_2_PIN 6
#define MOT_3_PIN 7
#define MOT_4_PIN 15

#define MOT_1 0
#define MOT_2 1
#define MOT_3 2
#define MOT_4 3

int thrust_command = 0;


// mpu
#include <SparkFunMPU9250-DMP.h>

#define MPU9250_SLAVE_ADDR 0x68

#define SDA 40
#define SCL 41

#define G 9.81

MPU9250_DMP imu;

bool mpu_flag = false;
bool distance_flag = false;

float gyro_sens;
float accel_sens;


// height sensor
#include "Adafruit_VL53L1X.h"
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();

float z_raw;



// thanks to https://forum.sparkfun.com/viewtopic.php?p=193532&sid=91c3497f56e84bf09f5f2e16b582f016#p193532 for a body reference quat to euler transformer
static void toEulerianAngle(float w, float x, float y, float z, float& roll_, float& pitch_, float& yaw_)
{
  float ysqr = y * y;

  // roll (x-axis rotation)
  float t0 = +2.0 * (w * x + y * z);
  float t1 = +1.0 - 2.0 * (x * x + ysqr);
  roll_ = atan2(t0, t1) * 180/PI;

  // pitch (y-axis rotation)
  float t2 = +2.0 * (w * y - z * x);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  pitch_ = asin(t2) * 180/PI;

  // yaw (z-axis rotation)
  float t3 = +2.0 * (w * z + x * y);
  float t4 = +1.0 - 2.0 * (ysqr + z * z);  
  yaw_ = atan2(t3, t4) * 180/PI;
}


void disp_accel(float * ax, float * ay, float * az) {
  serial_and_udp(("Acceleration: " + String(*ax) + ", " + String(*ay) + ", " + String(*az)).c_str());
}

void disp_ang_rate(float * dr, float * dp, float * dy) {
  serial_and_udp(("Angle Rate: " + String(*dr) + ", " + String(*dp) + ", " + String(*dy)).c_str());
}

void disp_orien(float * r, float * p, float * y) {
  serial_and_udp(("Orientation: " + String(*r, 6) + "," + String(*p, 6) + "," + String(*y, 6)).c_str());
}

void disp_pos(float * x, float * y, float * z) {
  serial_and_udp(("Position: " + String(*x, 6) + "," + String(*y, 6) + "," + String(*z, 6)).c_str());
}

void disp_vel(float * dx, float * dy) {
  serial_and_udp(("Velocity: " + String(*dx, 6) + "," + String(*dy, 6)).c_str());
}


// control system

int every_10;
int every_10_ctr;

int every_100;
int every_100_ctr;


unsigned long t_last = 0;
unsigned long Ts;

int motor_max = 2048;
int motor_min = 0;

int mot_arr[4] = {0, 0, 0, 0};
int mot_arr_old[4] = {0, 0, 0, 0};
int mot_arr_unsaturated[4] = {0, 0, 0, 0};

void clear_mot_array() {
  for (int i=0;i<4;i++) mot_arr[i] = 0;
}

int saturate(int val, int max, int min) {
  return val < min ? min : (val > max ? max : val);
}

bool changed(int mot_arr[], int mot_arr_old[]) {
  bool acc = true;
  for (int i=0;i<4;i++){ // acc will be false if one of the motor commands has changed
    acc = acc & (mot_arr[i] == mot_arr_old[i]);
  }
  return !acc; // flip the result so 'changed' is true if there is a change
}


// pid controller classes
class PI_Controller {
  public:
    float *kp;
    float *ki;
    float error_sum;
    float integrator_windup_threshold;

    bool zero_cross_en;
    bool did_zero_cross = false;

    PI_Controller(float *kp_, float *ki_, float integrator_windup_threshold_, bool zero_cross_en_) {
      kp = kp_;
      ki = ki_;
      integrator_windup_threshold = integrator_windup_threshold_;
      zero_cross_en = zero_cross_en_;
    };

    float step(float input, float setpoint, float Ts) { // put Ts in seconds
      error = input - setpoint;
      error_diff = error - error_last;
      error_sum = error_sum + error;
      
      if (zero_cross_en) { // enable zero crossing reset
        if (error_last/abs(error_last) != error/abs(error)) {
          // this means the error sign changed, mustve crossed the setpoint.
          // reset the sum
          error_sum = 0;
          did_zero_cross = true;
        }
        else did_zero_cross = false;
      }

      error_last = error;
      // anti windup
      // if the controller is going to enter the saturated nonlinear region,
      // (only enable the integral term around the operating point)
      // stop the integral term increasing/decreasing by clamping the error sum

      if (abs(*ki * error_sum * Ts) > integrator_windup_threshold) {
        // subtract this step's contribution to the error,
        // basically freezes integral term contribution for the next steps
        error_sum = error_sum - error; 
      }

      control_output = (*kp * error) + (*ki * error_sum * Ts);// + (*kd * error_diff / Ts) ;

      return control_output;
    };

    void reset() {
      error = 0;
      error_last = 0;
      error_diff = 0;
      control_output = 0;
      error_sum = 0;
    };
  
  private:
    float error;
    float error_last;
    float error_diff;
    float control_output;
};

class AttitudePI {
  public:
    AttitudePI(float *kp_rate_, float *ki_rate_, float rate_limiter_, float *kp_ang_, float *ki_ang_, float angle_limiter_)
      : ratePI(kp_rate_, ki_rate_, rate_limiter_, false)
      , anglePI(kp_ang_, ki_ang_, angle_limiter_, true)
      {
    };
    
    float step(float angle_setpoint, float angle, float angle_rate, float Ts) {
      // feed angle pid into rate pid
      rate_control = anglePI.step(angle, angle_setpoint, Ts);
      rate_output = ratePI.step(angle_rate, -rate_control, Ts); // minus???

      if (every_10) {
        //serial_and_udp((String("ang_er: ") + String(angle - angle_setpoint) + String(" ang_err_sum: ") + String(anglePI.error_sum) + String(" w_set: ") + String(rate_control) + String(" w_err: ") + String(angle_rate + rate_control) + String(" w_err_sum: ") + String(ratePI.error_sum) + String(" mot_set: ") + String(rate_output) + String(" zero_cross: ") + String(anglePI.did_zero_cross)).c_str());
      }

      return rate_output;
    };

    void reset() {
      rate_control = 0;
      rate_output = 0;
      ratePI.reset();
      anglePI.reset();
    };

  private:
    float rate_control;
    float rate_output;
    PI_Controller ratePI;
    PI_Controller anglePI;
};


float r;
float p;
float yaw;

float dr;
float dp;
float dyaw;

float x = 0;
float y = 0;
float z = 0;

float dx = 0;
float dy = 0;
float dz = 0;

void position_reset() {
  x = 0;
  y = 0;
  z = 0;
  dx = 0;
  dy = 0;
  dz = 0;
}

float ax_raw;
float ay_raw;
float az_raw;

float ax;
float ay;
float az;

float xt = 0;
float yt = 0;

float rt = 0;
float pt = 0;
float yawt = 0.0;

float roll_command;
float pitch_command;
float yaw_command;


float kp_roll_angle = 0.7;
float kp_pitch_angle = kp_roll_angle;
float kp_yaw_angle = 0;

float ki_roll_angle = 0;
float ki_pitch_angle = ki_roll_angle;
float ki_yaw_angle = 0;

float kp_roll_rate = 0.5;
float kp_pitch_rate = kp_roll_rate;
float kp_yaw_rate = 2;

float ki_roll_rate = 0;
float ki_pitch_rate = ki_roll_rate;
float ki_yaw_rate = 0;

// guesswork: clamp control output at 5% of total motor value, i.e. 2048/20 ~ 100
float rate_i_limiter = 200.0;

// around the operating point, the angle rate error will (hopefully) be relatively small thanks
// to the P component doing the hard work. we only need the integral component to correct small 
// steady state errors 
float angle_i_limiter = 200.0;

AttitudePI rollPI(&kp_roll_rate, &ki_roll_rate, rate_i_limiter, &kp_roll_angle, &ki_roll_angle, angle_i_limiter);
AttitudePI pitchPI(&kp_roll_rate, &ki_roll_rate, rate_i_limiter, &kp_roll_angle, &ki_roll_angle, angle_i_limiter);
AttitudePI yawPI(&kp_yaw_rate, &ki_yaw_rate, rate_i_limiter, &kp_yaw_angle, &ki_yaw_angle, angle_i_limiter);


// height control

// will have to get the drone up to the approximate hover thrust and then allow it to control around 
// this operating point. otherwise there will be nasty overshoot at liftoff.

// hovering thrust can be determined via battery voltage probably. need to find this value
// then start the drone motors at this value and let it creep around to change height


float kp_height = 0;
float ki_height = 0;

float height_i_limiter = 200.0;

PI_Controller heightPI(&kp_height, &ki_height, height_i_limiter, true);

float height_command;

float ht;
float h;


// xy control

// we need to find x and y

// by optical flow-

// or by accelerometer

// need to double integrate the acceleration values to get position
// how to get the true x acceleration rather than tilted drone body acceleration??
// just glue down the imu lmao dont compute its physical angle.


// then can use the attitude values to compensate for tilt?
// do it without tilt compensation first



// these will control x and y via pitch and roll

float kp_xy = 0;
float ki_xy = 0;

float xy_i_limiter = 200.0;


// x controller will control roll angle, y controller will control pitch angle
// these x and y will be in drone body reference values
PI_Controller xPI(&kp_xy, &ki_xy, xy_i_limiter, true);
PI_Controller yPI(&kp_xy, &ki_xy, xy_i_limiter, true);



// compensates a value for tilting away from level.
// for example, if you multiply the detected height by this number
// while the drone is tilted, it will give the true drone height
float tilt_comp(float r, float p) {
  return cos(r * PI / 180) * cos(p * PI / 180);
}


void motor_mix(float rc, float pc, float yc, float tc) {
// rpy commands now need to be translated into a sensible motor output

  // we want to choose kp, ki so that the drone doesnt go mental.
  // then use these rpy commands in the MMA
  // then finally, saturate the MMA so that motors dont go outside of the 0-2048 range.

  // mot 1 top left, CCW
  // mot 2 top right, CW
  // mot 3 bottom right, CCW
  // mot 4 bottom left, CW

  // a roll left would require motors 1 and 3 to decrease, 2 and 4 to increase
  // a pitch forward would require motors 1 and 2 to decrease, 3 and 4 to increase
  // and the reverse is true for roll right and pitch backward

  // drone roll is mpu pitch and vice versa (oop)
  // drone left side down (roll left) is a negative value in mpu pitch
  // drone nose down (pitch forward) is a negative value in mpu roll

  // clockwise drone rotation causes a negative yaw angle
  // decreasing M1 and M3, increasing M2 and M4 will cause a clockwise rotation
  // decreasing M2 and M4, increasing M1 and M3 will cause a ccw rotation

  // dampen/boost motor values depending on battery vol

  // determine power draw at 'hover'
  // this value will be used to tweak the pwm values based on the battery voltage
  // eg. if the drone hovers at 80% thrust with a 3.8 battery voltage, then the 'hover voltage' is 0.8 * 3.8 = 3.04v.
  // then if the battery voltage is 3.3v, the 'hover voltage' will occur at 3.04/3.3 = 92% pwm
  // we then track the battery voltage with the adc and change the motor commands to normalise them over the voltage range,
  // effectively keeping the PID tuning constant whilst the battery discharges.

  // with a fully charged battery, 4A current draw drags the voltage to approximately around 3.3V
  // have all of the motor thrust commands get normalised 1x at 3.3V, then at say 3.5V, nultiply the commands by 3.3/3.5. 
  // when battery goes down to 3.2-3.1, multiply by 3.3/3.1. this should make the motor actions appear normalised to the controller

  
  mot_arr_unsaturated[MOT_1] = (-rc + -pc + -yc + tc) * 3.3/rolling_avg_adc_val;
  mot_arr_unsaturated[MOT_2] = (+rc + -pc + +yc + tc) * 3.3/rolling_avg_adc_val;
  mot_arr_unsaturated[MOT_3] = (+rc + +pc + -yc + tc) * 3.3/rolling_avg_adc_val;
  mot_arr_unsaturated[MOT_4] = (-rc + +pc + +yc + tc) * 3.3/rolling_avg_adc_val;

  mot_arr[MOT_1] = saturate(mot_arr_unsaturated[MOT_1], motor_max, motor_min);
  mot_arr[MOT_2] = saturate(mot_arr_unsaturated[MOT_2], motor_max, motor_min);
  mot_arr[MOT_3] = saturate(mot_arr_unsaturated[MOT_3], motor_max, motor_min);
  mot_arr[MOT_4] = saturate(mot_arr_unsaturated[MOT_4], motor_max, motor_min); 
}


void balance_control_() {
  
  roll_command = rollPI.step(rt, r, dr, Ts/1000.0); // Ts in seconds
  pitch_command = pitchPI.step(pt, p, dp, Ts/1000.0); // Ts in seconds
  yaw_command = yawPI.step(yawt, yaw, -dyaw, Ts/1000.0); // Ts in seconds, investigate this negation......


  //height_command = heightPI.step(ht, h, Ts/1000);

  motor_mix(roll_command, pitch_command, yaw_command, thrust_command);

}


bool balance_flag = false;

bool emergency_override = true;

bool grain = false;

void setup() {

  // begin serial
  Serial.begin(115200);

  EEPROM.begin(EEPROM_SIZE);

  // setup wifi connection
  const char* ssid = "jame wiffy";
  const char* password = "24382438";
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  Serial.println("WiFi Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // mpu

  Wire.setPins(SDA, SCL); // SDA , SCLb

  imu.begin();

  imu.dmpBegin( DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
                DMP_FEATURE_GYRO_CAL | // zeros gyros after 8s standstill (auto calibrate)
                DMP_FEATURE_SEND_RAW_ACCEL |
                DMP_FEATURE_SEND_CAL_GYRO,
                100); // Set DMP FIFO rate to 90 Hz (just under loop polling rate)
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive

  imu.setLPF(5);
  


  gyro_sens = (float)imu.getGyroSens();
  accel_sens = (float)imu.getAccelSens();

  // setup OTA

  ArduinoOTA.onEnd([]() {EEPROM.write(0, 1); EEPROM.commit();});

  if (!EEPROM.read(0)) {
    ArduinoOTA.begin();
    Serial.println("Pausing to wait for OTA prog.");
    unsigned long start = millis();

    while (millis() - start < 10000) {
      ArduinoOTA.handle();
      delay(50);
    }
  }
  else {
    EEPROM.write(0, 0);
    EEPROM.commit();
  }

  
  // setup command listener
  udp.begin(WiFi.localIP(), udpPort);


  // setup sensors

  vl53.begin();

  vl53.startRanging();

  vl53.setTimingBudget(33);



  // adc
  adcAttachPin(BAT_VOL_PIN);

  // setup motors
  ledcAttachPin(MOT_1_PIN, MOT_1);
  ledcAttachPin(MOT_2_PIN, MOT_2);
  ledcAttachPin(MOT_3_PIN, MOT_3);
  ledcAttachPin(MOT_4_PIN, MOT_4);

  ledcSetup(MOT_1, 15000, 11); // 15 kHz PWM, 11-bit resolution
  ledcSetup(MOT_2, 15000, 11); // 15 kHz PWM, 11-bit resolution
  ledcSetup(MOT_3, 15000, 11); // 15 kHz PWM, 11-bit resolution
  ledcSetup(MOT_4, 15000, 11); // 15 kHz PWM, 11-bit resolution

  // POST
  ledcWrite(MOT_1, 256);
  delay(500);
  ledcWrite(MOT_1, 0);
  
  ledcWrite(MOT_2, 256);
  delay(500);
  ledcWrite(MOT_2, 0);

  ledcWrite(MOT_3, 256);
  delay(500);
  ledcWrite(MOT_3, 0);

  ledcWrite(MOT_4, 256);
  delay(500);
  ledcWrite(MOT_4, 0);

  t_last = millis();
  Ts = 10;
  
}



void balance_begin_stuff() {

}

int delay_val = 0;

void loop() {

  every_10 = false;
  every_100 = false;

  if (every_10_ctr < 10) {
    every_10_ctr++;
  }
  else {
    every_10_ctr = 0;
    every_10 = true;
  }

  if (every_10) {
    if (every_100_ctr < 10) {
      every_100_ctr++;
    }
    else {
      every_100_ctr = 0;
      every_100 = true;
    }
  }

  // update mpu values
  if ( imu.fifoAvailable() ) { 
    imu.dmpUpdateFifo();
    toEulerianAngle(imu.calcQuat(imu.qw), imu.calcQuat(imu.qx), imu.calcQuat(imu.qy), imu.calcQuat(imu.qz), p, r, yaw); // FLIPPED HERE
    dr = imu.gy/gyro_sens;
    dp = imu.gx/gyro_sens;
    dyaw = imu.gz/gyro_sens;
    ax_raw = imu.ax/accel_sens;
    ay_raw = imu.ay/accel_sens;
    az_raw = imu.az/accel_sens;

    // nasty integration of accelerometer for position

    // subtract gravity component from all axis using orientation?? later

    ax = ax_raw;
    ay = ay_raw;
    az = az_raw;


    // convert ax from Gs to m/s/s, convert Ts into s, get small increment dv
    dx = dx + ax/G * Ts/1000;
    dy = dy + ay/G * Ts/1000;
    //dz = dz + az/G * Ts/1000;

    // convert Ts into s, add small velocity increment to position??
    x = x + dx;
    y = y + dy;
    //z = z + dz;
  };

  // display mpu values
  if (mpu_flag && every_10) {
    
    disp_orien(&r, &p, &yaw);
    //disp_accel(&ax_raw, &ay_raw, &az_raw);
    //disp_ang_rate(&dr, &dp, &dyaw);
    //disp_vel(&dx, &dy);
    //disp_pos(&x, &y, &z);
  }

  if (vl53.dataReady()) {
    z_raw = vl53.distance();
    z = z_raw * tilt_comp(r, p);
  }

  // clear command array
  for (int i = 0; i<NUM_COMMANDS; i++) command[i] = 0;

  // read instructions
  while (int packetsize = udp.parsePacket()) {
    for (int i=0;i<2;i++) udp_buff_rx[i] = 0;
    int num = udp.read(udp_buff_rx, (size_t)packetsize);
    int cmd = atoi(&udp_buff_rx[0]);
    if (!(cmd == 12)) {
      serial_and_udp("Recieved command: ");
      serial_and_udp((String(atoi(&udp_buff_rx[0]))).c_str());
    }
    command[atoi(&udp_buff_rx[0])] = 1;
  }

  if (every_10) {
    float divider_offset_read = (((float)analogRead(BAT_VOL_PIN) * 1.5) / 4096) * 3.3;
    analogue_read_val = divider_offset_read * 0.9743 + 0.1872; // calibration values
    rolling_avg_adc_arr[rolling_avg_adc_ptr] = analogue_read_val;
    rolling_avg_adc_ptr = (rolling_avg_adc_ptr + 1) % ADC_ARR_NUM_SAMPLES;
    float sum = 0;
    for (int i=0;i<ADC_ARR_NUM_SAMPLES;i++) sum = sum + rolling_avg_adc_arr[i];
    rolling_avg_adc_val = sum/ADC_ARR_NUM_SAMPLES;
  }

  // battery status
  if (command[CMD_BATTERY_STATUS]) {
    sprintf(bat_vol_buf, "%.2f", rolling_avg_adc_val);
    serial_and_udp((String("First buffer read, rolling avg: ") + String(bat_vol_buf) + String(", ") + String(rolling_avg_adc_val)).c_str());
  }

  if (command[CMD_CHANGE_K]) { // format: CMD_NUM K_SELECT PID_SELECT VALUE 
    // tune mode
    if (udp_buff_rx[2] == '0') { 
      //  kp
      if (udp_buff_rx[4] == '0') {
        // rate
        kp_roll_rate = atof(&udp_buff_rx[6]);
        serial_and_udp((String("Converted kp_roll_rate to: ") + String(kp_roll_rate, 4)).c_str());
      }
      else if (udp_buff_rx[4] == '1'){
        // angle
        kp_roll_angle = atof(&udp_buff_rx[6]);
        serial_and_udp((String("Converted kp_roll_angle to: ") + String(kp_roll_angle, 4U)).c_str());
      }
      else if (udp_buff_rx[4] == '2') {
        // yaw
        kp_yaw_rate = atof(&udp_buff_rx[6]);
        serial_and_udp((String("Converted kp_yaw_rate to: ") + String(kp_yaw_rate, 4U)).c_str());
      }
    }
    else if (udp_buff_rx[2] == '1') {
      // ki
      if (udp_buff_rx[4] == '0') {
        // rate
        ki_roll_rate = atof(&udp_buff_rx[6]);
        serial_and_udp((String("Converted ki_roll_rate to: ") + String(ki_roll_rate, 4U)).c_str());
      }
      else if (udp_buff_rx[4] == '1'){
        // angle
        ki_roll_angle = atof(&udp_buff_rx[6]);
        serial_and_udp((String("Converted ki_roll_angle to: ") + String(ki_roll_angle, 4U)).c_str());
      }
      else if (udp_buff_rx[4] == '2') {
        // yaw
        ki_yaw_rate = atof(&udp_buff_rx[6]);
        serial_and_udp((String("Converted ki_yaw_rate to: ") + String(ki_yaw_rate, 4U)).c_str());
      }
    }
    else if (udp_buff_rx[2] == '2') {
      // setpoint
      if (udp_buff_rx[4] == '0') {
        // roll
        rt = atof(&udp_buff_rx[6]);
        serial_and_udp((String("Converted rt to: ") + String(rt, 4U)).c_str());
      }
      else if (udp_buff_rx[4] == '1') {
        // roll
        pt = atof(&udp_buff_rx[6]);
        serial_and_udp((String("Converted pt to: ") + String(pt, 4U)).c_str());
      }
    }
    // clear udp buff
    for (int i=0;i<64;i++) udp_buff_rx[i] = 0;
  }

  if (command[CMD_REBOOT]) ESP.restart();

  if (command[CMD_DISPLAY_IMU]) {
    mpu_flag = !mpu_flag;
    position_reset();
  }

  if (command[CMD_EMERGENCY_OVERRIDE]) {
    emergency_override = !emergency_override;
    command[CMD_BEGIN_BALANCE] = 1;
  }

  if (command[CMD_TOGGLE_OTA_NEXT]) {
    int val = EEPROM.read(0);
    serial_and_udp((String("EEPROM was: ") + String(val) + String(", toggling")).c_str());
    EEPROM.write(0, !val);
    EEPROM.commit();
  }

  if (command[CMD_DISTANCE]) distance_flag = !distance_flag;

  if (command[CMD_PRINT_K]) {
    serial_and_udp((String("kp_rp_rate: ") + String(kp_roll_rate, 4U)).c_str());
    serial_and_udp((String("kp_rp_angle: ") + String(kp_roll_angle, 4U)).c_str());
    serial_and_udp((String("ki_rp_rate: ") + String(ki_roll_rate, 4U)).c_str());
    serial_and_udp((String("ki_rp_angle: ") + String(ki_roll_angle, 4U)).c_str());
    serial_and_udp((String("kp_yaw_rate: ") + String(kp_yaw_rate, 4U)).c_str());
    serial_and_udp((String("ki_yaw_rate: ") + String(ki_yaw_rate, 4U)).c_str());
    serial_and_udp((String("rt: ") + String(rt, 4U)).c_str());  
    serial_and_udp((String("pt: ") + String(pt, 4U)).c_str());  
  }

  if (command[CMD_COARSE_FINE_THRUST]) {
    grain = !grain;
    serial_and_udp((("grain changed to: ") + String(grain)).c_str());
  }
    
  // action motors
  if (command[CMD_INCREASE_THRUST]){
    if (thrust_command < 2048){
      thrust_command = thrust_command + (grain ? 128 : 16);
      serial_and_udp((("thrust increased to :") + String(thrust_command)).c_str());
    }
    else {
      serial_and_udp("cant increase");
    }
  }
  else if(command[CMD_DECREASE_THRUST]){
    if (thrust_command > 0){
      thrust_command = thrust_command - (grain ? 128 : 16);
      serial_and_udp((("thrust decreased to :") + String(thrust_command)).c_str());
    }
    else {
      serial_and_udp("cant decrease");
    }
  }


  if (command[CMD_CONTROLLER]) {

    
    rt = -atoi(&udp_buff_rx[3])/4;
    pt = -atoi(&udp_buff_rx[9])/4;
    yawt = atoi(&udp_buff_rx[15]);
    thrust_command = 1408 + 2*atoi(&udp_buff_rx[27]);
    serial_and_udp((String("roll set: ") + String(rt) + String(", Pitch set: ") + String(pt) + String(", Yaw set: ") + String(yawt)).c_str());
  }

  if (command[CMD_BEGIN_BALANCE]) {
    command[CMD_BEGIN_BALANCE] = 0;
    balance_flag = !balance_flag;
    imu.resetFifo(); // clear the fifo of old data before reading it in realtime (probably a better way to do this)
    // reset pids
    rollPI.reset();
    pitchPI.reset();
    yawPI.reset();

    heightPI.reset();

    position_reset();
    mpu_flag = !mpu_flag;
  }


  if (balance_flag) balance_control_();


  if (emergency_override){
    ledcWrite(MOT_1, 0);
    ledcWrite(MOT_2, 0);
    ledcWrite(MOT_3, 0);
    ledcWrite(MOT_4, 0);
  }
  else if (changed(mot_arr, mot_arr_old)) { // if motor commands changed, update
    ledcWrite(MOT_1, mot_arr[MOT_1]);
    ledcWrite(MOT_2, mot_arr[MOT_2]);
    ledcWrite(MOT_3, mot_arr[MOT_3]);
    ledcWrite(MOT_4, mot_arr[MOT_4]);
  }

  // repeat
  // we want to set the freq. of this loop if possible. delay for the target delay minus the loop duration

  // hopefully we will have a Ts of about 100Hz (10ms)
  Ts = millis() - t_last;
  t_last = t_last + Ts;

  // time spent running this cycle (no sleep) = Ts - delay_val
  // time spent running this cycle (sleep) = Ts
  // sleep fraction = delay_val/Ts

  float sleep_pc = ((float)delay_val)/((float)Ts); // in this 10ms loop, what fraction of time was spent sleeping?

  int delay_target = 10;
  delay_val = (delay_target < Ts) ? 0 : (delay_target - Ts);

  

  //serial_and_udp((String("Loop Time: ") + String(sleep_pc, 4)).c_str());

  //Serial.print("Loop period: ");
  //Serial.println(Ts);
  //Serial.print("Delay value: ");
  //Serial.println(delay_val);
  delay(delay_val);
}