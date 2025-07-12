#include <RoboClaw.h>
#include<Servo.h>
#include <Wire.h>
#include <MPU6050_light.h>

//////mode and speed//////
int mode1 = 0;
int mode2 = 0;
int mode3 = 0;
int mode4 = 0;
float v1 = 0, v2 = 0, v3 = 0;
float maxspeed = 32000;
/////////////////////////////

////////feeder//////////
int feed = 0;

float speed1 = 150, speed2 = 200;
float mult = 10;

int reset = 0;

int amp1 = 5;   ///amp1 is black mounted
int le_amp1 = 7;
int re_amp1 = 6;

int amp2 = 11;  //amp2 is ylw mounted
int le_amp2 = 13;
int re_amp2 = 12;

int feed_stat = 0;
////////////PID///////////////
float target = 0;
float output = 0;
float integral = 0;
float proportional = 0;
float derivative = 0;
float lastError = 0;
float piderror = 0;
float kp = 6.7;  //6.7 0.008 9.5
float ki = 0.008;
float kd = 10.5;
float printp = 0, printi = 0, printd = 0;
/////////ps dis//////////
int count_RX = 0;

///////////align
int receivedAngle = 0;
int align_stat = 0;


////angle sign
int sign=0;
////////////define all/////////
#define BAUD 9600
#define BAUDRATE  ((F_CPU/(BAUD*16UL)-1))
#define pwm_range 115
#define PS_L1 0
#define PS_R1 1
#define PS_L2 2
#define PS_R2 3
#define PS_L3 4
#define PS_R3 5
#define PS_TRIANGLE 6
#define PS_SQUARE 7
#define PS_CROSS 8
#define PS_CIRCLE 9
#define PS_UP 10
#define PS_LEFT 11
#define PS_DOWN 12
#define PS_RIGHT 13
#define PS_START 14
#define PS_SELECT 15

/////////////////////

MPU6050 mpu(Wire);
RoboClaw roboclaw(&Serial1, 10000);

int prevMillis = 0;
int xj1Offset = 0, yj1Offset = 0, xj2Offset = 0, yj2Offset = 0, calibrateFlag = 0, prevComp = 0;

int xj1 = 0, yj1 = 0, xj2 = 0, yj2 = 0, pot1 = 0, pot2 = 0, pot3 = 0, pot4 = 0; //analog values(serially received from remote)
int butt[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  //digital values(serially received from remote)

uint8_t RX[16] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
int RX_range = 200, RX_raw = 255, RX_ad = 255, RX_count = 0;
uint8_t TX[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
int flag[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t TX_raw = 200, TX_ad = 201, TX_flag = 0;

bool ps_setup() {
  UBRR3H = BAUDRATE >> 8;
  UBRR3L = BAUDRATE;
  UCSR3B = 0b10011000;
  UCSR3C = 0b00000110;
}

ISR(USART3_RX_vect) {
  RX_count = 1;
  RX_raw = UDR3;
  if ((RX_raw > 200) && (RX_raw < 255)) {
    RX_ad = RX_raw;
    if ((RX_raw > 230) && (RX_raw < 247))
    {
      uint8_t r_temp0 = (RX_raw - 231);
      butt[r_temp0] = 1;
    }
  }
  else if ((RX_raw >= 0) && (RX_raw < 201)) {
    uint8_t r_temp1 = (RX_ad - 201);
    if (r_temp1 < 16)
    {
      RX[r_temp1] = RX_raw;
    }
  }
}
//////////////ps read///////////
bool ps_read() {
  RX_count = 0;

  xj1 = map(RX[0], 0, RX_range, (-pwm_range), pwm_range);
  yj1 = map(RX[1], 0, RX_range, pwm_range, (-pwm_range));
  xj2 = map(RX[2], 0, RX_range, (-pwm_range), pwm_range);
  yj2 = map(RX[3], 0, RX_range, pwm_range, (-pwm_range));
  pot1 = map(RX[4], 0, RX_range, 0, 255);  // L2
  pot2 = map(RX[5], 0, RX_range, 0, 255);  // R2

  if (xj1 < 30 && xj1 > -30) xj1 = 0;
  if (xj2 < 30 && xj2 > -30) xj2 = 0;
  if (yj1 < 30 && yj1 > -30) yj1 = 0;
  if (yj2 < 30 && yj2 > -30) yj2 = 0;

  // --- SIGN logic using latched trigger input ---
  static bool l2Latched = false;
  static bool r2Latched = false;

  if (pot1 > 200 && !l2Latched && pot2 <= 200) {
    sign = 1;
    l2Latched = true;
    r2Latched = false;
  }
  else if (pot2 > 200 && !r2Latched && pot1 <= 200) {
    sign = -1;
    r2Latched = true;
    l2Latched = false;
  }

  // ----------------------------- BUTTONS -----------------------------
  if (butt[PS_START] == 1) {
    speed1 = 0;
    speed2 = 0;
    butt[PS_START] = 0;
  }

  if (butt[PS_SELECT] == 1) {
    asm volatile ("jmp 0");
    butt[PS_SELECT] = 0;
  }

  if (butt[PS_UP] == 1) {
    if (mode1 == 1) {
      target += 5;
      kp = 8.6;
      ki = 0.01;
      kd = 11.5;
    }
    if (mode2 == 1) {
      target += 90;
    }
    kp = 8.6;
    ki = 0.01;
    kd = 11.5;
    butt[PS_UP] = 0;
  }

  if (butt[PS_DOWN] == 1) {
    if (mode1 == 1) {
      target -= 5;
      kp = 8.6;
      ki = 0.01;
      kd = 11.5;
    }
    if (mode2 == 1) {
      target -= 90;
    }
    kp = 8.6;
    ki = 0.01;
    kd = 11.5;
    butt[PS_DOWN] = 0;
  }

  if (butt[PS_LEFT] == 1) {
    target += 30;
    kp = 8.6;
    ki = 0.01;
    kd = 11.5;
    butt[PS_LEFT] = 0;
  }

  if (butt[PS_RIGHT] == 1) {
    target -= 30;
    kp = 8.6;
    ki = 0.01;
    kd = 11.5;
    butt[PS_RIGHT] = 0;
  }

  if (butt[PS_SQUARE] == 1) {
    speed2 -= mult;
    if (speed2 < 10) speed2 = 10;
    butt[PS_SQUARE] = 0;
  }

  while (butt[PS_CIRCLE] == 1) {
    speed1 += mult;
    if (speed1 > 250) speed1 = 250;
    butt[PS_CIRCLE] = 0;
  }

  if (butt[PS_TRIANGLE] == 1) {
    speed2 += mult;
    if (speed2 > 250) speed2 = 250;
    butt[PS_TRIANGLE] = 0;
  }

  if (butt[PS_CROSS] == 1) {
    speed1 -= mult;
    if (speed1 < 10) speed1 = 10;
    butt[PS_CROSS] = 0;
  }

  if (butt[PS_L1] == 1) {
    feed = !feed;
    butt[PS_L1] = 0;
  }

  if (butt[PS_R1] == 1) {
    target -= receivedAngle;
    kp = 8.6;
    ki = 0.01;
    kd = 11.5;
    butt[PS_R1] = 0;
  }

  if (butt[PS_L2] == 1) {
    butt[PS_L2] = 0;
  }

  if (butt[PS_R2] == 1) {
    butt[PS_R2] = 0;
  }

  if (butt[PS_L3] == 1) {
    butt[PS_L3] = 0;
  }

  if (butt[PS_R3] == 1) {
    butt[PS_R3] = 0;
  }

  return true;
}

//////////////////////

//////////setup/////////////
void setup() {
  ps_setup();
  roboclaw.begin(115200);
  Serial.begin(9600);
  Serial2.begin(9600);
  Wire.begin();

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) { } // stop everything if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");

  pinMode(amp1, OUTPUT);
  pinMode(re_amp1, OUTPUT);
  pinMode(le_amp1, OUTPUT);

  pinMode(amp2, OUTPUT);
  pinMode(re_amp2, OUTPUT);
  pinMode(le_amp2, OUTPUT);


}
//////////////////////

////////loop//////
void loop() {
  //Serial.println("hello");
  dis_ps();
  if (count_RX <= 105)
  {
    ps_read();
    modecheck();
    if (mode1 == 1 || mode2 == 1) {
      fod();
      inv_kin(xj1, yj1, PID());
    }
    feeder();
    Serial.print("sign");Serial.println(sign);
//    if (Serial2.available()) {
//      int raw = Serial2.read();  // 1-byte value from Jetson
//      receivedAngle = raw - 128;
//          Serial.print("Received angle: ");
//          Serial.println(receivedAngle);
//    }
  }
}
///////////////

////angle() function///////
float angle() {
  mpu.update();
  float angle_z = mpu.getAngleZ();
  //  timer = millis();
  //Serial.println(angle_z);
  return angle_z;
}
////////////////////////

//PS_disconnect code block//////
void dis_ps() {
  if (RX_count == 0) {
    count_RX++;
  }
  if (RX_count == 1) {
    count_RX = 0;
  }
  if (count_RX >= 35) {
    xj1 = 0;
    yj1 = 0;
    roboclaw.DutyM1(128, 0);
    roboclaw.DutyM2(128, 0);// if on other roboclaw M1 129 -v2
    roboclaw.DutyM2(129, 0);
    digitalWrite(re_amp2, LOW);
    digitalWrite(le_amp2, LOW);
    digitalWrite(re_amp1, LOW);
    digitalWrite(le_amp1, LOW);
    analogWrite(amp2, 0);
    analogWrite(amp1, 0);
    Serial.println("PS disconnected");
  }
}
///////////////////////

///////////mode///////////////
void modecheck() {
  if (xj2 > 100) {
    mode1 = 1;
    mode2 = 0;
    mode3 = 0;
    mode4 = 0;
  }
  if (xj2 < -100) {
    mode1 = 0;
    mode2 = 1;
    mode3 = 0;
    mode4 = 0;
  }
  if (yj2 > 100) {
    mode1 = 0;
    mode2 = 0;
    mode3 = 1;
    mode4 = 0;
  }
  if (yj2 < -100) {
    mode1 = 0;
    mode2 = 0;
    mode3 = 0;
    mode4 = 1;
  }
  //Serial.print("xj2 ");Serial.print(xj2);
  //Serial.print(" || yj2");Serial.print(yj2);
  //  Serial.print(" ||mode1 ");Serial.print(mode1);
  //    Serial.print(" ||mode2 ");Serial.print(mode2);
  //  Serial.print(" || mode3 ");Serial.print(mode3);
  //  Serial.print(" || mode4 ");Serial.println(mode4);

}
//////////////////////

////fod code block/////
void fod() {
  float pi = 3.1415926;
  float theta = (angle() * (pi / 180));

  //  Serial.print("xj1: "); Serial.print(xj1);
  //  Serial.print(" || yj1: "); Serial.print(yj1);

  float temp1 = ((yj1 * cos(theta)) - (xj1 * sin(theta)));
  float temp2 = ((yj1 * sin(theta)) + (xj1 * cos(theta)));
  xj1 = temp2;
  yj1 = temp1;

  //  Serial.print("xx: "); Serial.print(xj1);
  //  Serial.print(" || yy: "); Serial.print(yj1);

}
////////////////////

//Inverse Kinematis code block///////
void inv_kin(float X, float Y, float Z) {

  float z = map(Z, -200, 200, -40000, 40000); //maps value of output with range of speed of motors // 255
  float x = map(X, -255, 255, -95000, 95000); //maps xj1 values to max speed of motors //95k
  float y = map(Y, -255, 255, -95000, 95000); //maps yj1 values to max speed of motors

  //  Serial.print(" || X "); Serial.print(x);
  //  Serial.print(" || Y "); Serial.print(y);
  //  Serial.print(" || Z "); Serial.print(z);

  x = -x;
  y = -y;
  z = -z;
  //inverse kinematics equation for each motor
  v1 = (0.3333 * z) - (0.66667 * x);
  v2 = (0.3333 * x) - (0.57735 * y) + (0.3333 * z);
  v3 = (0.3333 * x) + (0.57735 * y) + (0.3333 * z);

  //constrain speed to max values
  v1 = constrain(v1, -maxspeed, maxspeed);
  v2 = constrain(v2, -maxspeed, maxspeed);
  v3 = constrain(v3, -maxspeed, maxspeed);

  roboclaw.DutyM1(128, -v3);
  roboclaw.DutyM2(128, -v2);// if on other roboclaw M1 129 -v2
  roboclaw.DutyM2(129,  v1);

//  Serial.print(" || v1 = "); Serial.print(v1);
//  Serial.print(" || v2 = "); Serial.print(v2);
//  Serial.print(" || v3 = "); Serial.print(v3);
//  Serial.print("end = "); Serial.print(output);
//  Serial.print(" || target = "); Serial.print(target);
//  Serial.print(" || current = "); Serial.print(angle());
//  Serial.print(" || error = "); Serial.print(piderror);
//  Serial.print("proportional"); Serial.print(printp);
//  Serial.print(" || integral"); Serial.print(printi);
//  Serial.print(" || derivative"); Serial.println(printd);
}
///////////////////////////////

//PID application code block/////////
float PID() {
  piderror = target - angle();

  integral += piderror;
  proportional = piderror;
  derivative = piderror - lastError;

  printp = proportional * kp;
  printi = integral * ki;
  printd = derivative * kd;

  output = printp + printi + printd; //PID equation

  output = constrain(output, -200, 200);

  lastError = piderror;
  return output;
}
//////////////////////

///////feeder//////////
void feeder() {
  if (feed == 1) {

    if (mode3 == 1) {
      analogWrite(amp1, speed1);  // speed for DC Motor 1
      digitalWrite(le_amp1, LOW);
      digitalWrite(re_amp1, HIGH);

      analogWrite(amp2, speed2);  // speed for DC Motor 2.
      digitalWrite(le_amp2, LOW);
      digitalWrite(re_amp2, HIGH);
    }
    else
    {
      analogWrite(amp1, speed1);  // speed for DC Motor 1
      digitalWrite(le_amp1, HIGH);
      digitalWrite(re_amp1, LOW);

      analogWrite(amp2, speed2);  // speed for DC Motor 2
      digitalWrite(le_amp2, HIGH);
      digitalWrite(re_amp2, LOW);
    }

    //    Serial.print("amp1 "); Serial.print(speed1);
    //    Serial.print("amp2 "); Serial.println(speed2);
    //Serial.println("Feeder started");
  }
  else {
    digitalWrite(re_amp2, LOW);
    digitalWrite(le_amp2, LOW);
    digitalWrite(re_amp1, LOW);
    digitalWrite(le_amp1, LOW);
    analogWrite(amp2, 0);
    analogWrite(amp1, 0);
    //Serial.println("Feeder stopped");
  }
}
///////////
