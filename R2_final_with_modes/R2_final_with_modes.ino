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

int b = 41;
int g = 43;
int r = 45;
/////////////////////////////

////////feeder//////////
int speed1 = 150, speed2 = 200;
int mult = 10;

int reset = 0;

int amp1 = 5;   ///aaage wala
int le_amp1 = 7;
int re_amp1 = 6;

int amp2 = 11;  //piche wala
int le_amp2 = 13;
int re_amp2 = 12;

#define fUp 0
#define fDown 1
bool feeder_check[2] = {false, false};
int feeder_val = 2;
////////////PID///////////////
float target = 0;
float output = 0;
float integral = 0;
float proportional = 0;
float derivative = 0;
float lastError = 0;
float piderror = 0;
float kp = 27;  //6.7 0.008 9.5
float ki = 0.0011;
float kd = 50;
float printp = 0, printi = 0, printd = 0;
/////////ps dis//////////
int count_RX = 0;

///////////align
int receivedAngle = 0;
int align_stat = 0;

int sign = 0;
bool passing = true; //passing at start
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

  if (butt[PS_START] == 1) {
    speed1 = 145;
    speed2 = 200;
    butt[PS_START] = 0;
  }

  if (butt[PS_SELECT] == 1) {
    asm volatile ("jmp 0");
    butt[PS_SELECT] = 0;
  }

  if (butt[PS_UP] == 1) {
    feeder_check[fUp] = !feeder_check[fUp];
    if (feeder_check[fUp]) feeder_check[fDown] = false;
    butt[PS_UP] = 0;
  }

  if (butt[PS_DOWN] == 1) {
    //Serial.println("down");
    feeder_check[fDown] = !feeder_check[fDown];
    if (feeder_check[fDown]) feeder_check[fUp] = false;
    butt[PS_DOWN] = 0;
  }

  if (butt[PS_LEFT] == 1) {
    sign = 1;
    butt[PS_LEFT] = 0;
  }

  if (butt[PS_RIGHT] == 1) {
    sign = -1;
    butt[PS_RIGHT] = 0;
  }

  if (butt[PS_SQUARE] == 1) {
    if (mode1 == 1) {
      target = target + ((sign) * 5);
      kp = 27;  //6.7 0.008 9.5
      ki = 0.0011;
      kd = 50;
    }

    if (mode2 == 1) {
      target = target + ((sign) * 30);
      kp = 20;  //6.7 0.008 9.5
      ki = 0.0011;
      kd = 50;
    }

    if (mode3 == 1) {
      speed1 = 140;
      speed2 = 180;
    }
    butt[PS_SQUARE] = 0;
  }

  if (butt[PS_CIRCLE] == 1) {
    if (mode1 == 1) {
      target = target + ((sign) * 15);
      kp = 27;  //6.7 0.008 9.5
      ki = 0.0011;
      kd = 50;
    }

    if (mode2 == 1) {
      target = target + ((sign) * 60);
      kp = 20;  //6.7 0.008 9.5
      ki = 0.0011;
      kd = 50;
    }

    else if (mode3 == 1) {
      speed1 = 150;
      speed2 = 200;
      //140-180 150-200 145-190 155 215
    }
    butt[PS_CIRCLE] = 0;
  }

  if (butt[PS_TRIANGLE] == 1) {
    if (mode1 == 1) {
      target = target + ((sign) * 10);
      kp = 27;  //6.7 0.008 9.5
      ki = 0.0011;
      kd = 50;
    }

    if (mode2 == 1) {
      target = target + ((sign) * 45);
      kp = 20;  //6.7 0.008 9.5
      ki = 0.0011;
      kd = 50;
    }

    if (mode3 == 1) {
      speed1 = 145;
      speed2 = 190;
    }
    butt[PS_TRIANGLE] = 0;
  }

  if (butt[PS_CROSS] == 1) {
    if (mode1 == 1) {
      target = target + ((sign) * 20);
      kp = 27;  //6.7 0.008 9.5
      ki = 0.0011;
      kd = 50;
    }

    if (mode2 == 1) {
      target = target + ((sign) * 90);
      kp = 20;  //6.7 0.008 9.5
      ki = 0.0011;
      kd = 50;
    }

    if (mode3 == 1) {
      speed1 = 155;
      speed2 = 215;
    }
    butt[PS_CROSS] = 0;
  }

  if (butt[PS_L1] == 1) {
    if (passing) {
      passing = false;
      target += 180;
      kp = 20;  //6.7 0.008 9.5
      ki = 0.0011;
      kd = 50;
    }
    else {
      target -= 180;
      kp = 20;  //6.7 0.008 9.5
      ki = 0.0011;
      kd = 50;
    }
    butt[PS_L1] = 0;
  }

  if (butt[PS_R1] == 1) {
    if (receivedAngle != 40) {
      target -= receivedAngle;
      kp = 27;  //6.7 0.008 9.5
      ki = 0.0011;
      kd = 50;
    }
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

  if (pot2 > 180) {
    roboclaw.DutyM1(129, 7000);  // Clockwise
  }
  else if (pot1 > 180) {
    roboclaw.DutyM1(129, -7000); // Anticlockwise
  }
  else {
    roboclaw.DutyM1(129, 0);     // Stop
  }

}
//////////setup/////////////
void setup() {
  ps_setup();
  roboclaw.begin(115200);
  Serial.begin(9600);
  Serial2.begin(9600);
  Wire.begin();

  pinMode(r, OUTPUT);
  pinMode(g, OUTPUT);
  pinMode(b, OUTPUT);
  Serial.println("Setup started");
  rgb(1, 1, 1);

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) { } // stop everything if could not connect to MPU6050

  //Serial.println(F("Calculating offsets, do not move MPU6050"));
  mpu.calcOffsets(); // gyro and accelero
  //Serial.println("Done!\n");

  pinMode(amp1, OUTPUT);
  pinMode(re_amp1, OUTPUT);
  pinMode(le_amp1, OUTPUT);

  pinMode(amp2, OUTPUT);
  pinMode(re_amp2, OUTPUT);
  pinMode(le_amp2, OUTPUT);

  Serial.println("Setup completed");
}
//////////////////////

////////loop//////
void loop() {
  //Serial.println("hello");
  dis_ps();
  if (count_RX <= 40)
  {
    ps_read();
    modecheck();
    if (mode1 == 1 || mode2 == 1)
    {
      fod();
      inv_kin(xj1, yj1, PID());

    }
    if (feeder_check[fUp]) {
      feeder_control(0);
    } else if (feeder_check[fDown]) {
      feeder_control(1);
    } else {
      feeder_control(2);  // stop
    }
    //Serial.print("amp1: "); Serial.print(speed1);
    //Serial.print(" || amp2: "); Serial.println(speed2);

    if (Serial2.available()) {
      int raw = Serial2.read();
      // 1-byte value from Jetson
      receivedAngle = raw - 128;
      //Serial.println(receivedAngle);
      alignment(receivedAngle);
      //          Serial.print("Received angle: ");
      //          Serial.print(receivedAngle);
      //          Serial.print(" || current");Serial.println(angle());
    }
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
    //Serial.println("PS disconnected");
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

  float z = map(Z, -200, 200, -45000, 45000); //maps value of output with range of speed of motors // 255
  float x = map(X, -255, 255, -105000, 105000); //maps xj1 values to max speed of motors //95k
  float y = map(Y, -255, 255, -105000, 105000); //maps yj1 values to max speed of motors

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

  roboclaw.DutyM1(128, v3);
  roboclaw.DutyM2(128, v2);// if on other roboclaw M1 129 -v2
  roboclaw.DutyM2(129,  -v1);

  //    Serial.print(" || v1 = "); Serial.print(v1);
  //    Serial.print(" || v2 = "); Serial.print(v2);
  //    Serial.print(" || v3 = "); Serial.print(v3);
  //  Serial.print("end = "); Serial.print(output);
  Serial.print(" || target = "); Serial.print(target);
  Serial.print(" || current = "); Serial.print(angle());
  Serial.print(" || error = "); Serial.println(piderror);
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

//////feeder direction and speed control function////////////
void feeder_control(int a) {
  if (a == 0) {
    bts_control(le_amp1, re_amp1, amp1, speed1);//feeder to go up
    bts_control(le_amp2, re_amp2, amp2, speed2);
  }
  else if (a == 1) {
    bts_control(re_amp1, le_amp1, amp1, speed1);//feeder to go down
    bts_control(re_amp2, le_amp2, amp2, speed2);
  }
  else {
    //    Serial.println("Feeder stopped");
    bts_control(le_amp1, re_amp1, amp1, 0);//feeder stop
    bts_control(le_amp2, re_amp2, amp2, 0);
  }
}
//////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////
void bts_control(int a, int b, int c, int d) {
  digitalWrite(a, HIGH);
  digitalWrite(b, LOW);
  analogWrite(c, d);
}
/////////////////////////////////////////////////////////////////

//////////////alingment angle check/////////////
void alignment(int ang) {
  if (ang == 40) {
    rgb(1, 0, 0);
  }
  else {
    if (abs(ang) < 2) {
      rgb(0, 1, 0);
    }
    else {
      rgb(1, 1, 0);
    }
  }
}
////////////////////////////////////

///////////rgb colour////////////////
void rgb(int a, int b, int c) {
  digitalWrite(r, a);
  digitalWrite(g, b);
  digitalWrite(b, c);
}
