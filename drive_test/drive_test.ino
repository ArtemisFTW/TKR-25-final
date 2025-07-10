  //roboclaw test
#include <RoboClaw.h>
#include<Wire.h>
RoboClaw roboclaw(&Serial1, 10000); ///serial 1 is removed now
void setup() {
  // put your setup code here, to run once:
  roboclaw.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  int u=10000;
roboclaw.DutyM1(128,-u);
roboclaw.DutyM2(128,-u);
roboclaw.DutyM2(129,-u);
}
