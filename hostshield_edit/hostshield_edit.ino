#include <PS5BT.h>
#include <usbhub.h>
#include <SPI.h>

USB Usb;
BTD Btd(&Usb);
PS5BT PS5(&Btd, PAIR);

bool printAngle = false, printTouch = false;
uint16_t lastMessageCounter = -1;
uint8_t player_led_mask = 0;
bool microphone_led = false;
uint32_t ps_timer;

int xj1, yj1, xj2, yj2, r2, l2;
int counter = 0;
char data = '0';

bool bldcState = false;
int mode = 0;
void setup() {
  Serial.begin(9600);
  if (Usb.Init() == -1) {
    while (1);
  }
}

void loop() {
  Usb.Task();

  if (PS5.connected() && lastMessageCounter != PS5.getMessageCounter()) {
    lastMessageCounter = PS5.getMessageCounter();

    if (Serial.available()) {
      data = Serial.read();


      if (data == '0') {
        PS5.setLed(0, 255, 255);
      }
    }
    else if (data == '1') {
      PS5.setLed(0, 255, 255);
    }

    if (counter == 0) {
      counter = 1;
      PS5.setPlayerLed(0b00000100);
    }

    xj1 = PS5.getAnalogHat(LeftHatX);
    yj1 = PS5.getAnalogHat(LeftHatY);
    xj2 = PS5.getAnalogHat(RightHatX);
    yj2 = PS5.getAnalogHat(RightHatY);
    r2 = PS5.getAnalogButton(R2);
    l2 = PS5.getAnalogButton(L2);

    xj1 = map(xj1, 0, 255, 0, 200);
    yj1 = map(yj1, 0, 255, 0, 200);
    xj2 = map(xj2, 0, 255, 0, 200);
    yj2 = map(yj2, 0, 255, 0, 200);
    r2 = map(r2, 0, 255, 0, 200);
    l2 = map(l2, 0, 255, 0, 200);

    Serial.write(201);
    Serial.write(xj1);
    Usb.Task();
    delay_ms(10);
    Serial.write(202);
    Serial.write(yj1);
    Usb.Task();
    delay_ms(10);
    Serial.write(203);
    Serial.write(xj2);
    Usb.Task();
    delay_ms(10);
    Serial.write(204);
    Serial.write(yj2);
    delay_ms(10);
    Serial.write(205);
    Serial.write(r2);
    delay_ms(10);
    Serial.write(206);
    Serial.write(l2);
    delay_ms(10);
    Usb.Task();

    if (PS5.getButtonClick(RIGHT)) Serial.write(244);
    Usb.Task();
    if (PS5.getButtonClick(DOWN)) Serial.write(243);
    Usb.Task();
    if (PS5.getButtonClick(LEFT)) Serial.write(242);
    Usb.Task();
    if (PS5.getButtonClick(UP)) Serial.write(241);
    Usb.Task();
    if (PS5.getButtonClick(SQUARE)) Serial.write(238);
    Usb.Task();
    if (PS5.getButtonClick(R1))Serial.write(232);
    Usb.Task();
    if (PS5.getButtonClick(CROSS)) Serial.write(239);
    Usb.Task();
    if (PS5.getButtonClick(CIRCLE)) Serial.write(240);
    Usb.Task();
    if (PS5.getButtonClick(TRIANGLE)) Serial.write(237);
    Usb.Task();
    if (PS5.getButtonClick(L1)) Serial.write(231);
    Usb.Task();
    if (l2 >= 180) Serial.write(250);
    Usb.Task();
    if (r2 >= 180) Serial.write(251);
    Usb.Task();
    if (PS5.getButtonClick(MICROPHONE)) Serial.write(233);
    Usb.Task();
    if (PS5.getButtonClick(SHARE)) Serial.write(246);// left side next to up
    Usb.Task();
    if (PS5.getButtonClick(OPTIONS)) Serial.write(245);// right side next to triangle
    Usb.Task();
    if (PS5.getButtonClick(PS)) Serial.write(236);
    Usb.Task();
    if (PS5.getButtonClick(R3)) Serial.write(248);
    Usb.Task();
    if (PS5.getButtonClick(L3)) Serial.write(247);
    Usb.Task();
    //  if (PS5.getButtonClick(TOUCHPAD)) Serial.write(249);
    //  Usb.Task();
    if (PS5.getButtonClick(PS)) Serial.write(234);
    Usb.Task();

    //xj2 yj2 mode colour change
    if (xj2 > 180) {
      mode = 1;
    }
    if (xj2 < 20) {
      mode = 2;
    }
    if (yj2 > 180) {
      mode = 3;
    }
    if (yj2 < 20) {
      mode = 4;
    }

    clr(mode);
  }
}


void delay_ms(int ms) {
  long int currentTime = millis();
  while (millis() - currentTime < ms) {
    Usb.Task();
  }
}

void clr(int num){
  if(num==1){
    PS5.setLed(0,0,255);
  }
  if(num==2){
    PS5.setLed(255,255,0);
  }
  if(num==3){
    PS5.setLed(165,42,42);
  }
  if(num==4){
    PS5.setLed(0,255,0);
  }
}



//connected cyan
//sirf drive blue
//shooting yellow
//brown baaki

//defence purple
//passing orange
