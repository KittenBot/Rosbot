#include <Wire.h>
#include <EEPROM.h>
#include <IRremote.h>
#include <LiquidCrystal_I2C.h>
#include <TM1637Display.h>
#include "LedControl.h"
#include "ServoTimer2.h"
#include "AccelStepper.h"
#include "KittenBot.h"
#include "Timer.h"

#define FIRMWARE "Kittenbot V2.6\r\n"

ServoTimer2 servo[11];
unsigned char servoPinMap[10]  ={4,7,8,11,12,13,A0,A1,A2,A3};
IRrecv irrecv;
decode_results results;
LiquidCrystal_I2C lcd(0x3f, 16, 2);
TM1637Display tm1637;
LedControl lc = LedControl(4, 7, 8, 1);
Timer timer;
KittenBot kb;

#define QUERY_NULL 0
#define QUERY_DIGI 1
#define QUERY_ANALOG 2
#define QUERY_SONIC 3
#define QUERY_SONIC_CLASSIC 4

typedef struct {
  unsigned char type;
  unsigned char v0;
  unsigned char v1;
}DataQuery;

DataQuery query[10];
static union {
  struct {
    unsigned int sign;
    float dcdiff;
  } data;
  char buf[16];
} robotSetup;

int16_t ax, ay, az;
int16_t gx, gy, gz;
bool stepMoving = false;
bool LCD = false;
float cos45 = cos(PI/4);

// parse pin, 0~13 digital, 14.. analog pin
void parsePinVal(char * cmd, int * pin) {
  if (cmd[0] == 'A') {
    sscanf(cmd, "A%d\n", pin);
    *pin += A0;
  } else {
    sscanf(cmd, "%d\n", pin);
  }
}


void parsePinVal(char * cmd, int * pin, int * v0) {
  if (cmd[0] == 'A') {
    sscanf(cmd, "A%d %d\n", pin, v0);
    *pin += A0;
  } else {
    sscanf(cmd, "%d %d\n", pin, v0);
  }
}

void parsePinVal(char * cmd, int * pin, int * v0, int * v1) {
  if (cmd[0] == 'A') {
    sscanf(cmd, "A%d %d %d\n", pin, v0, v1);
    *pin += A0;
  } else {
    sscanf(cmd, "%d %d %d\n", pin, v0, v1);
  }
}

void parsePinVal(char * cmd, int * pin, int * v0, int * v1, int * v2, int * v3) {
  if (cmd[0] == 'A') {
    sscanf(cmd, "A%d %d %d %d %d\n", pin, v0, v1, v2, v3);
    *pin += A0;
  } else {
    sscanf(cmd, "%d %d %d %d %d\n", pin, v0, v1, v2, v3);
  }
}

// parse left or right value
void parseLR(char * cmd, int * lvalue, int * rvalue) {
  char * tmp;
  char * str;
  *lvalue = 0;
  *rvalue = 0;

  str = cmd;
  tmp = cmd;
  while (str != NULL) {
    str = strtok_r(0, " ", &tmp);
    if (str[0] == 'L') {
      *lvalue = atoi(str + 1);
    } else if (str[0] == 'R') {
      *rvalue = atoi(str + 1);
    }
  }
}

// parse left or right value
void parseLR(char * cmd, int * lvalue, int * rvalue, int * lspd, int * rspd) {
  char * tmp;
  char * str;
  *lvalue = 0;
  *rvalue = 0;
  *lspd = 400;
  *rspd = 400;

  str = cmd;
  tmp = cmd;
  while (str != NULL) {
    str = strtok_r(0, " ", &tmp);
    if (str[0] == 'L') {
      *lvalue = atoi(str + 1);
    } else if (str[0] == 'R') {
      *rvalue = atoi(str + 1);
    } else if (str[0] == 'A') {
      *lspd = atoi(str + 1);
    } else if (str[0] == 'B') {
      *rspd = atoi(str + 1);
    }
  }

}
void parseABCD(char *cmd, int *Avalue, int *Bvalue, int *Cvalue, int *Dvalue)
{
  char *tmp;
  char *str;
  *Avalue = 0;
  *Bvalue = 0;
  *Cvalue = 0;
  *Dvalue = 0;

  str = cmd;
  tmp = cmd;
  while (str != NULL) {
    str = strtok_r(0, " ", &tmp);
    if (str[0] == 'A') {
      *Avalue = atoi(str + 1);
    }
    else if (str[0] == 'B') {
      *Bvalue = atoi(str + 1);
    }
    else if (str[0] == 'C') {
      *Cvalue = atoi(str + 1);
    }
    else if (str[0] == 'D') {
      *Dvalue = atoi(str + 1);
    }
  }
}

void printPin(int pin) {
  if (pin >= 14) {
    Serial.print("A" + String(pin - 14));
  } else {
    Serial.print(String(pin));
  }
}

void echoPinValue(const char * code, int pin, int value) {
  Serial.print(code);Serial.print(" ");
  printPin(pin);Serial.print(" ");
  Serial.println(value);
}

void echoPinValue(const char * code, int pin, float value) {
  Serial.print(code);Serial.print(" ");
  printPin(pin);Serial.print(" ");
  Serial.println(value, 2);
}


void echoValue(const char * code, float value) {
  Serial.print(code);Serial.print(" ");
  Serial.println(value, 2);
}

void echoVersion() {
  Serial.print("M0 ");
  Serial.print(FIRMWARE);
}

void doPinMode(char * cmd) {
  int pin, mod;
  parsePinVal(cmd, &pin, &mod);
  pinMode(pin, mod);
}

void doDigitalWrite(char * cmd) {
  int pin, val;
  parsePinVal(cmd, &pin, &val);
  digitalWrite(pin, val);
}

void doDigitalRead(char * cmd) {
  int pin, val;
  parsePinVal(cmd, &pin);
  pinMode(pin, INPUT);
  val = digitalRead(pin);
  echoPinValue("M3", pin, val);

}

void doButton(char * cmd){
  int pin, val;
  parsePinVal(cmd, &pin);
  pinMode(pin, INPUT);
  val = !digitalRead(pin);
  echoPinValue("M10", pin, val);
}

void doAnalogWrite(char * cmd) {
  int pin, val;
  parsePinVal(cmd, &pin, &val);
  if (pin == 3 || pin == 5 || pin == 6 || pin == 9 || pin == 10 || pin == 11) { // only work on 3,5,6,9,10,11
    analogWrite(pin, val);
  }
}

void doAnalogRead(char * cmd) {
  int pin, val;
  parsePinVal(cmd, &pin);
  val = analogRead(pin);
  if (pin < 14) return;
  echoPinValue("M5", pin, val);
}

void doTone(char * cmd) {
  int pin, freq, t;
  parsePinVal(cmd, &pin, &freq, &t);
  tone(pin, freq, t);
}

void doServo(char * cmd) {
  int pin, degree;
  parsePinVal(cmd, &pin, &degree);
  // servotimer2 accept time pulse in ms
  servo[10].attach(pin);
  servo[10].write(degree);
}

void doServoArray(char * cmd){
  int idx, degree;
  parsePinVal(cmd, &idx, &degree);
  // servotimer2 accept time pulse in ms
  if(!servo[idx].attached()){
    servo[idx].attach(servoPinMap[idx]);
  }
  servo[idx].write(degree);
}

void doAppCmd(char * cmd){
  int i;
  parsePinVal(cmd, &i);
  // todo: what todo with app command
  
}

void doEchoVin() {
  float v;
  v = kb.getBatteryVoltage();
  echoValue("M8", v);
}

void doRgb(char * cmd) {
  int pin, pix, r, g, b;

  parsePinVal(cmd, &pin, &pix, &r, &g, &b);
  kb.rgbShow(pin,pix,r,g,b);
}

void doIRAttach(char * cmd) {
  int pin;
  parsePinVal(cmd, &pin);
  irrecv.attach(pin);
  irrecv.enableIRIn();
}

// --- M100 ---
void doStepperSingle(char * cmd) {
  int posL, posR, spdL, spdR;
  parseLR(cmd, &posL, &posR, &spdL, &spdR);
  kb.stepRun(posL,spdL,posR,spdR);
  Serial.println("M100");
}

void doStepperMove(char * cmd) {
  int stpL = 0, stpR = 0;
  parseLR(cmd, &stpL, &stpR);
  kb.stepRun(stpL, stpR);
  Serial.println("M100");
}
void doStop()
{
  kb.motorStop();
}

//--- M108 ---
void doEnableMotor(char * cmd){
  int m1, m2, m3, m4;
  sscanf(cmd, "%d %d %d %d\n", &m1, &m2, &m3, &m4);
  kb.enableMotor(m1, m2, m3, m4);
}

//--- M109 ---
void doOmniWheel(char * cmd){
  int spdM1, spdM2, spdM3, spdM4;
  int vspeed = 0, hspeed = 0, rspeed=0;
  sscanf(cmd, "%d %d %d\n", &vspeed, &hspeed, &rspeed);
  int tspd;
  // then map into 4 wheels
  tspd = vspeed/cos45;
  spdM1 = spdM4 = tspd;
  spdM2 = spdM3 = -tspd;
  tspd = hspeed/cos45;
  spdM1+=tspd;
  spdM2+=tspd;
  spdM3-=tspd;
  spdM4-=tspd;
  // no mapping for rotate
  spdM1+=rspeed;
  spdM2+=rspeed;
  spdM3+=rspeed;
  spdM4+=rspeed;
  // limit max and min value for each motor
  spdM1 = constrain(spdM1,-255,255);
  spdM2 = constrain(spdM2,-255,255);
  spdM3 = constrain(spdM3,-255,255);
  spdM4 = constrain(spdM4,-255,255);
  /*
  Serial.print("V=");Serial.print(vspeed);
  Serial.print(" ,H=");Serial.print(hspeed);
  Serial.print(" ,R=");Serial.print(rspeed);  
  Serial.print(" ,M1=");  Serial.print(spdM1);
  Serial.print(" ,M2=");  Serial.print(spdM2);
  Serial.print(" ,M3=");  Serial.print(spdM3);
  Serial.print(" ,M4=");  Serial.println(spdM4);
  */
  kb.motorRun(spdM1,spdM2,spdM3,spdM4);
}

void doGetTimer(char * cmd){
  int index = 0;
  parsePinVal(cmd,&index);
  float t = kb.getTimer(index);
  Serial.print("M111 ");
  Serial.print(index);Serial.print(" ");
  Serial.println(t);
}

void doResetTimer(char * cmd){
  int index = 0;
  parsePinVal(cmd,&index);
  kb.resetTimer(index);
}

//--- M200 ----
void doDcSpeed(char *cmd)
{
  int index = 0, spd = 0;
  parsePinVal(cmd,&index,&spd);

  kb.motorRunByIndex(index, spd);
}

// --- M201 ---
void doCarMove(char *cmd)
{
  int fw=0,lr=0;
  int lmotor,rmotor;
  parsePinVal(cmd,&fw,&lr);
  lmotor = (fw+lr);
  rmotor = (fw-lr)*robotSetup.data.dcdiff;
  kb.motorRun((int)lmotor,(int)rmotor);
}

void doShoot() {
  analogWrite(9, 0);
  analogWrite(10, 200);
  delay(300);
  analogWrite(9, 0);
  analogWrite(10, 0);
}

void doPing(char * cmd) {
  float distance;
  int trig, echo=-1;
  parsePinVal(cmd, &trig, &echo);
  if(echo>=0 && echo<A5){
    distance = kb.doPingSR04(trig,echo);  
  }else{
    distance = kb.doPingSR04(trig);
  }
  Serial.print("M202 ");
  Serial.println(distance);
}
void doReadDS18B20(char * cmd){
  float temp;
  int pin;
  parsePinVal(cmd, &pin);
  temp = kb.getDS18B20Temp(pin);
  Serial.println(temp);
}
void doReadDHT11(char * cmd){
  int pin;
  parsePinVal(cmd, &pin);
  
  double *TempHum = new double[2];
  kb.getDHT11TempHum(TempHum,pin);
  for(int i = 0;i < 2;i++)Serial.println(TempHum[i]);
  delete[] TempHum;
}
void doLCDPrint(char * cmd) {
  int i = 0;
  if (LCD == false)
  {
    lcd.init();
    lcd.backlight();
    LCD = true;
  }
  lcd.clear();
  while (cmd[i] != '\0' && cmd[i] != '\r' && cmd[i] != '\n') {
    lcd.write(cmd[i]);
    i++;
  }
}

void doTM1637Display(char * cmd) {
  int dio, clk, digi;
  parsePinVal(cmd, &clk, &dio, &digi);
  tm1637.setPins(clk, dio);
  tm1637.showNumberDec(digi);
}

void doLEDCtrl(char * cmd) {
  int c, r, onoff;
  parsePinVal(cmd, &c, &r, &onoff);
  lc.setLed(0, r, c, onoff);
}

void doLEDAttach(char * cmd) {
  int dio, clk, cs;
  parsePinVal(cmd, &dio, &clk, &cs);
  lc.attach(dio, clk, cs, 1);
  lc.shutdown(0, false);
  lc.setIntensity(0, 8);
  lc.clearDisplay(0);
}

void doWriteDcAdjust(char *cmd)
{
  robotSetup.data.dcdiff = atof(cmd);
  syncRobotSetup();
}

void doReadDcAdjust()
{
  Serial.print("M210 ");
  Serial.println(robotSetup.data.dcdiff);
}

void doSoftReset() {
  asm volatile ("  jmp 0");
}
void doStepperArc(char *cmd)
{
  int diameter, angle;
  sscanf(cmd, "%d %d\n", &diameter, &angle);
  float r = ((float)diameter)/100; // cm -> m
  kb.stepArc(r,angle);
}

void resetQueryList() {
  memset(query,0,sizeof(query));
}

void doAttachQuery(char * cmd){
  int type,v0,v1;
  parsePinVal(cmd, &v0, &type, &v1);
  
  for(int i=0;i<10;i++){
    if(query[i].type==0 || query[i].v0==v0){
      query[i].type = type;
      query[i].v0 = v0;
      if(type==QUERY_SONIC_CLASSIC){
        query[i].v1 = v1;
      }else if(type==QUERY_DIGI){
        pinMode(v0, INPUT);  
      }
      break;  
    }
  }
}

void doQueryWork() {
  int i;
  for (i = 0; i < 10; i++) {
    if (query[i].type) {
      if(query[i].type==QUERY_DIGI){
        int val = digitalRead(query[i].v0);
        echoPinValue("M3", query[i].v0, val);
      }else if(query[i].type==QUERY_ANALOG){
        int val = analogRead(query[i].v0);
        echoPinValue("M5", query[i].v0, val);
      }else if(query[i].type==QUERY_SONIC){
        int pin = query[i].v0;
        int val = kb.doPingSR04(pin);
        echoPinValue("M202",pin, val);
        //echoValue("M202", val);
      }else if(query[i].type==QUERY_SONIC_CLASSIC){
        int trig = query[i].v0;
        int echo = query[i].v1;
        int val = kb.doPingSR04(trig, echo);
        echoPinValue("M202",trig, val);
        //Serial.print("M202 ");Serial.print(trig);Serial.print(" ");Serial.println(val);
        //echoValue("M202", val);
      }
    }
  }
}

void parseMcode(char * cmd) {
  int code;
  char * tmp;
  code = atoi(cmd);
  cmd = strtok_r(cmd, " ", &tmp);

  switch (code) {
    case 0:
      echoVersion();
      break;
    case 1: // set pin mode: M1 pin mode
      doPinMode(tmp);
      break;
    case 2: // digital write: M2 pin level
      doDigitalWrite(tmp);
      break;
    case 3: // digital read: M3 pin
      doDigitalRead(tmp);
      break;
    case 4: // analog write: M4 pin pwm
      doAnalogWrite(tmp);
      break;
    case 5: // analog read: M5 pin
      doAnalogRead(tmp);
      break;
    case 6: // tone : M6 pin freq duration
      doTone(tmp);
      break;
    case 7: // servo : M7 pin degree
      doServo(tmp);
      break;
    case 8: // read vin voltage
      doEchoVin();
      break;
    case 9: // rgb led
      doRgb(tmp);
      break;
    case 10: // button
      doButton(tmp);
      break;
    case 13: // query
      doAttachQuery(tmp);
      break;
    case 14: // reset query
      resetQueryList();
      break;
    case 100: // set stepper move speed and position
      doStepperSingle(tmp);
      break;
    case 101: // step: M101 L1000 R-2000
      doStepperMove(tmp);
      break;
    case 102: // stop motors
      kb.motorStop();
      break;
    case 106: // disable stepper
      kb.stepStop();
      break;
    case 108:
      doEnableMotor(tmp);
      break;
    case 109: // omni wheel car movement
      doOmniWheel(tmp);
      break;
    case 111: // get timer
      doGetTimer(tmp);
      break;
    case 112: // reset timer
      doResetTimer(tmp);
      break;  
    case 200:
      doDcSpeed(tmp);
      break;
    case 201:
      doCarMove(tmp);
      break;
    case 202:
      doPing(tmp);
      break;
    case 203:
      doShoot();
      break;
    case 204:
      doIRAttach(tmp);
      break;
    case 205:
      doLCDPrint(tmp);
      break;
    case 206:
      doTM1637Display(tmp);
      break;
    case 207:
      doLEDCtrl(tmp);
      break;
    case 208:
      doLEDAttach(tmp);
      break;
    case 209:
      doWriteDcAdjust(tmp);
      break;
    case 210:
      doReadDcAdjust();
      break;
    case 211:
      doStepperArc(tmp);
      break;
    case 212: // servo array
      doServoArray(tmp);
      break;
    case 213: // app command
      doAppCmd(tmp);
    case 214:
      doReadDS18B20(tmp);
      break; 
    case 215:
      doReadDHT11(tmp);
      break;      
    case 999:
      doSoftReset();
    break;
  }

}

void parseCmd(char * cmd) {
  if (cmd[0] == 'G') { // gcode

  } else if (cmd[0] == 'M') { // mcode
    parseMcode(cmd + 1);
  }
  Serial.println("OK");
}

void initRobotSetup() {
  int i;
  for (i = 0; i < 16; i++) {
    robotSetup.buf[i] = EEPROM.read(i);
  }
  if (robotSetup.data.sign != 1234) {
    memset(robotSetup.buf, 0, 16);
    robotSetup.data.sign = 1234;
    robotSetup.data.dcdiff = 1.0;
    syncRobotSetup();
  }
}

void syncRobotSetup()
{
  int i;
  for (i = 0; i < 16; i++) {
    EEPROM.write(i, robotSetup.buf[i]);
  }
}

void setup() {
  Serial.begin(115200);
  
  resetQueryList();
  echoVersion();
  tm1637.setBrightness(0x0f);
  lc.shutdown(0, false);
  lc.setIntensity(0, 8);
  lc.clearDisplay(0);
  initRobotSetup();
  pinMode(7,OUTPUT);digitalWrite(7,0);
  pinMode(8,OUTPUT);digitalWrite(8,0);
  
  timer.every(100, doQueryWork);
}

char buf[64];
int8_t bufindex;

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    buf[bufindex++] = c;
    if (c == '\n') {
      buf[bufindex] = '\0';
      parseCmd(buf);
      memset(buf, 0, 64);
      bufindex = 0;
    }
    if (bufindex >= 64) {
      bufindex = 0;
    }
  }
  
  if (irrecv.enabled) {
    if (irrecv.decode(&results)) {
      if (results.value != 0xffffffff) {
        Serial.print("M204 ");
        Serial.println(results.value, HEX);
      }
      irrecv.resume(); // Receive the next value
    }
  }
  
  kb.loop();
  timer.update();
}


