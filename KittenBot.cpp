#include"KittenBot.h"
#include "AccelStepper.h"
#include "MultiStepper.h"
#include "Adafruit_NeoPixel.h"

#define M1_A 5
#define M1_B 6
#define M2_A 9
#define M2_B 10
#define M3_A 7
#define M3_B 8
#define M4_A 12
#define M4_B 13

AccelStepper stpA(AccelStepper::FULL4WIRE, 5, 9, 6, 10);
AccelStepper stpB(AccelStepper::FULL4WIRE, 7, 12, 8, 13);
MultiStepper steppers;
Adafruit_NeoPixel rgbled(16);

KittenBot::KittenBot()
{
	stpA.setMaxSpeed(600.0);
	stpA.setAcceleration(200.0);
	stpB.setMaxSpeed(600.0);
	stpB.setAcceleration(200.0);
	steppers.addStepper(stpA);
	steppers.addStepper(stpB);
	rgbled.begin();
	stepMoving = false;
	enableM[0] = enableM[1] = 1;
	enableM[2] = enableM[3] = 0;
	ppm = 14124;
	baseWidth = 0.122;
	for(int i=0;i<8;i++){
		pinMode(MotorPin[i],OUTPUT);
		digitalWrite(MotorPin[i],0);
	}
}

void KittenBot::enableMotor(int m1, int m2, int m3, int m4){
	enableM[0] = m1;
	enableM[1] = m2;
	enableM[2] = m3;
	enableM[3] = m4;
}

void KittenBot::motorStop()
{
	for(int i=0;i<4;i++) spdM[i] = 0;
}

void KittenBot::stopAll()
{
	analogWrite(5,0);
	analogWrite(6,0);
	analogWrite(9,0);
	analogWrite(10,0);
}

void KittenBot::rgbShow(int pin, int pix, int r, int g, int b){
	rgbled.setPin(pin);
	if(pix==0){
		for(int i=0;i<16;i++){
		  rgbled.setPixelColor(i, r, g, b);
		}
	}else{
		rgbled.setPixelColor(pix-1, r, g, b);  
	}
	rgbled.show();	
}

void KittenBot::runDCMotor(int idx, int spd){
	if(idx==0){
		if(spd>=0){
			analogWrite(5,spd);
			analogWrite(6,0);
		}else{
			analogWrite(5,0);
			analogWrite(6,-spd);
		}
	}else if(idx==1){
		if(spd>=0){
			analogWrite(9,spd);
			analogWrite(10,0);
		}else{
			analogWrite(9,0);
			analogWrite(10,-spd);
		}
	}
	
}

void KittenBot::loop()
{
	if (stpA.distanceToGo() != 0 || stpB.distanceToGo() != 0) {
		stpA.runSpeedToPosition();
		stpB.runSpeedToPosition();
	} else {
		if (stepMoving == true) {
		  stepMoving = false;
		  Serial.println("M100");
		}
	}
	// update dc motor
	if (micros() - timecount > 100) {
		timecount = micros();
		if(enableM[0] && counter == abs(spdM[0])){
		  digitalWrite(M1_A, 0);
		  digitalWrite(M1_B, 0);
		}
		if(enableM[1] && counter == abs(spdM[1])){
		  digitalWrite(M2_A, 0);
		  digitalWrite(M2_B, 0);
		}
		if(enableM[2] && counter == abs(spdM[2])){
		  digitalWrite(M3_A, 0);
		  digitalWrite(M3_B, 0);
		}
		if(enableM[3] && counter == abs(spdM[3])){
		  digitalWrite(M4_A, 0);
		  digitalWrite(M4_B, 0);
		}
		counter++;
		if (counter >= 255) {
		  if(enableM[0]){
			  digitalWrite(M1_A, 0);
			  digitalWrite(M1_B, 0);
		  }
		  if(enableM[1]){
			  digitalWrite(M2_A, 0);
			  digitalWrite(M2_B, 0);
		  }
		  if(enableM[2]){
			digitalWrite(M3_A, 0);
			digitalWrite(M3_B, 0);
		  }
		  if(enableM[3]){
			digitalWrite(M4_A, 0);
			digitalWrite(M4_B, 0);
		  }
		  counter = 0;
		  if(enableM[0]){
			  if (spdM[0] > 0) {
				digitalWrite(M1_A, 1);
			  } else if (spdM[0] < 0) {
				digitalWrite(M1_B, 1);
			  }
		  }
		  if(enableM[1]){
			  if (spdM[1] > 0) {
				digitalWrite(M2_A, 1);
			  } else if (spdM[1] < 0) {
				digitalWrite(M2_B, 1);
			  }
		  }
		  if(enableM[2]){
			  if (spdM[2] > 0) {
				digitalWrite(M3_A, 1);
			  } else if (spdM[2] < 0) {
				digitalWrite(M3_B, 1);
			  }
		  }
		  if(enableM[3]){
			  if (spdM[3] > 0) {
				digitalWrite(M4_A, 1);
			  } else if (spdM[3] < 0) {
				digitalWrite(M4_B, 1);
			  }
		  }
		}
	}
}

float KittenBot::getBatteryVoltage()
{
	int a = analogRead(A7);
	float v = float(a) / 1024.0 * 5.2 * 2;
	return v;
}


void KittenBot::stepRun(int pos1, int pos2){
	stepRun(pos1,600,pos2,600);
}

void KittenBot::stepRun(int pos1, int spd1, int pos2, int spd2){
	stpA.move(pos1);
	stpA.setSpeed(spd1);
	stpB.move(pos2);
	stpB.setSpeed(spd2);
	if (spd1 == 0 && spd2 == 0) {
		stepStop();
	} else {
		stepMoving = true;
	}	
	
}

void KittenBot::stepMove(float l){
	long positions[2];
	positions[0] = -l*ppm/100;
	positions[1] = l*ppm/100;
    steppers.moveTo(positions);
	steppers.runSpeedToPosition();
	stpA.disableOutputs();
	stpB.disableOutputs();
}

void KittenBot::stepMoveByIndex(int index, int pos, int speed){
	if(index==0){
		stpA.move(pos);
		stpA.setSpeed(speed);
		stpA.runToPosition();
		stpA.disableOutputs();
	}else{
		stpB.move(pos);
		stpB.setSpeed(speed);
		stpB.runToPosition();
		stpB.disableOutputs();
	}
	
}

void KittenBot::stepMoveMultiple(int pos1, int speed1, int pos2, int speed2){
	long positions[2];
	positions[0] = pos1;
	positions[1] = pos2;
	stpA.setSpeed(speed1);
	stpB.setSpeed(speed2);
    steppers.moveTo(positions);
	steppers.runSpeedToPosition();
	stpA.disableOutputs();
	stpB.disableOutputs();
}

void KittenBot::stepTurn(float d){
	long positions[2];
	///180.0*3.141*KittenBot.BASE_WIDTH/2.0*KittenBot.PULSE_PER_METER
	float dis = -d/180*3.14*ppm*baseWidth/2.0; // todo: the direction perform different to online mode
	positions[0] = dis;
	positions[1] = dis;
    steppers.moveTo(positions);
	steppers.runSpeedToPosition();	
	stpA.disableOutputs();
	stpB.disableOutputs();
}

void KittenBot::stepStop(){
	stpA.stop();
	stpB.stop();
}


void KittenBot::motorRun(int m1, int m2){
	spdM[0] = m1;
	spdM[1] = m2;
}

void KittenBot::motorRun(int m1, int m2, int m3, int m4){
	spdM[0] = m1;
	spdM[1] = m2;
	spdM[2] = m3;
	spdM[3] = m4;
}

void KittenBot::motorRunByIndex(int idx, int spd){
	if(enableM[idx]==0){
		enableM[idx] = 1;
	}
	spdM[idx] = spd;
}

int KittenBot::doPingSR04(int pin)
{
	pinMode(pin, OUTPUT); 
	digitalWrite(pin, LOW); // 使发出发出超声波信号接口低电平2μs
	delayMicroseconds(2);
	digitalWrite(pin, HIGH); // 使发出发出超声波信号接口高电平10μs，这里是至少10μs
	delayMicroseconds(10);
	digitalWrite(pin, LOW); // 保持发出超声波信号接口低电平
	pinMode(pin, INPUT);
	int distance = pulseIn(pin, HIGH); // 读出脉冲时间
	distance= distance/58; // 将脉冲时间转化为距离（单位：厘米）
	
	if(distance == 0){
		distance = 999;
	}
	return distance;
}