#ifndef KittenBot_h
#define KittenBot_h

#define INF_NUM 65535

#define M1 0
#define M2 1
#define M3 2
#define M4 3

class KittenBot
{
	private:
		int spdM[4];
		bool enableM[4];
		unsigned long timecount = 0;
		int counter = 0;
		long stepPos[2];
	public:
		float getTimer(int timerindex);
		void resetTimer(int timerindex);
		float ppm;
		float baseWidth;
		KittenBot();
		void loop(); // Main loop for motors
		void updateT();
		float getBatteryVoltage(); // Vin voltage from A7
		
		void motorRun(int spd1, int spd2); // drive dual motor
		void motorRun(int spd1, int spd2, int spd3, int spd4); // drive 4 motor
		void enableMotor(int m1, int m2, int m3, int m4);
		void motorRunByIndex(int idx, int spd);
		void motorStop(void);
		void runDCMotor(int idx, int spd);
		void stopAll(void);
		void rgbShow(int pin, int pix, int r, int g, int b);
		
		void stepRun(int pos1, int pos2);
		void stepRun(int pos1, int spd1, int pos2, int spd2);
		void stepMove(float length);
		void stepTurn(float degree);
		void stepArc(float diameter, float degree);
		void stepMoveByIndex(int index, int pos, int speed);
		void stepMoveMultiple(int pos1, int speed1, int pos2, int speed2);
		void stepStop(void);
		
		int doPingSR04(int pin);
		int doPingSR04(int trigPin, int echoPin);

        float getDS18B20Temp(int pin);   //获取DS18B20温度
		float getDHT11TempHum(double TempHum[2], int pin);     //获取DHT11  温度 湿度
//	    float getDHT11Hum();       //获取DHT11  湿度

	protected:
		unsigned char MotorPin[8] = { 5,6,9,10,7,8,12,13 };

};
#endif