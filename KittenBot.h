#ifndef KittenBot_h
#define KittenBot_h

#define INFINITY 65535

class KittenBot
{
	private:
		int spdM[4];
		bool stepMoving;
		unsigned long timecount = 0;
		int counter = 0;
	public:
		float ppm;
		float baseWidth;
		KittenBot();
		void loop(); // Main loop for kittenbot lib
		float getBatteryVoltage(); // Vin voltage from A7

		void motorRun(int spd1, int spd2); // drive dual motor
		void motorRun(int spd1, int spd2, int spd3, int spd4); // drive 4 motor
		void motorRunByIndex(int idx, int spd);
		void motorStop(void);
		void runDCMotor(int idx, int spd);
		void stopAll(void);
		
		void stepRun(int pos1, int pos2);
		void stepRun(int pos1, int spd1, int pos2, int spd2);
		void stepMove(float length);
		void stepTurn(float degree);
		void stepArc(float diameter, float degree);
		void stepStop(void);
		
		float doPingSR04(int pin);
	

	protected:
		unsigned char MotorPin[8] = { 5,6,9,10,7,8,12,13 };

};
#endif