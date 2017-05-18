#include <Servo.h>



int state = 0;
Servo myServoX;
Servo myServoY;
int servoPosX = 90;
int servoPosY = 90;

void setup() {
	Serial.begin(9600);
	pinMode(LED_BUILTIN, OUTPUT);

	myServoX.attach(9);
	myServoY.attach(11);

	myServoX.write(servoPosX);
	myServoY.write(servoPosY);

}

void loop() {
	while (Serial.available() > 0)
	{
		char incommingByte = Serial.read();
		if (incommingByte == 'V')
		{
			int xAxisMovement = Serial.parseInt();
			int yAxisMovement = Serial.parseInt();
			Serial.print("Recieved: (X: ");
			Serial.print(xAxisMovement);
			Serial.print(",Y: ");
			Serial.print(yAxisMovement);
			Serial.print(+")");

			if (abs(xAxisMovement) > 5 || abs(yAxisMovement) > 5)
			{
				//X Axis
				if (xAxisMovement < 0)
				{
					servoPosX = min(170, servoPosX + 1);
				}
				else {
					servoPosX = max(servoPosX - 1, 10);
				}

				//Y Axis
				if (yAxisMovement < 0)
				{
					servoPosY = min(150, servoPosY + 1);
				}
				else {
					servoPosY = max(servoPosY - 1, 30);
				}

				//move servos accordingly
				myServoY.write(servoPosY);
				myServoX.write(servoPosX);

			}

			//       if(state == 0) {
			//        state = 1;
			//        digitalWrite(LED_BUILTIN, HIGH);
			//        servoPosY -= 10;
			//        servoPosX -=10;
			//        myServoY.write(servoPosY);
			//        myServoX.write(servoPosX);
			//       }
			//       else {
			//        state = 0;
			//        digitalWrite(LED_BUILTIN, LOW);
			//        servoPosY +=10;
			//        servoPosX +=10;
			//        myServoY.write(servoPosY);
			//        myServoX.write(servoPosX);         
			//       }
			//delay(5);

		}
	}

}