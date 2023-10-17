/*
Version 6.0
Date: December 6, 2022
Source code for forklift ROBOT
*/

#include "UW_sensorMux.c"
#include "PC_FileIO.c"

//Sensor Calibration Function
void sensorConfig();

//Base Driving Functions
void driveBoth(int motor_PC, int motor_PD);
void driveDist(float dist, int speed);
void driveAngle(int angle, int speed, int &i_angle);

//Shipping Selection Functions
void displayShipping();
void selectShipping(int &colour, int &zone);

//Shelf Locating/Reaching Functions
void driveTillColour(int colour);
void driveUltra(int dist, int direction, int speed);

//Start/End Functions
void startup(int &i_angle);
void shutdown(int &i_angle);

//Shelf Management Functions
int colour_sorter(bool check, int colour, bool *shelf_inventory);
void storeblock(int shelf, int colour, bool *shelf_inventory, int *blocks_io, int &i_angle);
void shipblock(int shelf, int zone, int colour, bool *shelf_inventory, int *blocks_io, int &i_angle);

//File Output Functions
void arrayEdit(int colour, int store, int *blocks_io);
void outputFile(int *blocks_io);

//Forklift Movement Functions
void takeBlock(int shelf);
void putBlock(int shelf);


task main()
{
	//Initial Sensor/Variable Config
	sensorConfig();
	int colour = 0;
	int zone = 0;
	int shelf = 0;
	int i_angle = 0;
	bool shelf_inventory[6] = {0,0,0,0,0,0};
	int blocks_io[7] = {0,0,0,0,0,0,0};

	//Wait for User to Press Enter to Start
	displayBigTextLine(0,"PRESS ENTER");
	while(!getButtonPress(buttonEnter))
	{
	}
	while(getButtonPress(buttonEnter))
	{
	}

	//Robot Exits Garage and Goes to Receiving
	displayShipping();
	startup(i_angle);
	time1[T1] = 0;

	//Waits 30 seconds for a block to be received or shipping request.
	while(time1[T1] < 30000)
	{
		//User Selects Shipping
		if(getButtonPress(buttonEnter))
		{
			selectShipping(colour, zone);
			shelf = colour_sorter(1, colour, shelf_inventory);
			//Ships block if available
			if(shelf != -1)
			{
				shipblock(shelf, zone, colour, shelf_inventory, blocks_io, i_angle);
			}
			time1[T1] = 0; //Reset 30 second counter
		}

		//Block Received and Deposit
		if(readMuxSensor(msensor_S3_2)==(int)colorRed
			|| readMuxSensor(msensor_S3_2)==(int)colorGreen
			|| readMuxSensor(msensor_S3_2)==(int)colorBlue)
		{
			colour = readMuxSensor(msensor_S3_2);
			shelf = colour_sorter(0, colour, shelf_inventory);
			storeblock(shelf, colour, shelf_inventory, blocks_io, i_angle);
			time1[T1] = 0; //Reset 30 second counter
		}

		wait1Msec(100);
	}

	//Robot Returns to Garage and Outputs File
	shutdown(i_angle);
	outputFile(blocks_io);
}

/*
Function to configure all sensors.
*/
void sensorConfig()
{
	SensorType[S3] = sensorEV3_GenericI2C;
	wait1Msec(100);
	// configure each channel on the sensor mux
		//Ground Color
	if (!initSensorMux(msensor_S3_1, colorMeasureColor))
		return;
	//Forklift Color
	if (!initSensorMux(msensor_S3_2, colorMeasureColor))
		return;
	//Forklift Ultrasonic
	if (!initSensorMux(msensor_S3_3, sonarCM))
		return;

	//Gyro
	SensorType[S2] = sensorEV3_Gyro;
	wait1Msec(50);
	SensorMode[S2] = modeEV3Gyro_Calibration;
	wait1Msec(100);
	SensorMode[S2] = modeEV3Gyro_RateAndAngle;
	wait1Msec(50);

	//Front Facing Ultrasonic
	SensorType[S4] = sensorEV3_Ultrasonic;

	wait1Msec(100);
}

/*
Function to indicate drivetrain motor power.
*/
void driveBoth(int motor_PC, int motor_PD)
{
	motor[motorC] = -motor_PC;
	motor[motorD] = -motor_PD;
}

/*
Function to drive a specified distance using motor encoders.
*/
void driveDist(float dist, int speed)
{
	const float C_CM = (4*PI)/180;
	int c_dist = nMotorEncoder[motorC];
	int direction = 1;

	//Check if driving forward or backward
	if (dist < 0)
	{
		direction = -1;
	}

	//Drive until reaches specified distance
	driveBoth(speed*direction, speed*direction);
	while(direction*nMotorEncoder[motorC] > direction*((-dist/C_CM)+(c_dist)))
	{
	}
	driveBoth(0,0);
	wait1Msec(250);
}

/*
Function to drive to a specified angle based on the ideal expected angle.
*/
void driveAngle(int angle, int speed, int &i_angle)
{
	int direction = 1;
	i_angle += angle;

	//Checking direction from relative angle to ideal angle
	if(getGyroDegrees(S2) > i_angle)
	{
		direction = -1;
	}

	//Turn towards angle, then slow down when near the expected angle until reached.
	driveBoth(direction*speed,direction*-speed);
	while (direction*(getGyroDegrees(S2))<direction*(i_angle+(direction*-45)))
	{
	}
	driveBoth((direction*speed)/2,(direction*-speed)/2);
	while (direction*(getGyroDegrees(S2))<direction*(i_angle))
	{
	}
	driveBoth(0,0);
	wait1Msec(250);
}

/*
Function to display shipping request code.
*/
void displayShipping()
{
	displayBigTextLine(0,"PRESS ENTER");
	displayBigTextLine(3,"TO SHIP");
}

/*
Function to allow user to select a colour and zone to ship to.
*/
void selectShipping(int &colour, int &zone)
{
	//Wait for release
	while(getButtonPress(buttonAny))
	{
	}
	displayBigTextLine(0,"SELECT A");
	displayBigTextLine(3,"COLOURED BLOCK.");
	displayBigTextLine(6,"LEFT: RED");
	displayBigTextLine(9,"ENTER: GREEN");
	displayBigTextLine(12,"RIGHT: BLUE");

	//User selects colour of block to ship
	while(!getButtonPress(buttonLeft) && !getButtonPress(buttonEnter)
		&& !getButtonPress(buttonRight))
	{
	}
	if(getButtonPress(buttonLeft))
	{
		colour = (int)colorRed;
	}
	if(getButtonPress(buttonEnter))
	{
		colour = (int)colorGreen;
	}
	if(getButtonPress(buttonRight))
	{
		colour = (int)colorBlue;
	}

	//Release
	while(getButtonPress(buttonAny))
	{
	}
	displayBigTextLine(0,"SELECT A");
	displayBigTextLine(3,"SHIPPING ZONE.");
	displayBigTextLine(6,"LEFT: ZONE 1");
	displayBigTextLine(9,"ENTER:ZONE 2");
	displayBigTextLine(12,"RIGHT: ZONE 3");

	//User selects a zone to ship to
	while(!getButtonPress(buttonLeft) && !getButtonPress(buttonEnter)
		&& !getButtonPress(buttonRight))
	{
	}
	if(getButtonPress(buttonLeft))
	{
		zone = (int)colorRed;
	}
	if(getButtonPress(buttonEnter))
	{
		zone = (int)colorGreen;
	}
	if(getButtonPress(buttonRight))
	{
		zone = (int)colorBlue;
	}

	eraseDisplay();
	displayShipping();

	//Release
	while(getButtonPress(buttonAny))
	{
	}
}

/*
Function to drive till a specified colour is under the robot.
*/
void driveTillColour(int colour)
{
	driveBoth(20,20);
	while(readMuxSensor(msensor_S3_1) != colour)
	{
		wait1Msec(25);
	}
	driveBoth(0, 0);
	wait1Msec(250);
}

/*
Function to drive till the front facing ultrasonic
is a set distance from the wall.
*/
void driveUltra(int dist, int direction, int speed)
{
	driveBoth(speed*direction, speed*direction);
	while(SensorValue[S4]*direction > dist*direction || SensorValue[S4] == 0)
	{
	}
	driveBoth(0,0);
	wait1Msec(250);
}

/*
Function to move from garage to default position.
*/
void startup(int &i_angle)
{
	const float D_TO = 35;
	driveDist(50, 20);
	driveAngle(-90,20, i_angle);
	driveUltra(D_TO, 1, 20);
}

/*
Function to go from the default position back to the garage.
*/
void shutdown(int &i_angle)
{
	const float D_FROM = 60;
	driveUltra(D_FROM, -1, 20);
	driveAngle(90,20, i_angle);
	driveDist(-50, 20);
}

/*
Function to check if the specified colour of block is on a shelf.
*/
int colour_sorter(bool check, int colour, bool *shelf_inventory)
{
	int min_index = 0;
	int max_index = 0;

	//Changes the shelf to check depending on colour.
	if (colour == (int)colorRed)
	{
		max_index = 1;
	}
	else if (colour == (int)colorBlue)
	{
		min_index = 2;
		max_index = 3;
	}
	else if (colour == (int)colorGreen)
	{
		min_index = 4;
		max_index = 5;
	}

	//Checks the shelf levels to check if they are full.
	for (int count = min_index; count <= max_index; count++)
	{
		if (shelf_inventory[count] == check)
		{
			return count;
		}
	}

	return -1; // Overflow integer equivalent for false
}

/*
Function to drive from the receiving zone to store a block and back.
*/
void storeblock(int shelf, int colour, bool *shelf_inventory, int *blocks_io, int &i_angle)
{
	const float D_TO = 35;
	const float D_FROM = 40;

	//If To Overflow
	if(shelf == -1)
	{
		driveUltra(D_FROM, -1, 20);
		driveAngle(180, 20, i_angle);
		driveUltra(D_TO, 1, 20);
		putBlock(shelf);
		arrayEdit(colour, -1, blocks_io);
		driveUltra(D_FROM, -1, 20);
		driveAngle(-180, 20, i_angle);
		driveUltra(D_TO, 1, 20);
	}

	//If To Shelf
	else
	{
		//Go to Shelf
		driveUltra(D_FROM, -1, 20);
		driveAngle(180, 20, i_angle);
		driveTillColour((int)colorYellow);
		driveAngle(-90, 20, i_angle);
		driveTillColour(colour);
		driveAngle(-90, 20, i_angle);
		driveUltra(D_TO, 1, 20);
		putBlock(shelf);
		shelf_inventory[shelf] = 1;
		arrayEdit(colour, 0, blocks_io);
		//Go back to Receiving
		driveUltra(D_FROM, -1, 20);
		driveAngle(180, 20, i_angle);
		driveTillColour((int)colorYellow);
		driveDist(25, 20);
		driveAngle(90, 20, i_angle);
		driveTillColour((int)colorYellow);
		driveAngle(90, 20, i_angle);
		driveUltra(D_TO, 1, 20);
	}
}

/*
Function to pickup a block from a shelf
and ship it to a zone, then return to receiving.
*/
void shipblock(int shelf, int zone, int colour,
	bool *shelf_inventory, int *blocks_io, int &i_angle)
{
	const float D_TO = 35;
	const float D_FROM = 40;

	//Go to Shelf
	driveUltra(D_FROM, -1, 20);
	driveAngle(180, 20, i_angle);
	driveTillColour((int)colorYellow);
	driveAngle(-90, 20, i_angle);
	driveTillColour(colour);
	driveAngle(-90, 20, i_angle);
	driveUltra(D_TO, 1, 20);
	takeBlock(shelf);
	shelf_inventory[shelf] = 0;
	driveUltra(D_FROM, -1, 20);

	//Go to default ship
	driveAngle(180, 20, i_angle);
	driveTillColour((int)colorYellow);
	driveDist(30, 20);
	driveAngle(-90, 20, i_angle);
	driveTillColour((int)colorYellow);
	driveAngle(180, 20, i_angle);

	//Go to Zone
	driveTillColour(zone);
	driveAngle(-90, 20, i_angle);
	driveUltra(D_TO, 1, 20);
	putBlock(-1);
	arrayEdit(colour, 1, blocks_io);
	driveUltra(D_FROM, -1, 20);

	//Return to Receiving
	driveAngle(-180, 20, i_angle);
	driveTillColour((int)colorYellow);
	driveDist(-10, 20);
	driveAngle(-90, 20, i_angle);
	driveTillColour((int)colorYellow);
	driveAngle(90, 20, i_angle);
	driveUltra(D_TO, 1, 20);
}

/*
Function to accumulate received/shipped blocks in an array.
*/
void arrayEdit(int colour, int store, int *blocks_io)
{

	//If Overflow
	if (store == -1)
	{
		blocks_io[0] = blocks_io[0] +1;
	}

	//If Stored
	else if(store == 0)
	{
		if (colour == (int)colorRed)
		{
			blocks_io[1] = blocks_io[1] +1;
		}
		else if (colour == (int)colorGreen)
		{
			blocks_io[2] = blocks_io[2] +1;
		}
		else if (colour == (int)colorBlue)
		{
			blocks_io[3] = blocks_io[3] +1;
		}
	}

	//If Shipped
	else if(store == 1)
	{
		if (colour == (int)colorRed)
		{
			blocks_io[4] = blocks_io[4] +1;
		}
		else if (colour == (int)colorGreen)
		{
			blocks_io[5] = blocks_io[5] +1;
		}
		else if (colour == (int)colorBlue)
		{
			blocks_io[6] = blocks_io[6] +1;
		}
	}
}

/*
Function to output the blocks_io array as a text file.
*/
void outputFile(int *blocks_io)
{
	TFileHandle fout;
	openWritePC(fout, "shippingList.txt");

	string shippedStr = "# Blocks Shipped:";
	string redString = "Red: ";
	string greenString = "Green: ";
	string blueString = "Blue: ";
	string overloadStr = "# Overload: ";
	string storage = "# Blocks in Storage:";

	int overloadNum = blocks_io [0];
	int redShipped = blocks_io [1];
	int greenShipped = blocks_io [2];
	int blueShipped = blocks_io[3];
	int redStorage = blocks_io[4];
	int greenStorage = blocks_io[5];
	int blueStorage = blocks_io[6];

	writeTextPC(fout,overloadStr);
	writeLongPC(fout,overloadNum);
	writeEndlPC(fout);
	writeEndlPC(fout);

	writeTextPC(fout, shippedStr);
	writeEndlPC(fout);

	writeTextPC(fout,redString);
	writeLongPC(fout,redShipped);
	writeEndlPC(fout);

	writeTextPC(fout,greenString);
	writeLongPC(fout,greenShipped);
	writeEndlPC(fout);

	writeTextPC(fout,blueString);
	writeLongPC(fout,blueShipped);
	writeEndlPC(fout);
	writeEndlPC(fout);

	writeTextPC(fout, storage);
	writeEndlPC(fout);

	writeTextPC(fout,redString);
	writeLongPC(fout,redStorage);
	writeEndlPC(fout);

	writeTextPC(fout,greenString);
	writeLongPC(fout,greenStorage);
	writeEndlPC(fout);

	writeTextPC(fout,blueString);
	writeLongPC(fout,blueStorage);

	closeFilePC(fout);

}

/*
Function to take a block off from the shelf.
*/
void takeBlock(int shelf)
{
	float shelfH = 0;
	int time = 0;

	//Checking whether to take from top or bottom shelf.
	if ((shelf+1)%2!=0)
	{
		shelfH = (18.7*(3-0.5));
		time = 1000;
	}

	else if ((shelf+1)%2==0)
	{
		shelfH = (18.7*(18-2.5));
		time = 200;
	}

	//Make sure forklift is at the bottom
	motor[motorA] = -20;
	while (abs(nMotorEncoder[motorA]) > 0){}
	motor[motorA] = 0;

	//Raise forklift
	if ((shelf+1)%2==0){
		nMotorEncoder[motorA] = 0;
		motor[motorA] = -40;
		while (abs(nMotorEncoder[motorA]) < shelfH) {}
		motor[motorA] = 0;
	}

	//Drive into shelf
	nMotorEncoder[motorC] = 0;
	motor[motorC] = motor[motorD] = -10;
	while (abs(nMotorEncoder[motorC]) < 8*18.7)
	{
	}
	motor[motorC] = motor[motorD] = 0;

	//Raise forklift
	motor[motorA] = -20;
	wait1Msec(400);
	motor[motorA] = 0;

	//Back Out
	motor[motorC] = motor[motorD] = 10;
	while (abs(nMotorEncoder[motorC]) > 0)
	{
	}
	motor[motorC] = motor[motorD] = 0;

	//Back out further and bring forklift all the way down
	driveDist(-3,20);

	//Sensor Code that was replaced with hard coding due to sensor inconsistency
	/*motor[motorA] = 50;
	wait1Msec(25);
	while (readMuxSensor(msensor_S3_3) >= 4)
	{
		wait1Msec(25);
	}
	motor[motorA] = 0;
	*/

	if ((shelf+1)%2!=0)
	{
		motor[motorA] = 20;
		wait1Msec(1000);
		motor[motorA] = 0;
	}

	else if ((shelf+1)%2==0)
	{
		motor[motorA] = 20;
		wait1Msec(3000);
		motor[motorA] = 0;
	}

	//Bring forklift down
 	motor[motorA] = 10;
	wait1Msec(300);
	motor[motorA] = 0;


}

/*
Function to place a block onto the shelf.
*/
void putBlock(int shelf)
{

	float shelfH = 0;
	int time = 0;

	//If Overflow, set to bottom position.
	if(shelf == -1)
	{
		shelf++;
	}

	//Checking whether to place on top or bottom shelf.
	if ((shelf+1)%2!=0)
	{
		shelfH = (18.7*3);
		time = 1000;
	}

	else if ((shelf+1)%2==0)
	{
		shelfH = (18.7*18);
	  time = 200;
	}

	//Make sure forklift is at the bottom
	motor[motorA] = -20;
	while (abs(nMotorEncoder[motorA]) > 0)
	{
	}
	motor[motorA] = 0;

	//Raise forklift
	nMotorEncoder[motorA] = 0;
	motor[motorA] = -40;
	while (abs(nMotorEncoder[motorA]) < shelfH)
	{
	}
	motor[motorA] = 0;

	//Drive into shelf
	nMotorEncoder[motorC] = 0;
	motor[motorC] = motor[motorD] = -10;
	while (abs(nMotorEncoder[motorC]) < 8*18.7)
	{
	}
	motor[motorC] = motor[motorD] = 0;

	//Bring forklift down
 	motor[motorA] = 30;
	wait1Msec(time);
	motor[motorA] = 0;
	wait1Msec(500);

	//Back Out
	motor[motorC] = motor[motorD] = 10;
	while (abs(nMotorEncoder[motorC]) > 0)
	{
	}
	motor[motorC] = motor[motorD] = 0;

	//Back out further and bring lift all the way down.
	driveDist(-3,20);

	//Sensor Code that was replaced with hard coding due to sensor inconsistency
	/*
	motor[motorA] = 50;
	wait1Msec(25);
	while (readMuxSensor(msensor_S3_3) >= 4)
		{
		wait1Msec(25);
		}
	motor[motorA] = 0;
	*/

	if ((shelf+1)%2!=0)
	{
		motor[motorA] = 20;
		wait1Msec(1000);
		motor[motorA] = 0;
	}

	else if ((shelf+1)%2==0)
	{
		motor[motorA] = 20;
		wait1Msec(3000);
		motor[motorA] = 0;
	}

	//Bring forklift down
 	motor[motorA] = 10;
	wait1Msec(300);
	motor[motorA] = 0;

}
