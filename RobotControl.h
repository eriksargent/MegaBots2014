#include "WPILib.h"
#include "MecanumDrive.h"
#include "iostream.h"
#include "NetworkTables/NetworkTable.h"
#include "math.h"
#include "AnalogChannel.h"

class RobotControl : public IterativeRobot {
private:
	float cogArea, sonarR, sonarL, multiPotValue, shooterThrottle;		//define all these variables
	double throttle;
	bool upLimit, lowLimit, upTripped, align;		//and these
	void logs();		//create function called logs
	void setShooters(double setPoint);		//create function called setShooters and a double variable setPoint
	
	MecanumDrive drive;		//define a MecanumDrive function called drive
	
	Joystick *control;		//define a joystick called control (kaden)
	Joystick *notKaden;		//define a joystick called notKaden
	
	DigitalInput *upperLimit;		//define digital input as upperLimit
	DigitalInput *lowerLimit;		//define digital input as lowerLimit
	
	Compressor *compressor;
	Solenoid *prodSR;
	Solenoid *prodSL;
	
	//define the individual Jaguars on the shooter as shooters 1, 2, 3, and 4
	CANJaguar *shooter_1;
	CANJaguar *shooter_2;
	CANJaguar *shooter_3;
	CANJaguar *shooter_4;
	
	Jaguar *prodR;		//define prodR as a jaguar
	Jaguar *prodL;		//define prodL as a jaguar
public:					//create public variables/functions
	NetworkTable *table;		//create network table called table
	AnalogChannel *ultra1;		//create analog channel called ultra1 (Ultrasonic sensor)
	AnalogChannel *ultra2;		//create analog channel called ultra2
	AnalogChannel *multiPot;	//create analog channel called multiPot (Potentiometer)
	RobotControl();				//create function called RobotControl, which is control for the robot
	
	//provide for initialization at robot power-on
	void RobotInit();
	
	//Distance tracking
	float distance();
		
	//Autonomous vision tracking
	bool HotOrNot();
	
	//called only when first disabled
	void DisabledInit();
	
	//called each and every time autonomous is entered from another mode
	void AutonomousInit();
	
	//called each and every time teleop is entered from another mode
	void TeleopInit();
	
	/*
	 * Periodic functions are called iteratively at the 
	 * appropriate periodic rate (aka the "slow loop"). 
	 * By default, this is synced to the driver station control packets, about 50Hz
	*/

	void DisabledPeriodic();		//What happens while the robot is disabled

	void AutonomousPeriodic();		//what happens while the robot is in autonomous mode

	void TeleopPeriodic();		//what happens while the robot is in teleop mode
};
