#include "WPILib.h"
#include "MecanumDrive.h"
#include "iostream.h"
#include "NetworkTables/NetworkTable.h"
#include "math.h"
#include "AnalogChannel.h"

class RobotControl : public IterativeRobot {
private:
	float cogArea, sonarR, sonarL, multiPotValue, shooterThrottle;
	bool upLimit, lowLimit, upTripped, align;
	void logs();
	void setShooters(double setPoint);
	
	MecanumDrive drive;
	
	Joystick *control;
	Joystick *notKaden;
	
	DigitalInput *upperLimit;
	DigitalInput *lowerLimit;
	
	CANJaguar *shooter_1;
	CANJaguar *shooter_2;
	CANJaguar *shooter_3;
	CANJaguar *shooter_4;
	
	Relay *prodR;
	Relay *prodL;
public:
	NetworkTable *table;
	AnalogChannel *ultra1;
	AnalogChannel *ultra2;
	AnalogChannel *multiPot;
	RobotControl();
	
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

	void DisabledPeriodic();

	void AutonomousPeriodic();

	void TeleopPeriodic();
};
