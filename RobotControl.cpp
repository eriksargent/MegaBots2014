#include "RobotControl.h"

RobotControl::RobotControl() : drive(2, 11, 4, 5) {
	//First Joystick 
	control = new Joystick(1);		//Drive and shooting joystick
	notKaden = new Joystick(2);		//Throttle
	upperLimit = new DigitalInput(1);		//Limit for top of throw
	lowerLimit = new DigitalInput(2);		//Limit for bottom of throw
	ultra1 = new AnalogChannel(1);		//Ultrasonic sensor pointer		
	ultra2 = new AnalogChannel(2);		//US pointer 2
	multiPot = new AnalogChannel(3);		//Potentiometer pointer
	
	compressor = new Compressor(1,3);
	prodSR = new Solenoid(1);
	prodSL = new Solenoid(2);
	
	table = NetworkTable::GetTable("robo");  //Sets up network table called "robo"
	
	JagControl::config(shooter_1, 6, false);
	JagControl::config(shooter_2, 7, false);
	JagControl::config(shooter_3, 8, false);
	JagControl::config(shooter_4, 9, false);
	prodR = new Jaguar(5);
	prodL = new Jaguar(6);
	/*
	shooter_1 = new CANJaguar(6); 		//set shooter_1 to CANJaguar 6
	shooter_2 = new CANJaguar(7);		//set shooter_2 to CANJaguar 7mko0-0-----
	shooter_3 = new CANJaguar(8); 		//set shooter_3 to CANJaguar 8
	shooter_4 = new CANJaguar(9);		//set shooter_4 to CANJaguar 9
	shooter_1->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);		//set shooter_1's assigned jaguar to coast
	shooter_2->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);		//set shooter_2's assigned jaguar to coast
	shooter_3->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);		//set shooter_3's assigned jaguar to coast
	shooter_4->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);		//set shooter_4's assigned jaguar to coast
	*/
	
	
}


//provide for initialization at robot power-on
void RobotControl::RobotInit() {

}

/*float RobotControl::distance() {
	cogArea = table->GetNumber("COG_AREA", 0.0);
	return 4428.22665* pow(0.98828703, (cogArea))+100;
}*/

bool RobotControl::HotOrNot() {		//define boolean HotOrNot, that defines the target
	if(((table->GetNumber("BLOB_COUNT", 0.0)) >= 1) && (table->GetNumber("COG_X")>75) && (table->GetNumber("COG_X")<250)) {		//if the camera sees a blob and it is the right size, and it is in the center of the screen
		return true;		//set HotOrNot to true
	}
	else{
		return false;		//set HotOrNot to false
	}
}


//called only when first disabled
void RobotControl::DisabledInit() {			//define function DisabledInit
	drive.disableControl();		//disable drive control
}


//called each and every time autonomous is entered from another mode
void RobotControl::AutonomousInit() {  //define function AutonomousInit

}


//called each and every time teleop is entered from another mode
void RobotControl::TeleopInit() {		//define function TeleopInit
	upLimit = 1;		//set upLimit (upper shooting limit) to 1
	lowLimit = 1;		//set lowLimit (lower shooting limit) to 1
	upTripped = 0;		//set upTripped (limit switch variable) to 1
	align = false;		//set align (sonar alignment) to false
		
	if(drive.disabled)		//if drive is disabled
		drive.enableControl();		//enable drive
}


/*
 * Periodic functions are called iteratively at the 
 * appropriate periodic rate (aka the "slow loop"). 
 * By default, this is synced to the driver station control packets, about 50Hz
*/


void RobotControl::DisabledPeriodic() {  //define function DisabledPeriodic
	logs();
	compressor->Start();
	//cerr<<"Ultrasonic1 ="<<sonarR<<endl;		//display value of sonarR on Network Tables
	//cerr<<"Ultrasonic2 ="<<sonarL<<endl;		//display value of sonarL on Network Tables
	//cerr<<"Potentiometer ="<<multiPotValue<<endl;		//display value of multiPotValue on Network Tables

}

void RobotControl::logs() {
	sonarR = ultra1->GetVoltage();		//set sonarR to the reading of the right ultrasonic sensor
	sonarL = ultra2->GetVoltage();		//set sonarL to the reading of the left ultrasonic sensor
	shooterThrottle = (((notKaden->GetRawAxis(3))-1)/-2);
	multiPotValue = multiPot->GetVoltage();		//set value of potentiometer variable to reading on the potentiometer
	fprintf(stderr,"DT=%+2.5f ST=%+2.5f P=%+2.5f UpLimit=%s UpTripped=%s Align=%s\r",		//display a bunch of stuff
			throttle, shooterThrottle, multiPotValue,
			upLimit?"T":"F", upTripped?"T":"F",
		    align?"T":"F");
}


void RobotControl::AutonomousPeriodic() {		//define function AutonomousPeriodic
	logs();
}


void RobotControl::setShooters(double setPoint) {
	shooter_1->Set(setPoint);
	shooter_2->Set(setPoint);
	shooter_3->Set(-setPoint);
	shooter_4->Set(-setPoint);
}


void RobotControl::TeleopPeriodic() {		//define function TeleopPeriodic
	throttle = ((control->GetRawAxis(4) - 1) / -2);		//create variable throttle and set to value of throttle on joystick with bounds of 0-1
	//sonar alignment
	if (control->GetRawButton(10)) {		//if button twelve on kaden = 1
		align = true;						//align is true
	}
	if (control->GetRawButton(9)) {		//if button 11 on kaden = 1
		align = false;						//align is false
	}
	if (align) {							//if value of align is true
		if(sonarR > sonarL + .0098) {		//if sonarR is greater than sonarL + .0098
			drive.set(0,0,-.25);			//turn robot clockwise
		}
		else if(sonarL > sonarR + .0098) {	//if sonarL is greater than sonarR + .0098
			drive.set(0,0,.25);				//turn robot counterclockwise
		}
		else {								//otherwise
			align = false;					//set align to false
		}
	}
	else {									//otherwise
		//move drive train
		drive.set(control->GetRawAxis(2) * throttle, control->GetRawAxis(1) * throttle,  control->GetRawAxis(3) * throttle);
	}
	//simplified shooting code
	if (multiPotValue >= 2-((notKaden->GetRawAxis(3)-1)/-2)) {
		upLimit = 0;
		upTripped = 1;
	}
	else if (multiPotValue <= .3) {
		upLimit = 1;
		upTripped = 0;
	}
	lowLimit = (multiPotValue <= .3);
	if (upTripped) {
		setShooters(-.2);
	}
	else {
	//set value of all motors to throttle on notKaden for shooter speed control
		if (control->GetRawButton(1)) {
			setShooters((((notKaden->GetRawAxis(3))-1)/-2)*upLimit);
			// shooter_1->Set((((notKaden->GetRawAxis(3))-1)/-2)*upLimit);
			// shooter_2->Set((((notKaden->GetRawAxis(3))-1)/-2)*upLimit);
			// shooter_3->Set(-(((notKaden->GetRawAxis(3))-1)/-2)*upLimit);
			// shooter_4->Set(-(((notKaden->GetRawAxis(3))-1)/-2)*upLimit);
		}
		else if (control->GetRawButton(2)) {
			setShooters(-.1666*lowLimit);
			// shooter_1->Set(-.1666*lowLimit);
			// shooter_2->Set(-.1666*lowLimit);
			// shooter_3->Set(.1666*lowLimit);
			// shooter_4->Set(.1666*lowLimit);
		}
		else {
			setShooters(0);
			// shooter_1->Set(0);
			// shooter_2->Set(0);
			// shooter_3->Set(0);
			// shooter_4->Set(0);
		}
	}
	//cattle prod code
	if (control->GetRawButton(11)) {
		prodR->Set(throttle);
		prodL->Set(-throttle);
	}
	else if (control->GetRawButton(12)) {
		prodR->Set(-throttle);
		prodL->Set(throttle);
	}
	else {
		prodR->Set(0);
		prodL->Set(0);
	}
	
	//pnuematics code
	if(notKaden->GetRawButton(6)) {
		prodSR->Set(true);
		prodSL->Set(true);
	}
	if(notKaden->GetRawButton(7)) {
		prodSR->Set(false);
		prodSL->Set(false);
	}
	logs();
}

START_ROBOT_CLASS(RobotControl);
