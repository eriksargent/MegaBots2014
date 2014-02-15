#include "RobotControl.h"

RobotControl::RobotControl() : drive(2, 3, 4, 5) {
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
	JagControl::config(prodR, 12, false);
	JagControl::config(prodL, 13, false);
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
	start = time(0);
}


//called each and every time teleop is entered from another mode
void RobotControl::TeleopInit() {		//define function TeleopInit
	upLimit = 1;		//set upLimit (upper shooting limit) to 1
	lowLimit = 1;		//set lowLimit (lower shooting limit) to 1
	upTripped = 0;		//set upTripped (limit switch variable) to 1
	align = false;		//set align (sonar alignment) to false
	flipDrive = false;
	armsUp = true;
		
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
	shooterThrottle = ((notKaden->GetRawAxis(3) - 1)/-2);
	multiPotValue = multiPot->GetVoltage();		//set value of potentiometer variable to reading on the potentiometer
	fprintf(stderr,"DT=%+2.5f ST=%+2.5f P=%+2.5f UpLimit=%s UpTripped=%s Align=%s\r",		//display a bunch of stuff
			throttle, shooterThrottle, multiPotValue,
			upLimit?"T":"F", upTripped?"T":"F",
		    align?"T":"F");
}


void RobotControl::AutonomousPeriodic() {		//define function AutonomousPeriodic
	logs();
	autoTime = difftime(time(0), start);
	if (autoTime < .75) {
	drive.set(0 ,.5 ,0);
	}
	else if (autoTime > .75) {
		drive.set(0,0,0);
		prodSR->Set(true);
		prodSL->Set(true);
		prodR->Set(1);
		prodL->Set(-1);
		if (HotOrNot()) {
			setShooters( 1 * upLimit);
		}
		else if (multiPotValue >= 1.1) {
			upLimit = 0;
			upTripped = 1;
		}
		//lower limit
		else if (multiPotValue <= .4) {
			upLimit = 1;
			upTripped = 0;
		}

		lowLimit = (multiPotValue <= .4);
		if (upTripped){
			setShooters(-.2);
		}
	}
	else if (autoTime > 1.25) {
		prodR->Set(0);
		prodL->Set(0);
	}
	
}


void RobotControl::setShooters(double setPoint) {
	shooter_1->Set(setPoint);
	shooter_2->Set(setPoint);
	shooter_3->Set(-setPoint);
	shooter_4->Set(-setPoint);
}


void RobotControl::TeleopPeriodic() {		//define function TeleopPeriodic
	if(control->GetRawButton(2) && !flipPressed) {
		flipPressed = true;
		flipDrive = !flipDrive;
	}
	else if(flipPressed && !control->GetRawButton(2))
		flipPressed = false;
		
	//Drive variables
	
	//Throttle is used to adjust the driving speed
	throttle = ((control->GetRawAxis(4) - 1) / -2);
	//Magnitude to move in x-direction
	double xMove = control->GetRawAxis(2) * throttle * (flipDrive ? -1 : 1);
	//Magnitude to move in y-direction
	double yMove = control->GetRawAxis(1) * throttle * (flipDrive ? -1 : 1);

	/*
	* Drive train control
	* Supports sonar alignment with the wall
	* If button 10 is being pressed, disable joystick turning, 
	* and control turn from ultrasonic sensor.
	* Otherwise, drive normally from the joystick
	*/
	
	if (control->GetRawButton(10)) {
		//Right is farther from the wall. Turn left
		if(sonarR > sonarL + 0.0098)
			drive.set(xMove, yMove, -0.25);

		//Left is farther from the wall. Turn right
		else if(sonarL > sonarR + 0.0098)
			drive.set(xMove, yMove, 0.25);

		//Otherwize, drive without turn
		else
			drive.set(xMove, yMove, 0);
	}
	//move drive train normally
	else
		drive.set(xMove, yMove,  control->GetRawAxis(3) * throttle);


	/*
	* Throwing control
	* Control position of the throwing arms with a potentiometer
	* When throwing, stop the arms at a specific point based on the speed
	* and lower the arms slowly until they reach the bottom
	* The speed of the throwers is changed with the notKaden joystick throttle
	*/
	
	//Drop
	if(notKaden->GetRawButton(4)) {
		if(multiPotValue < 2.25)
			setShooters(.25);
		else
			setShooters(0);
	}
	//Upper limit
	else if (multiPotValue >= 2.1 - ((notKaden->GetRawAxis(3) - 1)/-2)) {
		upLimit = 0;
		upTripped = 1;
	}
	//lower limit
	else if (multiPotValue <= .4) {
		upLimit = 1;
		upTripped = 0;
	}

	lowLimit = (multiPotValue <= .4);
	//Lower the arms back down
	if(!notKaden->GetRawButton(4)) {
		if (upTripped)
			setShooters(-.2);
		else {
			//Throw
			if (notKaden->GetRawButton(1))
				setShooters(((notKaden->GetRawAxis(3) - 1)/-2) * upLimit * armsUp);
	
			//retract
			else if (notKaden->GetRawButton(5))
				setShooters(-0.2);
	
			//stop
			else
				setShooters(0);
		}
	}



	/*
	* Collector Control
	* Drives the collectors with the speed of the main throttle
	* Buttons 11 and 12 select which direction to turn them
	* The two arms spin in opposite directions to pull the ball in, or push it out
	*/
	
	prodR->Set(-notKaden->GetRawAxis(2));
	prodL->Set(notKaden->GetRawAxis(2));
	/*
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
	*/

	/*
	* Pneumatics Control
	* Button 6 pushes the cylindars out
	* Button 7 pulls them back in
	*/

	if(notKaden->GetRawButton(6) || notKaden->GetRawButton(1) || notKaden->GetRawButton(4)) {
		armsUp = true;
		prodSR->Set(true);
		prodSL->Set(true);
	}
	if(notKaden->GetRawButton(7)) {
		armsUp = false;
		prodSR->Set(false);
		prodSL->Set(false);
	}


	logs();
}

START_ROBOT_CLASS(RobotControl);
