#include "MecanumDrive.h"


//Constructor. Takes the ID's of the Jaguars, and 2,3,4,5 is the default
MecanumDrive::MecanumDrive(int fr = 2, int fl = 3, int rr = 4, int rl = 5) {
	/*f_right = new CANJaguar(fr, CANJaguar::kSpeed);
	f_left = new CANJaguar(fl, CANJaguar::kSpeed);
	r_right = new CANJaguar(rr, CANJaguar::kSpeed);
	r_left = new CANJaguar(rl, CANJaguar::kSpeed);*/
	
	f_right = new CANJaguar(fr);
	f_left = new CANJaguar(fl);
	r_right = new CANJaguar(rr);
	r_left = new CANJaguar(rl);

	f_right->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake); //set all Jaguars to break instead of coast
	f_left->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
	r_right->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
	r_left->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);

	/*f_right->ConfigEncoderCodesPerRev(360);
	f_right->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);

	f_left->ConfigEncoderCodesPerRev(360);
	f_left->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);

	r_right->ConfigEncoderCodesPerRev(360);
	r_right->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);

	r_left->ConfigEncoderCodesPerRev(360);
	r_left->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);

	f_right->SetPID(.02, 0.004, 0.0);
	f_left->SetPID(.02, 0.004, 0.0);
	r_right->SetPID(.02, 0.004, 0.0);
	r_left->SetPID(.02, 0.004, 0.0);*/

	disabled = true;
}


//enable Jaguar control
void MecanumDrive::enableControl() {
	f_right->EnableControl();
	f_left->EnableControl();
	r_right->EnableControl();
	r_left->EnableControl();
	disabled = false;
}


//disable Jaguar control
void MecanumDrive::disableControl() {
	f_right->DisableControl();
	f_left->DisableControl();
	r_right->DisableControl();
	r_left->DisableControl();
	disabled = true;
}


//Raw joystick input. Calculates direction, power, turn (movement in polar coordinates)
void MecanumDrive::set(double x, double y, double turn) {
	double dir;
	double pwr;
	pwr = sqrt(x*x+y*y);
	dir = atan2(y, -x)/pi*180;
	setDriveA(dir, pwr, turn);
}


//Translates movement in polar coordinates into individual motor power
void MecanumDrive::setDriveA(double dir, double pwr, double t) {
	double fr,fl,rr,rl;
	pwr = limit(pwr);
	t = limit(t)/2;
	fr = cos((dir+45)/180.0*3.14159);
	fl = cos((dir-45)/180.0*3.14159);
	rr = fl*pwr-t;
	rl = fr*pwr+t;
	fl = fl*pwr+t;
	fr = fr*pwr-t;

	setMotors(fl, fr, rl, rr);
}


//limits value to between -1 and 1
double MecanumDrive::limit(double d) {
//                d = d > 1? 1:d;
//                d = d < -1? -1:d;
	d = (d > -.03 && d < .03) ? 0 : d;
	return d;
}


//Moves the motors
void MecanumDrive::setMotors(double fl, double fr, double rl, double rr) {
	/*f_right->Set(-fr*500);
	f_left->Set(fl*500);
	r_right->Set(-rr*500);
	r_left->Set(rl*500);*/
	f_right->Set(-fr);
	f_left->Set(fl);
	r_right->Set(-rr);
	r_left->Set(rl);
}
