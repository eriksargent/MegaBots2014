#include "WPILib.h"
#include <CANJaguar.h>
#include <math.h>

#define pi 3.14159

class MecanumDrive{
private:
	//Four CANJaguars for the four motors on the drive train
	CANJaguar *f_right; 
	CANJaguar *f_left;
	CANJaguar *r_right;
	CANJaguar *r_left;
	double limit(double d);
	void setDriveA(double dir, double pwr, double t);
	void setMotors(double fl, double fr, double rl, double rr);

	public:
	MecanumDrive(int fr, int fl, int rr, int rl);
	void set(double x, double y, double turn);
	void enableControl();
	void disableControl();
	bool disabled;
};
