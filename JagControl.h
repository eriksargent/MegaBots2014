#include "WPILib.h"
#include <CANJaguar.h>

class JagControl {
public:
	static void config(CANJaguar *&jag, int id, bool brake=true, 
			bool encoder=false, int encRevs=360, 
			bool PID=false, double p=0, double i=0, double d=0) {

		if(encoder) {
			if(PID)
				jag = new CANJaguar(id, CANJaguar::kSpeed);
			else
				jag = new CANJaguar(id);
			jag->ConfigEncoderCodesPerRev(encRevs);
			jag->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
		}
		else
			jag = new CANJaguar(id);

		if(PID)
			jag->SetPID(p, i, d);
		
		jag->ConfigNeutralMode(brake ? CANJaguar::kNeutralMode_Brake : CANJaguar::kNeutralMode_Coast);
	}
};
