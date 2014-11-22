#include <iostream>
#include <openjaus.h>
#include <openjaus/core/Base.h>

using namespace openjaus;

class JAUSComponent: public openjaus::core::Base
{
/*private:
	openjaus::system::Timer *pingTimer;
	short pingId;*/
public:
	JAUSComponent();

	//bool processEcho(openjaus::core::ReportHeartbeatPulse &echo);
	void run();
	//void discover(openjaus::system::Timer *timer);
};
