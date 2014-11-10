#include <openjaus.h>
#include <openjaus/mobility.h>

using namespace std;

class JAUSComponent: public openjaus::core::Base
{
public:
	JAUSComponent();
	void run();
	void stop();
};
