#include <iostream>
#include <openjaus.h>
#include <openjaus/core/Base.h>

using namespace openjaus;

class JAUSComponent: public openjaus::core::Base
{
public:
	JAUSComponent();
	~JAUSComponent();
	typedef openjaus::core::Base super;
	void run();
};
