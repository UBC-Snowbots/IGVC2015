#include <iostream>
#include <openjaus.h>
#include <openjaus/core/Base.h>
#include <openjaus/mobility/GlobalPoseSensor.h>

#include "JAUSComponent.hpp"

using namespace openjaus;

class JAUSGPSComponent: public JAUSComponent,
			public openjaus::mobility::GlobalPoseSensor
{
private:
	mobility::ReportGlobalPose reportGlobalPose;
	mobility::ReportGeomagneticProperty reportGeomagneticProperty;
public:
	JAUSGPSComponent();
	~JAUSGPSComponent();

	
};
