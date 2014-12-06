#include <iostream>
#include <openjaus.h>
#include <openjaus/core/Base.h>

using namespace openjaus;
using namespace std;

JAUSGPSComponent::JAUSGPSComponent(),
		  JAUSComponent()
		  reportGlobalPose(),
		  reportGeomagneticProperty()
{
	//implements->push_back(discoveryService);
	//receive.addMessageCallback(&JAUSComponent::processEcho, this);
	addSelfToSystemTree();	
}

JAUSGPSComponent::~JAUSGPSComponent()
{

}
