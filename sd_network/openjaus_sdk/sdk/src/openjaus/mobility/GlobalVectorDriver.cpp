/**
\file GlobalVectorDriver.cpp

\par Copyright
Copyright (c) 2012, OpenJAUS, LLC
All rights reserved.

This file is part of the OpenJAUS Software Development Kit (SDK). This 
software is distributed under one of two licenses, the OpenJAUS SDK 
Commercial End User License Agreement or the OpenJAUS SDK Non-Commercial 
End User License Agreement. The appropriate licensing details were included 
in with your developer credentials and software download. See the LICENSE 
file included with this software for full details.
 
THIS SOFTWARE IS PROVIDED BY THE LICENSOR (OPENJAUS LCC) "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE LICENSOR BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THE SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE. THE LICENSOR DOES NOT WARRANT THAT THE LICENSED SOFTWARE WILL MEET
LICENSEE'S REQUIREMENTS OR THAT THE OPERATION OF THE LICENSED SOFTWARE
WILL BE UNINTERRUPTED OR ERROR-FREE, OR THAT ERRORS IN THE LICENSED
SOFTWARE WILL BE CORRECTED.

\ Software History
- [2011-08-23] - Added AS6057: Manipulators
- [2011-08-01] - Added AS6060: Environment Sensing
- [2011-06-16] - First Release 

*/


#include "openjaus/mobility/GlobalVectorDriver.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace mobility
{

GlobalVectorDriver::GlobalVectorDriver() : Managed(),
	globalVectorDefaultLoop(),
	globalVectorReadyLoop()
{
	// Add Service Identification Data to implements list
	name = "GlobalVectorDriver";
	
	model::Service *globalVectorDriverService = new model::Service();
	globalVectorDriverService->setName("GlobalVectorDriver");
	globalVectorDriverService->setUri("urn:jaus:jss:mobility:GlobalVectorDriver");
	globalVectorDriverService->setVersionMajor(1);
	globalVectorDriverService->setVersionMinor(0);
	this->implements->push_back(globalVectorDriverService);
	
	

	globalVectorDefaultLoop.setInterface(this);
	globalVectorDefaultLoop.setTransportInterface(this);
	accessStateMachine.addDefaultStateTransition(globalVectorDefaultLoop);
	
	globalVectorReadyLoop.setInterface(this);
	globalVectorReadyLoop.setTransportInterface(this);
	ready.addTransition(globalVectorReadyLoop);
	
    
    
	// Start of user code for Constructor:
	// End of user code
}

GlobalVectorDriver::~GlobalVectorDriver()
{
	// Start of user code for Destructor:
	// End of user code
}

mobility::ReportGlobalVector GlobalVectorDriver::getReportGlobalVector(QueryGlobalVector *queryGlobalVector)
{
	// Start of user code for action getReportGlobalVector(QueryGlobalVector *queryGlobalVector):
	mobility::ReportGlobalVector message;
	return message;
	// End of user code
}

bool GlobalVectorDriver::setGlobalVector(SetGlobalVector *setGlobalVector)
{
	// Start of user code for action setGlobalVector(SetGlobalVector *setGlobalVector):
	return false;
	// End of user code
}


bool GlobalVectorDriver::isControllingGvdClient(SetGlobalVector *setGlobalVector)
{
	// Start of user code for action isControllingGvdClient(SetGlobalVector *setGlobalVector):
	return false;
	// End of user code
}


// Start of user code for additional methods
// End of user code

} // namespace component
} // namespace openjaus

