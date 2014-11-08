/**
\file PrimitivePanTilt.cpp

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


#include "openjaus/manipulator/PrimitivePanTilt.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace manipulator
{

PrimitivePanTilt::PrimitivePanTilt() : Managed(),
	primitivePanTiltDefaultLoop(),
	primitivePanTiltControlledLoop()
{
	// Add Service Identification Data to implements list
	name = "PrimitivePanTilt";
	
	model::Service *primitivePanTiltService = new model::Service();
	primitivePanTiltService->setName("PrimitivePanTilt");
	primitivePanTiltService->setUri("urn:jaus:jss:manipulator:PrimitivePanTilt");
	primitivePanTiltService->setVersionMajor(1);
	primitivePanTiltService->setVersionMinor(0);
	this->implements->push_back(primitivePanTiltService);
	
	

	primitivePanTiltDefaultLoop.setInterface(this);
	primitivePanTiltDefaultLoop.setTransportInterface(this);
	accessStateMachine.addDefaultStateTransition(primitivePanTiltDefaultLoop);
	
	primitivePanTiltControlledLoop.setInterface(this);
	primitivePanTiltControlledLoop.setTransportInterface(this);
	controlled.addTransition(primitivePanTiltControlledLoop);
	
    
    
	// Start of user code for Constructor:
	// End of user code
}

PrimitivePanTilt::~PrimitivePanTilt()
{
	// Start of user code for Destructor:
	// End of user code
}

bool PrimitivePanTilt::setPanTiltJointEffort(SetPanTiltJointEffort *setPanTiltJointEffort)
{
	// Start of user code for action setPanTiltJointEffort(SetPanTiltJointEffort *setPanTiltJointEffort):
	return false;
	// End of user code
}

manipulator::ReportPanTiltSpecifications PrimitivePanTilt::getReportPanTiltSpecifications(QueryPanTiltSpecifications *queryPanTiltSpecifications)
{
	// Start of user code for action getReportPanTiltSpecifications(QueryPanTiltSpecifications *queryPanTiltSpecifications):
	manipulator::ReportPanTiltSpecifications message;
	return message;
	// End of user code
}

manipulator::ReportPanTiltJointEffort PrimitivePanTilt::getReportPanTiltJointEffort(QueryPanTiltJointEffort *queryPanTiltJointEffort)
{
	// Start of user code for action getReportPanTiltJointEffort(QueryPanTiltJointEffort *queryPanTiltJointEffort):
	manipulator::ReportPanTiltJointEffort message;
	return message;
	// End of user code
}


bool PrimitivePanTilt::isControllingPrimitivePanTiltClient(SetPanTiltJointEffort *setPanTiltJointEffort)
{
	// Start of user code for action isControllingPrimitivePanTiltClient(SetPanTiltJointEffort *setPanTiltJointEffort):
	return false;
	// End of user code
}


// Start of user code for additional methods
// End of user code

} // namespace component
} // namespace openjaus

