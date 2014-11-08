/**
\file ListManaged.cpp

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


#include "openjaus/mobility/ListManaged.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace mobility
{

ListManaged::ListManaged() : Managed(),
	listManagerDefaultLoop(),
	listManagerControlledLoop()
{
	// Add Service Identification Data to implements list
	name = "ListManaged";
	
	model::Service *listManagerService = new model::Service();
	listManagerService->setName("ListManager");
	listManagerService->setUri("urn:jaus:jss:mobility:ListManager");
	listManagerService->setVersionMajor(1);
	listManagerService->setVersionMinor(0);
	this->implements->push_back(listManagerService);
	
	

	listManagerDefaultLoop.setInterface(this);
	listManagerDefaultLoop.setTransportInterface(this);
	accessStateMachine.addDefaultStateTransition(listManagerDefaultLoop);
	
	listManagerControlledLoop.setInterface(this);
	listManagerControlledLoop.setTransportInterface(this);
	controlled.addTransition(listManagerControlledLoop);
	
    
    
	// Start of user code for Constructor:
	// End of user code
}

ListManaged::~ListManaged()
{
	// Start of user code for Destructor:
	// End of user code
}

mobility::ReportElement ListManaged::getReportElement(QueryElement *queryElement)
{
	// Start of user code for action getReportElement(QueryElement *queryElement):
	mobility::ReportElement message;
	return message;
	// End of user code
}

mobility::ReportElementList ListManaged::getReportElementList(QueryElementList *queryElementList)
{
	// Start of user code for action getReportElementList(QueryElementList *queryElementList):
	mobility::ReportElementList message;
	return message;
	// End of user code
}

mobility::ReportElementCount ListManaged::getReportElementCount(QueryElementCount *queryElementCount)
{
	// Start of user code for action getReportElementCount(QueryElementCount *queryElementCount):
	mobility::ReportElementCount message;
	return message;
	// End of user code
}

mobility::ConfirmElementRequest ListManaged::getConfirmElementRequest(SetElement *setElement)
{
	// Start of user code for action getConfirmElementRequest(SetElement *setElement):
	mobility::ConfirmElementRequest message;
	return message;
	// End of user code
}

mobility::ConfirmElementRequest ListManaged::getConfirmElementRequest(DeleteElement *deleteElement)
{
	// Start of user code for action getConfirmElementRequest(DeleteElement *deleteElement):
	mobility::ConfirmElementRequest message;
	return message;
	// End of user code
}

mobility::RejectElementRequest ListManaged::getRejectElementRequest(SetElement *setElement)
{
	// Start of user code for action getRejectElementRequest(SetElement *setElement):
	mobility::RejectElementRequest message;
	return message;
	// End of user code
}

mobility::RejectElementRequest ListManaged::getRejectElementRequest(DeleteElement *deleteElement)
{
	// Start of user code for action getRejectElementRequest(DeleteElement *deleteElement):
	mobility::RejectElementRequest message;
	return message;
	// End of user code
}

bool ListManaged::setElement(SetElement *setElement)
{
	// Start of user code for action setElement(SetElement *setElement):
	return false;
	// End of user code
}

bool ListManaged::deleteElement(DeleteElement *deleteElement)
{
	// Start of user code for action deleteElement(DeleteElement *deleteElement):
	return false;
	// End of user code
}


bool ListManaged::elementExists(QueryElement *queryElement)
{
	// Start of user code for action elementExists(QueryElement *queryElement):
	return false;
	// End of user code
}

bool ListManaged::elementExists(SetElement *setElement)
{
	// Start of user code for action elementExists(SetElement *setElement):
	return false;
	// End of user code
}

bool ListManaged::elementExists(DeleteElement *deleteElement)
{
	// Start of user code for action elementExists(DeleteElement *deleteElement):
	return false;
	// End of user code
}


bool ListManaged::isControllingListClient(SetElement *setElement)
{
	// Start of user code for action isControllingListClient(SetElement *setElement):
	return false;
	// End of user code
}

bool ListManaged::isControllingListClient(DeleteElement *deleteElement)
{
	// Start of user code for action isControllingListClient(DeleteElement *deleteElement):
	return false;
	// End of user code
}


bool ListManaged::isValidElementRequest(SetElement *setElement)
{
	// Start of user code for action isValidElementRequest(SetElement *setElement):
	return false;
	// End of user code
}


bool ListManaged::isElementSupported(SetElement *setElement)
{
	// Start of user code for action isElementSupported(SetElement *setElement):
	return false;
	// End of user code
}


// Start of user code for additional methods
// End of user code

} // namespace component
} // namespace openjaus

