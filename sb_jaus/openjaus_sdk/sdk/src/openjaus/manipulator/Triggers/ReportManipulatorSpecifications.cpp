
/**
\file ReportManipulatorSpecifications.h

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

#include <openjaus.h>
#include "openjaus/manipulator/Triggers/ReportManipulatorSpecifications.h"

namespace openjaus
{
namespace manipulator
{

ReportManipulatorSpecifications::ReportManipulatorSpecifications() : 
	model::Message(),
	manipulatorCoordinateSystemRec(),
	firstJointParameters(),
	jointSpecificationList()
{
	this->id = ReportManipulatorSpecifications::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&manipulatorCoordinateSystemRec);
	manipulatorCoordinateSystemRec.setName("ManipulatorCoordinateSystemRec");
	manipulatorCoordinateSystemRec.setOptional(true);
	//Nothing to Init

	fields.push_back(&firstJointParameters);
	firstJointParameters.setName("FirstJointParameters");
	firstJointParameters.setOptional(false);
	// Nothing to Init

	fields.push_back(&jointSpecificationList);
	jointSpecificationList.setName("JointSpecificationList");
	jointSpecificationList.setOptional(false);
	// Nothing to Init

}

ReportManipulatorSpecifications::ReportManipulatorSpecifications(model::Message *message) :
	model::Message(message),
	manipulatorCoordinateSystemRec(),
	firstJointParameters(),
	jointSpecificationList()
{
	this->id = ReportManipulatorSpecifications::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&manipulatorCoordinateSystemRec);
	manipulatorCoordinateSystemRec.setName("ManipulatorCoordinateSystemRec");
	manipulatorCoordinateSystemRec.setOptional(true);
	//Nothing to Init

	fields.push_back(&firstJointParameters);
	firstJointParameters.setName("FirstJointParameters");
	firstJointParameters.setOptional(false);
	// Nothing to Init

	fields.push_back(&jointSpecificationList);
	jointSpecificationList.setName("JointSpecificationList");
	jointSpecificationList.setOptional(false);
	// Nothing to Init


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

ReportManipulatorSpecifications::~ReportManipulatorSpecifications()
{

}


ManipulatorCoordinateSystemRecord& ReportManipulatorSpecifications::getManipulatorCoordinateSystemRec(void)
{
	return this->manipulatorCoordinateSystemRec;
}

FirstJointParametersVariant& ReportManipulatorSpecifications::getFirstJointParameters(void)
{
	return this->firstJointParameters;
}

JointSpecificationList& ReportManipulatorSpecifications::getJointSpecificationList(void)
{
	return this->jointSpecificationList;
}

int ReportManipulatorSpecifications::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(this->presenceVector);
	if(this->isManipulatorCoordinateSystemRecEnabled())
	{
		byteSize += dst->pack(manipulatorCoordinateSystemRec);
	}
	byteSize += dst->pack(firstJointParameters);
	byteSize += dst->pack(jointSpecificationList);
	return byteSize;
}

int ReportManipulatorSpecifications::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(this->presenceVector);
	if(this->isManipulatorCoordinateSystemRecEnabled())
	{
		byteSize += src->unpack(manipulatorCoordinateSystemRec);
	}
	byteSize += src->unpack(firstJointParameters);
	byteSize += src->unpack(jointSpecificationList);
	return byteSize;
}

int ReportManipulatorSpecifications::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += sizeof(uint8_t); // PresenceVector
	if(this->isManipulatorCoordinateSystemRecEnabled())
	{
		length += manipulatorCoordinateSystemRec.length(); // manipulatorCoordinateSystemRec
	}
	length += firstJointParameters.length(); // firstJointParameters
	length += jointSpecificationList.length(); // jointSpecificationList
	return length;
}

std::string ReportManipulatorSpecifications::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"ReportManipulatorSpecifications\"";
	oss << " id=\"0x4600\" >\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint8_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isManipulatorCoordinateSystemRecEnabled value=\"" << std::boolalpha << this->isManipulatorCoordinateSystemRecEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	if(this->isManipulatorCoordinateSystemRecEnabled())
	{
		oss << manipulatorCoordinateSystemRec.toXml(level+1); // manipulatorCoordinateSystemRec
	}
	oss << firstJointParameters.toXml(level+1); // firstJointParameters
	oss << jointSpecificationList.toXml(level+1); // jointSpecificationList
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

void ReportManipulatorSpecifications::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t ReportManipulatorSpecifications::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool ReportManipulatorSpecifications::isManipulatorCoordinateSystemRecEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportManipulatorSpecifications::MANIPULATORCOORDINATESYSTEMREC));
}

void ReportManipulatorSpecifications::enableManipulatorCoordinateSystemRec(void)
{
	this->presenceVector |= 0x01 << ReportManipulatorSpecifications::MANIPULATORCOORDINATESYSTEMREC;
}

void ReportManipulatorSpecifications::disableManipulatorCoordinateSystemRec(void)
{
	this->presenceVector &= ~(0x01 << ReportManipulatorSpecifications::MANIPULATORCOORDINATESYSTEMREC);
}

} // namespace manipulator
} // namespace openjaus


