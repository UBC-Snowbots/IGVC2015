
/**
\file SetPanTiltJointEffort.h

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
#include "openjaus/manipulator/Triggers/SetPanTiltJointEffort.h"

namespace openjaus
{
namespace manipulator
{

SetPanTiltJointEffort::SetPanTiltJointEffort() : 
	model::Message(),
	joint1Effort(),
	joint2Effort()
{
	this->id = SetPanTiltJointEffort::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&joint1Effort);
	joint1Effort.setName("Joint1Effort");
	joint1Effort.setOptional(false);
	joint1Effort.setInterpretation("Percent of maximum");
	// Nothing to init

	fields.push_back(&joint2Effort);
	joint2Effort.setName("Joint2Effort");
	joint2Effort.setOptional(false);
	joint2Effort.setInterpretation("Percent of Maximum");
	// Nothing to init

}

SetPanTiltJointEffort::SetPanTiltJointEffort(model::Message *message) :
	model::Message(message),
	joint1Effort(),
	joint2Effort()
{
	this->id = SetPanTiltJointEffort::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&joint1Effort);
	joint1Effort.setName("Joint1Effort");
	joint1Effort.setOptional(false);
	joint1Effort.setInterpretation("Percent of maximum");
	// Nothing to init

	fields.push_back(&joint2Effort);
	joint2Effort.setName("Joint2Effort");
	joint2Effort.setOptional(false);
	joint2Effort.setInterpretation("Percent of Maximum");
	// Nothing to init


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

SetPanTiltJointEffort::~SetPanTiltJointEffort()
{

}


double SetPanTiltJointEffort::getJoint1Effort(void)
{
	return this->joint1Effort.getValue();
}

void SetPanTiltJointEffort::setJoint1Effort(double value)
{
	this->joint1Effort.setValue(value);
}

double SetPanTiltJointEffort::getJoint2Effort(void)
{
	return this->joint2Effort.getValue();
}

void SetPanTiltJointEffort::setJoint2Effort(double value)
{
	this->joint2Effort.setValue(value);
}

int SetPanTiltJointEffort::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(joint1Effort);
	byteSize += dst->pack(joint2Effort);
	return byteSize;
}

int SetPanTiltJointEffort::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(joint1Effort);
	byteSize += src->unpack(joint2Effort);
	return byteSize;
}

int SetPanTiltJointEffort::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += joint1Effort.length(); // joint1Effort
	length += joint2Effort.length(); // joint2Effort
	return length;
}

std::string SetPanTiltJointEffort::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"SetPanTiltJointEffort\"";
	oss << " id=\"0x0621\" >\n";
	oss << joint1Effort.toXml(level+1); // joint1Effort
	oss << joint2Effort.toXml(level+1); // joint2Effort
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

} // namespace manipulator
} // namespace openjaus


