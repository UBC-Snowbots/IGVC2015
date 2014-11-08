
/**
\file SetSkidSteerEffort.h

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
#include "openjaus/ugv/Triggers/SetSkidSteerEffort.h"

namespace openjaus
{
namespace ugv
{

SetSkidSteerEffort::SetSkidSteerEffort() : 
	model::Message(),
	leftTrackPropulsiveEffort(),
	rightTrackPropulsiveEffort()
{
	this->id = SetSkidSteerEffort::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&leftTrackPropulsiveEffort);
	leftTrackPropulsiveEffort.setName("LeftTrackPropulsiveEffort");
	leftTrackPropulsiveEffort.setOptional(false);
	// Nothing to init

	fields.push_back(&rightTrackPropulsiveEffort);
	rightTrackPropulsiveEffort.setName("RightTrackPropulsiveEffort");
	rightTrackPropulsiveEffort.setOptional(false);
	// Nothing to init

}

SetSkidSteerEffort::SetSkidSteerEffort(model::Message *message) :
	model::Message(message),
	leftTrackPropulsiveEffort(),
	rightTrackPropulsiveEffort()
{
	this->id = SetSkidSteerEffort::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&leftTrackPropulsiveEffort);
	leftTrackPropulsiveEffort.setName("LeftTrackPropulsiveEffort");
	leftTrackPropulsiveEffort.setOptional(false);
	// Nothing to init

	fields.push_back(&rightTrackPropulsiveEffort);
	rightTrackPropulsiveEffort.setName("RightTrackPropulsiveEffort");
	rightTrackPropulsiveEffort.setOptional(false);
	// Nothing to init


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

SetSkidSteerEffort::~SetSkidSteerEffort()
{

}


double SetSkidSteerEffort::getLeftTrackPropulsiveEffort(void)
{
	return this->leftTrackPropulsiveEffort.getValue();
}

void SetSkidSteerEffort::setLeftTrackPropulsiveEffort(double value)
{
	this->leftTrackPropulsiveEffort.setValue(value);
}

double SetSkidSteerEffort::getRightTrackPropulsiveEffort(void)
{
	return this->rightTrackPropulsiveEffort.getValue();
}

void SetSkidSteerEffort::setRightTrackPropulsiveEffort(double value)
{
	this->rightTrackPropulsiveEffort.setValue(value);
}

int SetSkidSteerEffort::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(leftTrackPropulsiveEffort);
	byteSize += dst->pack(rightTrackPropulsiveEffort);
	return byteSize;
}

int SetSkidSteerEffort::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(leftTrackPropulsiveEffort);
	byteSize += src->unpack(rightTrackPropulsiveEffort);
	return byteSize;
}

int SetSkidSteerEffort::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += leftTrackPropulsiveEffort.length(); // leftTrackPropulsiveEffort
	length += rightTrackPropulsiveEffort.length(); // rightTrackPropulsiveEffort
	return length;
}

std::string SetSkidSteerEffort::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"SetSkidSteerEffort\"";
	oss << " id=\"0x0501\" >\n";
	oss << leftTrackPropulsiveEffort.toXml(level+1); // leftTrackPropulsiveEffort
	oss << rightTrackPropulsiveEffort.toXml(level+1); // rightTrackPropulsiveEffort
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

} // namespace ugv
} // namespace openjaus


