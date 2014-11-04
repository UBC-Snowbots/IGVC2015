
/**
\file SetPanTiltMotionProfile.h

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
#include "openjaus/manipulator/Triggers/SetPanTiltMotionProfile.h"

namespace openjaus
{
namespace manipulator
{

SetPanTiltMotionProfile::SetPanTiltMotionProfile() : 
	model::Message(),
	joint1MaxSpeed_rps(),
	joint1MaxAccelerationRate_rps2(),
	joint1MaxDecelerationRate_rps2(),
	joint2MaxSpeed_rps(),
	joint2MaxAccelerationRate_rps2(),
	joint2MaxDecelerationRate_rps2()
{
	this->id = SetPanTiltMotionProfile::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&joint1MaxSpeed_rps);
	joint1MaxSpeed_rps.setName("Joint1MaxSpeed");
	joint1MaxSpeed_rps.setOptional(false);
	// Nothing to init

	fields.push_back(&joint1MaxAccelerationRate_rps2);
	joint1MaxAccelerationRate_rps2.setName("Joint1MaxAccelerationRate");
	joint1MaxAccelerationRate_rps2.setOptional(false);
	// Nothing to init

	fields.push_back(&joint1MaxDecelerationRate_rps2);
	joint1MaxDecelerationRate_rps2.setName("Joint1MaxDecelerationRate");
	joint1MaxDecelerationRate_rps2.setOptional(false);
	// Nothing to init

	fields.push_back(&joint2MaxSpeed_rps);
	joint2MaxSpeed_rps.setName("Joint2MaxSpeed");
	joint2MaxSpeed_rps.setOptional(false);
	// Nothing to init

	fields.push_back(&joint2MaxAccelerationRate_rps2);
	joint2MaxAccelerationRate_rps2.setName("Joint2MaxAccelerationRate");
	joint2MaxAccelerationRate_rps2.setOptional(false);
	// Nothing to init

	fields.push_back(&joint2MaxDecelerationRate_rps2);
	joint2MaxDecelerationRate_rps2.setName("Joint2MaxDecelerationRate");
	joint2MaxDecelerationRate_rps2.setOptional(false);
	// Nothing to init

}

SetPanTiltMotionProfile::SetPanTiltMotionProfile(model::Message *message) :
	model::Message(message),
	joint1MaxSpeed_rps(),
	joint1MaxAccelerationRate_rps2(),
	joint1MaxDecelerationRate_rps2(),
	joint2MaxSpeed_rps(),
	joint2MaxAccelerationRate_rps2(),
	joint2MaxDecelerationRate_rps2()
{
	this->id = SetPanTiltMotionProfile::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&joint1MaxSpeed_rps);
	joint1MaxSpeed_rps.setName("Joint1MaxSpeed");
	joint1MaxSpeed_rps.setOptional(false);
	// Nothing to init

	fields.push_back(&joint1MaxAccelerationRate_rps2);
	joint1MaxAccelerationRate_rps2.setName("Joint1MaxAccelerationRate");
	joint1MaxAccelerationRate_rps2.setOptional(false);
	// Nothing to init

	fields.push_back(&joint1MaxDecelerationRate_rps2);
	joint1MaxDecelerationRate_rps2.setName("Joint1MaxDecelerationRate");
	joint1MaxDecelerationRate_rps2.setOptional(false);
	// Nothing to init

	fields.push_back(&joint2MaxSpeed_rps);
	joint2MaxSpeed_rps.setName("Joint2MaxSpeed");
	joint2MaxSpeed_rps.setOptional(false);
	// Nothing to init

	fields.push_back(&joint2MaxAccelerationRate_rps2);
	joint2MaxAccelerationRate_rps2.setName("Joint2MaxAccelerationRate");
	joint2MaxAccelerationRate_rps2.setOptional(false);
	// Nothing to init

	fields.push_back(&joint2MaxDecelerationRate_rps2);
	joint2MaxDecelerationRate_rps2.setName("Joint2MaxDecelerationRate");
	joint2MaxDecelerationRate_rps2.setOptional(false);
	// Nothing to init


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

SetPanTiltMotionProfile::~SetPanTiltMotionProfile()
{

}


double SetPanTiltMotionProfile::getJoint1MaxSpeed_rps(void)
{
	return this->joint1MaxSpeed_rps.getValue();
}

void SetPanTiltMotionProfile::setJoint1MaxSpeed_rps(double value)
{
	this->joint1MaxSpeed_rps.setValue(value);
}

double SetPanTiltMotionProfile::getJoint1MaxAccelerationRate_rps2(void)
{
	return this->joint1MaxAccelerationRate_rps2.getValue();
}

void SetPanTiltMotionProfile::setJoint1MaxAccelerationRate_rps2(double value)
{
	this->joint1MaxAccelerationRate_rps2.setValue(value);
}

double SetPanTiltMotionProfile::getJoint1MaxDecelerationRate_rps2(void)
{
	return this->joint1MaxDecelerationRate_rps2.getValue();
}

void SetPanTiltMotionProfile::setJoint1MaxDecelerationRate_rps2(double value)
{
	this->joint1MaxDecelerationRate_rps2.setValue(value);
}

double SetPanTiltMotionProfile::getJoint2MaxSpeed_rps(void)
{
	return this->joint2MaxSpeed_rps.getValue();
}

void SetPanTiltMotionProfile::setJoint2MaxSpeed_rps(double value)
{
	this->joint2MaxSpeed_rps.setValue(value);
}

double SetPanTiltMotionProfile::getJoint2MaxAccelerationRate_rps2(void)
{
	return this->joint2MaxAccelerationRate_rps2.getValue();
}

void SetPanTiltMotionProfile::setJoint2MaxAccelerationRate_rps2(double value)
{
	this->joint2MaxAccelerationRate_rps2.setValue(value);
}

double SetPanTiltMotionProfile::getJoint2MaxDecelerationRate_rps2(void)
{
	return this->joint2MaxDecelerationRate_rps2.getValue();
}

void SetPanTiltMotionProfile::setJoint2MaxDecelerationRate_rps2(double value)
{
	this->joint2MaxDecelerationRate_rps2.setValue(value);
}

int SetPanTiltMotionProfile::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(joint1MaxSpeed_rps);
	byteSize += dst->pack(joint1MaxAccelerationRate_rps2);
	byteSize += dst->pack(joint1MaxDecelerationRate_rps2);
	byteSize += dst->pack(joint2MaxSpeed_rps);
	byteSize += dst->pack(joint2MaxAccelerationRate_rps2);
	byteSize += dst->pack(joint2MaxDecelerationRate_rps2);
	return byteSize;
}

int SetPanTiltMotionProfile::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(joint1MaxSpeed_rps);
	byteSize += src->unpack(joint1MaxAccelerationRate_rps2);
	byteSize += src->unpack(joint1MaxDecelerationRate_rps2);
	byteSize += src->unpack(joint2MaxSpeed_rps);
	byteSize += src->unpack(joint2MaxAccelerationRate_rps2);
	byteSize += src->unpack(joint2MaxDecelerationRate_rps2);
	return byteSize;
}

int SetPanTiltMotionProfile::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += joint1MaxSpeed_rps.length(); // joint1MaxSpeed_rps
	length += joint1MaxAccelerationRate_rps2.length(); // joint1MaxAccelerationRate_rps2
	length += joint1MaxDecelerationRate_rps2.length(); // joint1MaxDecelerationRate_rps2
	length += joint2MaxSpeed_rps.length(); // joint2MaxSpeed_rps
	length += joint2MaxAccelerationRate_rps2.length(); // joint2MaxAccelerationRate_rps2
	length += joint2MaxDecelerationRate_rps2.length(); // joint2MaxDecelerationRate_rps2
	return length;
}

std::string SetPanTiltMotionProfile::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"SetPanTiltMotionProfile\"";
	oss << " id=\"0x0627\" >\n";
	oss << joint1MaxSpeed_rps.toXml(level+1); // joint1MaxSpeed_rps
	oss << joint1MaxAccelerationRate_rps2.toXml(level+1); // joint1MaxAccelerationRate_rps2
	oss << joint1MaxDecelerationRate_rps2.toXml(level+1); // joint1MaxDecelerationRate_rps2
	oss << joint2MaxSpeed_rps.toXml(level+1); // joint2MaxSpeed_rps
	oss << joint2MaxAccelerationRate_rps2.toXml(level+1); // joint2MaxAccelerationRate_rps2
	oss << joint2MaxDecelerationRate_rps2.toXml(level+1); // joint2MaxDecelerationRate_rps2
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

} // namespace manipulator
} // namespace openjaus


