
/**
\file ReportCommandedEndEffectorVelocityState.h

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
#include "openjaus/manipulator/Triggers/ReportCommandedEndEffectorVelocityState.h"

namespace openjaus
{
namespace manipulator
{

ReportCommandedEndEffectorVelocityState::ReportCommandedEndEffectorVelocityState() : 
	model::Message(),
	angularVelocityComponentX_rps(),
	angularVelocityComponentY_rps(),
	angularVelocityComponentZ_rps(),
	linearVelocityComponentX_mps(),
	linearVelocityComponentY_mps(),
	linearVelocityComponentZ_mps()
{
	this->id = ReportCommandedEndEffectorVelocityState::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&angularVelocityComponentX_rps);
	angularVelocityComponentX_rps.setName("AngularVelocityComponentX");
	angularVelocityComponentX_rps.setOptional(false);
	// Nothing to init

	fields.push_back(&angularVelocityComponentY_rps);
	angularVelocityComponentY_rps.setName("AngularVelocityComponentY");
	angularVelocityComponentY_rps.setOptional(false);
	// Nothing to init

	fields.push_back(&angularVelocityComponentZ_rps);
	angularVelocityComponentZ_rps.setName("AngularVelocityComponentZ");
	angularVelocityComponentZ_rps.setOptional(false);
	// Nothing to init

	fields.push_back(&linearVelocityComponentX_mps);
	linearVelocityComponentX_mps.setName("LinearVelocityComponentX");
	linearVelocityComponentX_mps.setOptional(false);
	// Nothing to init

	fields.push_back(&linearVelocityComponentY_mps);
	linearVelocityComponentY_mps.setName("LinearVelocityComponentY");
	linearVelocityComponentY_mps.setOptional(false);
	// Nothing to init

	fields.push_back(&linearVelocityComponentZ_mps);
	linearVelocityComponentZ_mps.setName("LinearVelocityComponentZ");
	linearVelocityComponentZ_mps.setOptional(false);
	// Nothing to init

}

ReportCommandedEndEffectorVelocityState::ReportCommandedEndEffectorVelocityState(model::Message *message) :
	model::Message(message),
	angularVelocityComponentX_rps(),
	angularVelocityComponentY_rps(),
	angularVelocityComponentZ_rps(),
	linearVelocityComponentX_mps(),
	linearVelocityComponentY_mps(),
	linearVelocityComponentZ_mps()
{
	this->id = ReportCommandedEndEffectorVelocityState::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&angularVelocityComponentX_rps);
	angularVelocityComponentX_rps.setName("AngularVelocityComponentX");
	angularVelocityComponentX_rps.setOptional(false);
	// Nothing to init

	fields.push_back(&angularVelocityComponentY_rps);
	angularVelocityComponentY_rps.setName("AngularVelocityComponentY");
	angularVelocityComponentY_rps.setOptional(false);
	// Nothing to init

	fields.push_back(&angularVelocityComponentZ_rps);
	angularVelocityComponentZ_rps.setName("AngularVelocityComponentZ");
	angularVelocityComponentZ_rps.setOptional(false);
	// Nothing to init

	fields.push_back(&linearVelocityComponentX_mps);
	linearVelocityComponentX_mps.setName("LinearVelocityComponentX");
	linearVelocityComponentX_mps.setOptional(false);
	// Nothing to init

	fields.push_back(&linearVelocityComponentY_mps);
	linearVelocityComponentY_mps.setName("LinearVelocityComponentY");
	linearVelocityComponentY_mps.setOptional(false);
	// Nothing to init

	fields.push_back(&linearVelocityComponentZ_mps);
	linearVelocityComponentZ_mps.setName("LinearVelocityComponentZ");
	linearVelocityComponentZ_mps.setOptional(false);
	// Nothing to init


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

ReportCommandedEndEffectorVelocityState::~ReportCommandedEndEffectorVelocityState()
{

}


double ReportCommandedEndEffectorVelocityState::getAngularVelocityComponentX_rps(void)
{
	return this->angularVelocityComponentX_rps.getValue();
}

void ReportCommandedEndEffectorVelocityState::setAngularVelocityComponentX_rps(double value)
{
	this->angularVelocityComponentX_rps.setValue(value);
}

double ReportCommandedEndEffectorVelocityState::getAngularVelocityComponentY_rps(void)
{
	return this->angularVelocityComponentY_rps.getValue();
}

void ReportCommandedEndEffectorVelocityState::setAngularVelocityComponentY_rps(double value)
{
	this->angularVelocityComponentY_rps.setValue(value);
}

double ReportCommandedEndEffectorVelocityState::getAngularVelocityComponentZ_rps(void)
{
	return this->angularVelocityComponentZ_rps.getValue();
}

void ReportCommandedEndEffectorVelocityState::setAngularVelocityComponentZ_rps(double value)
{
	this->angularVelocityComponentZ_rps.setValue(value);
}

double ReportCommandedEndEffectorVelocityState::getLinearVelocityComponentX_mps(void)
{
	return this->linearVelocityComponentX_mps.getValue();
}

void ReportCommandedEndEffectorVelocityState::setLinearVelocityComponentX_mps(double value)
{
	this->linearVelocityComponentX_mps.setValue(value);
}

double ReportCommandedEndEffectorVelocityState::getLinearVelocityComponentY_mps(void)
{
	return this->linearVelocityComponentY_mps.getValue();
}

void ReportCommandedEndEffectorVelocityState::setLinearVelocityComponentY_mps(double value)
{
	this->linearVelocityComponentY_mps.setValue(value);
}

double ReportCommandedEndEffectorVelocityState::getLinearVelocityComponentZ_mps(void)
{
	return this->linearVelocityComponentZ_mps.getValue();
}

void ReportCommandedEndEffectorVelocityState::setLinearVelocityComponentZ_mps(double value)
{
	this->linearVelocityComponentZ_mps.setValue(value);
}

int ReportCommandedEndEffectorVelocityState::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(angularVelocityComponentX_rps);
	byteSize += dst->pack(angularVelocityComponentY_rps);
	byteSize += dst->pack(angularVelocityComponentZ_rps);
	byteSize += dst->pack(linearVelocityComponentX_mps);
	byteSize += dst->pack(linearVelocityComponentY_mps);
	byteSize += dst->pack(linearVelocityComponentZ_mps);
	return byteSize;
}

int ReportCommandedEndEffectorVelocityState::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(angularVelocityComponentX_rps);
	byteSize += src->unpack(angularVelocityComponentY_rps);
	byteSize += src->unpack(angularVelocityComponentZ_rps);
	byteSize += src->unpack(linearVelocityComponentX_mps);
	byteSize += src->unpack(linearVelocityComponentY_mps);
	byteSize += src->unpack(linearVelocityComponentZ_mps);
	return byteSize;
}

int ReportCommandedEndEffectorVelocityState::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += angularVelocityComponentX_rps.length(); // angularVelocityComponentX_rps
	length += angularVelocityComponentY_rps.length(); // angularVelocityComponentY_rps
	length += angularVelocityComponentZ_rps.length(); // angularVelocityComponentZ_rps
	length += linearVelocityComponentX_mps.length(); // linearVelocityComponentX_mps
	length += linearVelocityComponentY_mps.length(); // linearVelocityComponentY_mps
	length += linearVelocityComponentZ_mps.length(); // linearVelocityComponentZ_mps
	return length;
}

std::string ReportCommandedEndEffectorVelocityState::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"ReportCommandedEndEffectorVelocityState\"";
	oss << " id=\"0x4612\" >\n";
	oss << angularVelocityComponentX_rps.toXml(level+1); // angularVelocityComponentX_rps
	oss << angularVelocityComponentY_rps.toXml(level+1); // angularVelocityComponentY_rps
	oss << angularVelocityComponentZ_rps.toXml(level+1); // angularVelocityComponentZ_rps
	oss << linearVelocityComponentX_mps.toXml(level+1); // linearVelocityComponentX_mps
	oss << linearVelocityComponentY_mps.toXml(level+1); // linearVelocityComponentY_mps
	oss << linearVelocityComponentZ_mps.toXml(level+1); // linearVelocityComponentZ_mps
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

} // namespace manipulator
} // namespace openjaus


