
/**
\file SetTravelSpeed.h

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
#include "openjaus/mobility/Triggers/SetTravelSpeed.h"

namespace openjaus
{
namespace mobility
{

SetTravelSpeed::SetTravelSpeed() : 
	model::Message(),
	speed_mps()
{
	this->id = SetTravelSpeed::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&speed_mps);
	speed_mps.setName("Speed");
	speed_mps.setOptional(false);
	// Nothing to init

}

SetTravelSpeed::SetTravelSpeed(model::Message *message) :
	model::Message(message),
	speed_mps()
{
	this->id = SetTravelSpeed::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&speed_mps);
	speed_mps.setName("Speed");
	speed_mps.setOptional(false);
	// Nothing to init


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

SetTravelSpeed::~SetTravelSpeed()
{

}


double SetTravelSpeed::getSpeed_mps(void)
{
	return this->speed_mps.getValue();
}

void SetTravelSpeed::setSpeed_mps(double value)
{
	this->speed_mps.setValue(value);
}

int SetTravelSpeed::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(speed_mps);
	return byteSize;
}

int SetTravelSpeed::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(speed_mps);
	return byteSize;
}

int SetTravelSpeed::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += speed_mps.length(); // speed_mps
	return length;
}

std::string SetTravelSpeed::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"SetTravelSpeed\"";
	oss << " id=\"0x040A\" >\n";
	oss << speed_mps.toXml(level+1); // speed_mps
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

} // namespace mobility
} // namespace openjaus


