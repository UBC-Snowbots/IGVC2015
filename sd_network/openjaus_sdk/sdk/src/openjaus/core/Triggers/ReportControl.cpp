
/**
\file ReportControl.h

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
#include "openjaus/core/Triggers/ReportControl.h"

namespace openjaus
{
namespace core
{

ReportControl::ReportControl() : 
	model::Message(),
	subsystemID(),
	nodeID(),
	componentID(),
	authorityCode()
{
	this->id = ReportControl::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&subsystemID);
	subsystemID.setName("SubsystemID");
	subsystemID.setOptional(false);
	subsystemID.setValue(0);

	fields.push_back(&nodeID);
	nodeID.setName("NodeID");
	nodeID.setOptional(false);
	nodeID.setValue(0);

	fields.push_back(&componentID);
	componentID.setName("ComponentID");
	componentID.setOptional(false);
	componentID.setValue(0);

	fields.push_back(&authorityCode);
	authorityCode.setName("AuthorityCode");
	authorityCode.setOptional(false);
	authorityCode.setValue(0);

}

ReportControl::ReportControl(model::Message *message) :
	model::Message(message),
	subsystemID(),
	nodeID(),
	componentID(),
	authorityCode()
{
	this->id = ReportControl::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&subsystemID);
	subsystemID.setName("SubsystemID");
	subsystemID.setOptional(false);
	subsystemID.setValue(0);

	fields.push_back(&nodeID);
	nodeID.setName("NodeID");
	nodeID.setOptional(false);
	nodeID.setValue(0);

	fields.push_back(&componentID);
	componentID.setName("ComponentID");
	componentID.setOptional(false);
	componentID.setValue(0);

	fields.push_back(&authorityCode);
	authorityCode.setName("AuthorityCode");
	authorityCode.setOptional(false);
	authorityCode.setValue(0);


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

ReportControl::~ReportControl()
{

}


uint16_t ReportControl::getSubsystemID(void)
{
	return this->subsystemID.getValue();
}

void ReportControl::setSubsystemID(uint16_t value)
{
	this->subsystemID.setValue(value);
}

uint8_t ReportControl::getNodeID(void)
{
	return this->nodeID.getValue();
}

void ReportControl::setNodeID(uint8_t value)
{
	this->nodeID.setValue(value);
}

uint8_t ReportControl::getComponentID(void)
{
	return this->componentID.getValue();
}

void ReportControl::setComponentID(uint8_t value)
{
	this->componentID.setValue(value);
}

uint8_t ReportControl::getAuthorityCode(void)
{
	return this->authorityCode.getValue();
}

void ReportControl::setAuthorityCode(uint8_t value)
{
	this->authorityCode.setValue(value);
}

int ReportControl::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(subsystemID);
	byteSize += dst->pack(nodeID);
	byteSize += dst->pack(componentID);
	byteSize += dst->pack(authorityCode);
	return byteSize;
}

int ReportControl::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(subsystemID);
	byteSize += src->unpack(nodeID);
	byteSize += src->unpack(componentID);
	byteSize += src->unpack(authorityCode);
	return byteSize;
}

int ReportControl::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += subsystemID.length(); // subsystemID
	length += nodeID.length(); // nodeID
	length += componentID.length(); // componentID
	length += authorityCode.length(); // authorityCode
	return length;
}

std::string ReportControl::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"ReportControl\"";
	oss << " id=\"0x400D\" >\n";
	oss << subsystemID.toXml(level+1); // subsystemID
	oss << nodeID.toXml(level+1); // nodeID
	oss << componentID.toXml(level+1); // componentID
	oss << authorityCode.toXml(level+1); // authorityCode
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

} // namespace core
} // namespace openjaus


