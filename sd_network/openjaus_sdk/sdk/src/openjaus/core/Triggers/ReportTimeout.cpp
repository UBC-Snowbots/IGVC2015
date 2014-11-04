
/**
\file ReportTimeout.h

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
#include "openjaus/core/Triggers/ReportTimeout.h"

namespace openjaus
{
namespace core
{

ReportTimeout::ReportTimeout() : 
	model::Message(),
	timeout_sec()
{
	this->id = ReportTimeout::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&timeout_sec);
	timeout_sec.setName("Timeout");
	timeout_sec.setOptional(false);
	timeout_sec.setInterpretation("Clients must re-request control to prevent being rejected when the timeout expires.  A value of zero indicates this feature is disabled.");
	timeout_sec.setValue(0);

}

ReportTimeout::ReportTimeout(model::Message *message) :
	model::Message(message),
	timeout_sec()
{
	this->id = ReportTimeout::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&timeout_sec);
	timeout_sec.setName("Timeout");
	timeout_sec.setOptional(false);
	timeout_sec.setInterpretation("Clients must re-request control to prevent being rejected when the timeout expires.  A value of zero indicates this feature is disabled.");
	timeout_sec.setValue(0);


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

ReportTimeout::~ReportTimeout()
{

}


uint8_t ReportTimeout::getTimeout_sec(void)
{
	return this->timeout_sec.getValue();
}

void ReportTimeout::setTimeout_sec(uint8_t value)
{
	this->timeout_sec.setValue(value);
}

int ReportTimeout::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(timeout_sec);
	return byteSize;
}

int ReportTimeout::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(timeout_sec);
	return byteSize;
}

int ReportTimeout::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += timeout_sec.length(); // timeout_sec
	return length;
}

std::string ReportTimeout::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"ReportTimeout\"";
	oss << " id=\"0x4003\" >\n";
	oss << timeout_sec.toXml(level+1); // timeout_sec
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

} // namespace core
} // namespace openjaus


