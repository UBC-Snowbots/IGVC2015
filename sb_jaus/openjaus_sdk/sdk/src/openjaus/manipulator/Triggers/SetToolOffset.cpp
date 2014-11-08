
/**
\file SetToolOffset.h

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
#include "openjaus/manipulator/Triggers/SetToolOffset.h"

namespace openjaus
{
namespace manipulator
{

SetToolOffset::SetToolOffset() : 
	model::Message(),
	toolPointCoordinateX_m(),
	toolPointCoordinateY_m(),
	toolPointCoordinateZ_m()
{
	this->id = SetToolOffset::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&toolPointCoordinateX_m);
	toolPointCoordinateX_m.setName("ToolPointCoordinateX");
	toolPointCoordinateX_m.setOptional(false);
	// Nothing to init

	fields.push_back(&toolPointCoordinateY_m);
	toolPointCoordinateY_m.setName("ToolPointCoordinateY");
	toolPointCoordinateY_m.setOptional(false);
	// Nothing to init

	fields.push_back(&toolPointCoordinateZ_m);
	toolPointCoordinateZ_m.setName("ToolPointCoordinateZ");
	toolPointCoordinateZ_m.setOptional(false);
	// Nothing to init

}

SetToolOffset::SetToolOffset(model::Message *message) :
	model::Message(message),
	toolPointCoordinateX_m(),
	toolPointCoordinateY_m(),
	toolPointCoordinateZ_m()
{
	this->id = SetToolOffset::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&toolPointCoordinateX_m);
	toolPointCoordinateX_m.setName("ToolPointCoordinateX");
	toolPointCoordinateX_m.setOptional(false);
	// Nothing to init

	fields.push_back(&toolPointCoordinateY_m);
	toolPointCoordinateY_m.setName("ToolPointCoordinateY");
	toolPointCoordinateY_m.setOptional(false);
	// Nothing to init

	fields.push_back(&toolPointCoordinateZ_m);
	toolPointCoordinateZ_m.setName("ToolPointCoordinateZ");
	toolPointCoordinateZ_m.setOptional(false);
	// Nothing to init


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

SetToolOffset::~SetToolOffset()
{

}


double SetToolOffset::getToolPointCoordinateX_m(void)
{
	return this->toolPointCoordinateX_m.getValue();
}

void SetToolOffset::setToolPointCoordinateX_m(double value)
{
	this->toolPointCoordinateX_m.setValue(value);
}

double SetToolOffset::getToolPointCoordinateY_m(void)
{
	return this->toolPointCoordinateY_m.getValue();
}

void SetToolOffset::setToolPointCoordinateY_m(double value)
{
	this->toolPointCoordinateY_m.setValue(value);
}

double SetToolOffset::getToolPointCoordinateZ_m(void)
{
	return this->toolPointCoordinateZ_m.getValue();
}

void SetToolOffset::setToolPointCoordinateZ_m(double value)
{
	this->toolPointCoordinateZ_m.setValue(value);
}

int SetToolOffset::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(toolPointCoordinateX_m);
	byteSize += dst->pack(toolPointCoordinateY_m);
	byteSize += dst->pack(toolPointCoordinateZ_m);
	return byteSize;
}

int SetToolOffset::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(toolPointCoordinateX_m);
	byteSize += src->unpack(toolPointCoordinateY_m);
	byteSize += src->unpack(toolPointCoordinateZ_m);
	return byteSize;
}

int SetToolOffset::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += toolPointCoordinateX_m.length(); // toolPointCoordinateX_m
	length += toolPointCoordinateY_m.length(); // toolPointCoordinateY_m
	length += toolPointCoordinateZ_m.length(); // toolPointCoordinateZ_m
	return length;
}

std::string SetToolOffset::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"SetToolOffset\"";
	oss << " id=\"0x0604\" >\n";
	oss << toolPointCoordinateX_m.toXml(level+1); // toolPointCoordinateX_m
	oss << toolPointCoordinateY_m.toXml(level+1); // toolPointCoordinateY_m
	oss << toolPointCoordinateZ_m.toXml(level+1); // toolPointCoordinateZ_m
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

} // namespace manipulator
} // namespace openjaus


