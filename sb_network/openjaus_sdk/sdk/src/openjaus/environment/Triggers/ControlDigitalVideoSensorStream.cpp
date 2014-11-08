
/**
\file ControlDigitalVideoSensorStream.h

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
#include "openjaus/environment/Triggers/ControlDigitalVideoSensorStream.h"

namespace openjaus
{
namespace environment
{

ControlDigitalVideoSensorStream::ControlDigitalVideoSensorStream() : 
	model::Message(),
	sensorID(),
	streamState()
{
	this->id = ControlDigitalVideoSensorStream::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&sensorID);
	sensorID.setName("SensorID");
	sensorID.setOptional(false);
	sensorID.setValue(0);

	fields.push_back(&streamState);
	streamState.setName("StreamState");
	streamState.setOptional(false);
	// Nothing to init

}

ControlDigitalVideoSensorStream::ControlDigitalVideoSensorStream(model::Message *message) :
	model::Message(message),
	sensorID(),
	streamState()
{
	this->id = ControlDigitalVideoSensorStream::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&sensorID);
	sensorID.setName("SensorID");
	sensorID.setOptional(false);
	sensorID.setValue(0);

	fields.push_back(&streamState);
	streamState.setName("StreamState");
	streamState.setOptional(false);
	// Nothing to init


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

ControlDigitalVideoSensorStream::~ControlDigitalVideoSensorStream()
{

}


uint16_t ControlDigitalVideoSensorStream::getSensorID(void)
{
	return this->sensorID.getValue();
}

void ControlDigitalVideoSensorStream::setSensorID(uint16_t value)
{
	this->sensorID.setValue(value);
}

StreamStateEnumeration::StreamStateEnum ControlDigitalVideoSensorStream::getStreamState(void)
{
	return this->streamState.getValue();
}

void ControlDigitalVideoSensorStream::setStreamState(StreamStateEnumeration::StreamStateEnum value)
{
	this->streamState.setValue(value);
}

int ControlDigitalVideoSensorStream::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(sensorID);
	byteSize += dst->pack(streamState);
	return byteSize;
}

int ControlDigitalVideoSensorStream::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(sensorID);
	byteSize += src->unpack(streamState);
	return byteSize;
}

int ControlDigitalVideoSensorStream::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += sensorID.length(); // sensorID
	length += streamState.length(); // streamState
	return length;
}

std::string ControlDigitalVideoSensorStream::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"ControlDigitalVideoSensorStream\"";
	oss << " id=\"0x0805\" >\n";
	oss << sensorID.toXml(level+1); // sensorID
	oss << streamState.toXml(level+1); // streamState
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

} // namespace environment
} // namespace openjaus


