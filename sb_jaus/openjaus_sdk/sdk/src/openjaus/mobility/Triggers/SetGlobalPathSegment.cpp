
/**
\file SetGlobalPathSegment.h

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
#include "openjaus/mobility/Triggers/SetGlobalPathSegment.h"

namespace openjaus
{
namespace mobility
{

SetGlobalPathSegment::SetGlobalPathSegment() : 
	model::Message(),
	p1Latitude_deg(),
	p1Longitude_deg(),
	p1Altitude_m(),
	p2Latitude_deg(),
	p2Longitude_deg(),
	p2Altitude_m(),
	weightingFactor(),
	pathTolerance()
{
	this->id = SetGlobalPathSegment::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&p1Latitude_deg);
	p1Latitude_deg.setName("P1Latitude");
	p1Latitude_deg.setOptional(false);
	// Nothing to init

	fields.push_back(&p1Longitude_deg);
	p1Longitude_deg.setName("P1Longitude");
	p1Longitude_deg.setOptional(false);
	// Nothing to init

	fields.push_back(&p1Altitude_m);
	p1Altitude_m.setName("P1Altitude");
	p1Altitude_m.setOptional(true);
	// Nothing to init

	fields.push_back(&p2Latitude_deg);
	p2Latitude_deg.setName("P2Latitude");
	p2Latitude_deg.setOptional(false);
	// Nothing to init

	fields.push_back(&p2Longitude_deg);
	p2Longitude_deg.setName("P2Longitude");
	p2Longitude_deg.setOptional(false);
	// Nothing to init

	fields.push_back(&p2Altitude_m);
	p2Altitude_m.setName("P2Altitude");
	p2Altitude_m.setOptional(true);
	// Nothing to init

	fields.push_back(&weightingFactor);
	weightingFactor.setName("WeightingFactor");
	weightingFactor.setOptional(false);
	weightingFactor.setInterpretation("Where 0 is a straight line.");
	// Nothing to init

	fields.push_back(&pathTolerance);
	pathTolerance.setName("PathTolerance");
	pathTolerance.setOptional(true);
	pathTolerance.setInterpretation("A value of 0 is used for infinite tolerance.");
	// Nothing to init

}

SetGlobalPathSegment::SetGlobalPathSegment(model::Message *message) :
	model::Message(message),
	p1Latitude_deg(),
	p1Longitude_deg(),
	p1Altitude_m(),
	p2Latitude_deg(),
	p2Longitude_deg(),
	p2Altitude_m(),
	weightingFactor(),
	pathTolerance()
{
	this->id = SetGlobalPathSegment::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&p1Latitude_deg);
	p1Latitude_deg.setName("P1Latitude");
	p1Latitude_deg.setOptional(false);
	// Nothing to init

	fields.push_back(&p1Longitude_deg);
	p1Longitude_deg.setName("P1Longitude");
	p1Longitude_deg.setOptional(false);
	// Nothing to init

	fields.push_back(&p1Altitude_m);
	p1Altitude_m.setName("P1Altitude");
	p1Altitude_m.setOptional(true);
	// Nothing to init

	fields.push_back(&p2Latitude_deg);
	p2Latitude_deg.setName("P2Latitude");
	p2Latitude_deg.setOptional(false);
	// Nothing to init

	fields.push_back(&p2Longitude_deg);
	p2Longitude_deg.setName("P2Longitude");
	p2Longitude_deg.setOptional(false);
	// Nothing to init

	fields.push_back(&p2Altitude_m);
	p2Altitude_m.setName("P2Altitude");
	p2Altitude_m.setOptional(true);
	// Nothing to init

	fields.push_back(&weightingFactor);
	weightingFactor.setName("WeightingFactor");
	weightingFactor.setOptional(false);
	weightingFactor.setInterpretation("Where 0 is a straight line.");
	// Nothing to init

	fields.push_back(&pathTolerance);
	pathTolerance.setName("PathTolerance");
	pathTolerance.setOptional(true);
	pathTolerance.setInterpretation("A value of 0 is used for infinite tolerance.");
	// Nothing to init


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

SetGlobalPathSegment::~SetGlobalPathSegment()
{

}


double SetGlobalPathSegment::getP1Latitude_deg(void)
{
	return this->p1Latitude_deg.getValue();
}

void SetGlobalPathSegment::setP1Latitude_deg(double value)
{
	this->p1Latitude_deg.setValue(value);
}

double SetGlobalPathSegment::getP1Longitude_deg(void)
{
	return this->p1Longitude_deg.getValue();
}

void SetGlobalPathSegment::setP1Longitude_deg(double value)
{
	this->p1Longitude_deg.setValue(value);
}

double SetGlobalPathSegment::getP1Altitude_m(void)
{
	return this->p1Altitude_m.getValue();
}

void SetGlobalPathSegment::setP1Altitude_m(double value)
{
	this->p1Altitude_m.setValue(value);
}

double SetGlobalPathSegment::getP2Latitude_deg(void)
{
	return this->p2Latitude_deg.getValue();
}

void SetGlobalPathSegment::setP2Latitude_deg(double value)
{
	this->p2Latitude_deg.setValue(value);
}

double SetGlobalPathSegment::getP2Longitude_deg(void)
{
	return this->p2Longitude_deg.getValue();
}

void SetGlobalPathSegment::setP2Longitude_deg(double value)
{
	this->p2Longitude_deg.setValue(value);
}

double SetGlobalPathSegment::getP2Altitude_m(void)
{
	return this->p2Altitude_m.getValue();
}

void SetGlobalPathSegment::setP2Altitude_m(double value)
{
	this->p2Altitude_m.setValue(value);
}

double SetGlobalPathSegment::getWeightingFactor(void)
{
	return this->weightingFactor.getValue();
}

void SetGlobalPathSegment::setWeightingFactor(double value)
{
	this->weightingFactor.setValue(value);
}

double SetGlobalPathSegment::getPathTolerance(void)
{
	return this->pathTolerance.getValue();
}

void SetGlobalPathSegment::setPathTolerance(double value)
{
	this->pathTolerance.setValue(value);
}

int SetGlobalPathSegment::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(this->presenceVector);
	byteSize += dst->pack(p1Latitude_deg);
	byteSize += dst->pack(p1Longitude_deg);
	if(this->isP1AltitudeEnabled())
	{
		byteSize += dst->pack(p1Altitude_m);
	}
	byteSize += dst->pack(p2Latitude_deg);
	byteSize += dst->pack(p2Longitude_deg);
	if(this->isP2AltitudeEnabled())
	{
		byteSize += dst->pack(p2Altitude_m);
	}
	byteSize += dst->pack(weightingFactor);
	if(this->isPathToleranceEnabled())
	{
		byteSize += dst->pack(pathTolerance);
	}
	return byteSize;
}

int SetGlobalPathSegment::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(this->presenceVector);
	byteSize += src->unpack(p1Latitude_deg);
	byteSize += src->unpack(p1Longitude_deg);
	if(this->isP1AltitudeEnabled())
	{
		byteSize += src->unpack(p1Altitude_m);
	}
	byteSize += src->unpack(p2Latitude_deg);
	byteSize += src->unpack(p2Longitude_deg);
	if(this->isP2AltitudeEnabled())
	{
		byteSize += src->unpack(p2Altitude_m);
	}
	byteSize += src->unpack(weightingFactor);
	if(this->isPathToleranceEnabled())
	{
		byteSize += src->unpack(pathTolerance);
	}
	return byteSize;
}

int SetGlobalPathSegment::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += sizeof(uint8_t); // PresenceVector
	length += p1Latitude_deg.length(); // p1Latitude_deg
	length += p1Longitude_deg.length(); // p1Longitude_deg
	if(this->isP1AltitudeEnabled())
	{
		length += p1Altitude_m.length(); // p1Altitude_m
	}
	length += p2Latitude_deg.length(); // p2Latitude_deg
	length += p2Longitude_deg.length(); // p2Longitude_deg
	if(this->isP2AltitudeEnabled())
	{
		length += p2Altitude_m.length(); // p2Altitude_m
	}
	length += weightingFactor.length(); // weightingFactor
	if(this->isPathToleranceEnabled())
	{
		length += pathTolerance.length(); // pathTolerance
	}
	return length;
}

std::string SetGlobalPathSegment::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"SetGlobalPathSegment\"";
	oss << " id=\"0x040F\" >\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint8_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isP1AltitudeEnabled value=\"" << std::boolalpha << this->isP1AltitudeEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isP2AltitudeEnabled value=\"" << std::boolalpha << this->isP2AltitudeEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isPathToleranceEnabled value=\"" << std::boolalpha << this->isPathToleranceEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	oss << p1Latitude_deg.toXml(level+1); // p1Latitude_deg
	oss << p1Longitude_deg.toXml(level+1); // p1Longitude_deg
	if(this->isP1AltitudeEnabled())
	{
		oss << p1Altitude_m.toXml(level+1); // p1Altitude_m
	}
	oss << p2Latitude_deg.toXml(level+1); // p2Latitude_deg
	oss << p2Longitude_deg.toXml(level+1); // p2Longitude_deg
	if(this->isP2AltitudeEnabled())
	{
		oss << p2Altitude_m.toXml(level+1); // p2Altitude_m
	}
	oss << weightingFactor.toXml(level+1); // weightingFactor
	if(this->isPathToleranceEnabled())
	{
		oss << pathTolerance.toXml(level+1); // pathTolerance
	}
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

void SetGlobalPathSegment::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t SetGlobalPathSegment::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool SetGlobalPathSegment::isP1AltitudeEnabled(void) const
{
	return (this->presenceVector & (0x01 << SetGlobalPathSegment::P1ALTITUDE_M));
}

void SetGlobalPathSegment::enableP1Altitude(void)
{
	this->presenceVector |= 0x01 << SetGlobalPathSegment::P1ALTITUDE_M;
}

void SetGlobalPathSegment::disableP1Altitude(void)
{
	this->presenceVector &= ~(0x01 << SetGlobalPathSegment::P1ALTITUDE_M);
}

bool SetGlobalPathSegment::isP2AltitudeEnabled(void) const
{
	return (this->presenceVector & (0x01 << SetGlobalPathSegment::P2ALTITUDE_M));
}

void SetGlobalPathSegment::enableP2Altitude(void)
{
	this->presenceVector |= 0x01 << SetGlobalPathSegment::P2ALTITUDE_M;
}

void SetGlobalPathSegment::disableP2Altitude(void)
{
	this->presenceVector &= ~(0x01 << SetGlobalPathSegment::P2ALTITUDE_M);
}

bool SetGlobalPathSegment::isPathToleranceEnabled(void) const
{
	return (this->presenceVector & (0x01 << SetGlobalPathSegment::PATHTOLERANCE));
}

void SetGlobalPathSegment::enablePathTolerance(void)
{
	this->presenceVector |= 0x01 << SetGlobalPathSegment::PATHTOLERANCE;
}

void SetGlobalPathSegment::disablePathTolerance(void)
{
	this->presenceVector &= ~(0x01 << SetGlobalPathSegment::PATHTOLERANCE);
}

} // namespace mobility
} // namespace openjaus


