
/**
\file ReportLocalPathSegment.h

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
#include "openjaus/mobility/Triggers/ReportLocalPathSegment.h"

namespace openjaus
{
namespace mobility
{

ReportLocalPathSegment::ReportLocalPathSegment() : 
	model::Message(),
	p1X_m(),
	p1Y_m(),
	p1Z_m(),
	p2X_m(),
	p2Y_m(),
	p2Z_m(),
	weightingFactor(),
	pathTolerance()
{
	this->id = ReportLocalPathSegment::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&p1X_m);
	p1X_m.setName("P1X");
	p1X_m.setOptional(false);
	// Nothing to init

	fields.push_back(&p1Y_m);
	p1Y_m.setName("P1Y");
	p1Y_m.setOptional(false);
	// Nothing to init

	fields.push_back(&p1Z_m);
	p1Z_m.setName("P1Z");
	p1Z_m.setOptional(true);
	// Nothing to init

	fields.push_back(&p2X_m);
	p2X_m.setName("P2X");
	p2X_m.setOptional(false);
	// Nothing to init

	fields.push_back(&p2Y_m);
	p2Y_m.setName("P2Y");
	p2Y_m.setOptional(false);
	// Nothing to init

	fields.push_back(&p2Z_m);
	p2Z_m.setName("P2Z");
	p2Z_m.setOptional(true);
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

ReportLocalPathSegment::ReportLocalPathSegment(model::Message *message) :
	model::Message(message),
	p1X_m(),
	p1Y_m(),
	p1Z_m(),
	p2X_m(),
	p2Y_m(),
	p2Z_m(),
	weightingFactor(),
	pathTolerance()
{
	this->id = ReportLocalPathSegment::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&p1X_m);
	p1X_m.setName("P1X");
	p1X_m.setOptional(false);
	// Nothing to init

	fields.push_back(&p1Y_m);
	p1Y_m.setName("P1Y");
	p1Y_m.setOptional(false);
	// Nothing to init

	fields.push_back(&p1Z_m);
	p1Z_m.setName("P1Z");
	p1Z_m.setOptional(true);
	// Nothing to init

	fields.push_back(&p2X_m);
	p2X_m.setName("P2X");
	p2X_m.setOptional(false);
	// Nothing to init

	fields.push_back(&p2Y_m);
	p2Y_m.setName("P2Y");
	p2Y_m.setOptional(false);
	// Nothing to init

	fields.push_back(&p2Z_m);
	p2Z_m.setName("P2Z");
	p2Z_m.setOptional(true);
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

ReportLocalPathSegment::~ReportLocalPathSegment()
{

}


double ReportLocalPathSegment::getP1X_m(void)
{
	return this->p1X_m.getValue();
}

void ReportLocalPathSegment::setP1X_m(double value)
{
	this->p1X_m.setValue(value);
}

double ReportLocalPathSegment::getP1Y_m(void)
{
	return this->p1Y_m.getValue();
}

void ReportLocalPathSegment::setP1Y_m(double value)
{
	this->p1Y_m.setValue(value);
}

double ReportLocalPathSegment::getP1Z_m(void)
{
	return this->p1Z_m.getValue();
}

void ReportLocalPathSegment::setP1Z_m(double value)
{
	this->p1Z_m.setValue(value);
}

double ReportLocalPathSegment::getP2X_m(void)
{
	return this->p2X_m.getValue();
}

void ReportLocalPathSegment::setP2X_m(double value)
{
	this->p2X_m.setValue(value);
}

double ReportLocalPathSegment::getP2Y_m(void)
{
	return this->p2Y_m.getValue();
}

void ReportLocalPathSegment::setP2Y_m(double value)
{
	this->p2Y_m.setValue(value);
}

double ReportLocalPathSegment::getP2Z_m(void)
{
	return this->p2Z_m.getValue();
}

void ReportLocalPathSegment::setP2Z_m(double value)
{
	this->p2Z_m.setValue(value);
}

double ReportLocalPathSegment::getWeightingFactor(void)
{
	return this->weightingFactor.getValue();
}

void ReportLocalPathSegment::setWeightingFactor(double value)
{
	this->weightingFactor.setValue(value);
}

double ReportLocalPathSegment::getPathTolerance(void)
{
	return this->pathTolerance.getValue();
}

void ReportLocalPathSegment::setPathTolerance(double value)
{
	this->pathTolerance.setValue(value);
}

int ReportLocalPathSegment::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(this->presenceVector);
	byteSize += dst->pack(p1X_m);
	byteSize += dst->pack(p1Y_m);
	if(this->isP1ZEnabled())
	{
		byteSize += dst->pack(p1Z_m);
	}
	byteSize += dst->pack(p2X_m);
	byteSize += dst->pack(p2Y_m);
	if(this->isP2ZEnabled())
	{
		byteSize += dst->pack(p2Z_m);
	}
	byteSize += dst->pack(weightingFactor);
	if(this->isPathToleranceEnabled())
	{
		byteSize += dst->pack(pathTolerance);
	}
	return byteSize;
}

int ReportLocalPathSegment::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(this->presenceVector);
	byteSize += src->unpack(p1X_m);
	byteSize += src->unpack(p1Y_m);
	if(this->isP1ZEnabled())
	{
		byteSize += src->unpack(p1Z_m);
	}
	byteSize += src->unpack(p2X_m);
	byteSize += src->unpack(p2Y_m);
	if(this->isP2ZEnabled())
	{
		byteSize += src->unpack(p2Z_m);
	}
	byteSize += src->unpack(weightingFactor);
	if(this->isPathToleranceEnabled())
	{
		byteSize += src->unpack(pathTolerance);
	}
	return byteSize;
}

int ReportLocalPathSegment::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += sizeof(uint8_t); // PresenceVector
	length += p1X_m.length(); // p1X_m
	length += p1Y_m.length(); // p1Y_m
	if(this->isP1ZEnabled())
	{
		length += p1Z_m.length(); // p1Z_m
	}
	length += p2X_m.length(); // p2X_m
	length += p2Y_m.length(); // p2Y_m
	if(this->isP2ZEnabled())
	{
		length += p2Z_m.length(); // p2Z_m
	}
	length += weightingFactor.length(); // weightingFactor
	if(this->isPathToleranceEnabled())
	{
		length += pathTolerance.length(); // pathTolerance
	}
	return length;
}

std::string ReportLocalPathSegment::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"ReportLocalPathSegment\"";
	oss << " id=\"0x4410\" >\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint8_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isP1ZEnabled value=\"" << std::boolalpha << this->isP1ZEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isP2ZEnabled value=\"" << std::boolalpha << this->isP2ZEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isPathToleranceEnabled value=\"" << std::boolalpha << this->isPathToleranceEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	oss << p1X_m.toXml(level+1); // p1X_m
	oss << p1Y_m.toXml(level+1); // p1Y_m
	if(this->isP1ZEnabled())
	{
		oss << p1Z_m.toXml(level+1); // p1Z_m
	}
	oss << p2X_m.toXml(level+1); // p2X_m
	oss << p2Y_m.toXml(level+1); // p2Y_m
	if(this->isP2ZEnabled())
	{
		oss << p2Z_m.toXml(level+1); // p2Z_m
	}
	oss << weightingFactor.toXml(level+1); // weightingFactor
	if(this->isPathToleranceEnabled())
	{
		oss << pathTolerance.toXml(level+1); // pathTolerance
	}
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

void ReportLocalPathSegment::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t ReportLocalPathSegment::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool ReportLocalPathSegment::isP1ZEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportLocalPathSegment::P1Z_M));
}

void ReportLocalPathSegment::enableP1Z(void)
{
	this->presenceVector |= 0x01 << ReportLocalPathSegment::P1Z_M;
}

void ReportLocalPathSegment::disableP1Z(void)
{
	this->presenceVector &= ~(0x01 << ReportLocalPathSegment::P1Z_M);
}

bool ReportLocalPathSegment::isP2ZEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportLocalPathSegment::P2Z_M));
}

void ReportLocalPathSegment::enableP2Z(void)
{
	this->presenceVector |= 0x01 << ReportLocalPathSegment::P2Z_M;
}

void ReportLocalPathSegment::disableP2Z(void)
{
	this->presenceVector &= ~(0x01 << ReportLocalPathSegment::P2Z_M);
}

bool ReportLocalPathSegment::isPathToleranceEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportLocalPathSegment::PATHTOLERANCE));
}

void ReportLocalPathSegment::enablePathTolerance(void)
{
	this->presenceVector |= 0x01 << ReportLocalPathSegment::PATHTOLERANCE;
}

void ReportLocalPathSegment::disablePathTolerance(void)
{
	this->presenceVector &= ~(0x01 << ReportLocalPathSegment::PATHTOLERANCE);
}

} // namespace mobility
} // namespace openjaus


