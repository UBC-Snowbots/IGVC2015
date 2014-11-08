/**
\file LocalPathSegmentRecord.h

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
#include "openjaus/mobility/Triggers/Fields/LocalPathSegmentRecord.h"

namespace openjaus
{
namespace mobility
{

LocalPathSegmentRecord::LocalPathSegmentRecord():
	p1X_m(),
	p1Y_m(),
	p1Z_m(),
	p2X_m(),
	p2Y_m(),
	p2Z_m(),
	weightingFactor(),
	pathTolerance()
{
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

LocalPathSegmentRecord::LocalPathSegmentRecord(const LocalPathSegmentRecord &source)
{
	this->copy(const_cast<LocalPathSegmentRecord&>(source));
}

LocalPathSegmentRecord::~LocalPathSegmentRecord()
{

}


double LocalPathSegmentRecord::getP1X_m(void)
{
	return this->p1X_m.getValue();
}

void LocalPathSegmentRecord::setP1X_m(double value)
{
	this->p1X_m.setValue(value);
}

double LocalPathSegmentRecord::getP1Y_m(void)
{
	return this->p1Y_m.getValue();
}

void LocalPathSegmentRecord::setP1Y_m(double value)
{
	this->p1Y_m.setValue(value);
}

double LocalPathSegmentRecord::getP1Z_m(void)
{
	return this->p1Z_m.getValue();
}

void LocalPathSegmentRecord::setP1Z_m(double value)
{
	this->p1Z_m.setValue(value);
}

double LocalPathSegmentRecord::getP2X_m(void)
{
	return this->p2X_m.getValue();
}

void LocalPathSegmentRecord::setP2X_m(double value)
{
	this->p2X_m.setValue(value);
}

double LocalPathSegmentRecord::getP2Y_m(void)
{
	return this->p2Y_m.getValue();
}

void LocalPathSegmentRecord::setP2Y_m(double value)
{
	this->p2Y_m.setValue(value);
}

double LocalPathSegmentRecord::getP2Z_m(void)
{
	return this->p2Z_m.getValue();
}

void LocalPathSegmentRecord::setP2Z_m(double value)
{
	this->p2Z_m.setValue(value);
}

double LocalPathSegmentRecord::getWeightingFactor(void)
{
	return this->weightingFactor.getValue();
}

void LocalPathSegmentRecord::setWeightingFactor(double value)
{
	this->weightingFactor.setValue(value);
}

double LocalPathSegmentRecord::getPathTolerance(void)
{
	return this->pathTolerance.getValue();
}

void LocalPathSegmentRecord::setPathTolerance(double value)
{
	this->pathTolerance.setValue(value);
}

int LocalPathSegmentRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
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
int LocalPathSegmentRecord::from(system::Buffer *src)
{
	int byteSize = 0;
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

int LocalPathSegmentRecord::length(void)
{
	int length = 0;
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

std::string LocalPathSegmentRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"LocalPathSegmentRecord\">\n";
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
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}


void LocalPathSegmentRecord::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t LocalPathSegmentRecord::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool LocalPathSegmentRecord::isP1ZEnabled(void) const
{
	return (this->presenceVector & (0x01 << LocalPathSegmentRecord::P1Z_M));
}

void LocalPathSegmentRecord::enableP1Z(void)
{
	this->presenceVector |= 0x01 << LocalPathSegmentRecord::P1Z_M;
}

void LocalPathSegmentRecord::disableP1Z(void)
{
	this->presenceVector &= ~(0x01 << LocalPathSegmentRecord::P1Z_M);
}

bool LocalPathSegmentRecord::isP2ZEnabled(void) const
{
	return (this->presenceVector & (0x01 << LocalPathSegmentRecord::P2Z_M));
}

void LocalPathSegmentRecord::enableP2Z(void)
{
	this->presenceVector |= 0x01 << LocalPathSegmentRecord::P2Z_M;
}

void LocalPathSegmentRecord::disableP2Z(void)
{
	this->presenceVector &= ~(0x01 << LocalPathSegmentRecord::P2Z_M);
}

bool LocalPathSegmentRecord::isPathToleranceEnabled(void) const
{
	return (this->presenceVector & (0x01 << LocalPathSegmentRecord::PATHTOLERANCE));
}

void LocalPathSegmentRecord::enablePathTolerance(void)
{
	this->presenceVector |= 0x01 << LocalPathSegmentRecord::PATHTOLERANCE;
}

void LocalPathSegmentRecord::disablePathTolerance(void)
{
	this->presenceVector &= ~(0x01 << LocalPathSegmentRecord::PATHTOLERANCE);
}


void LocalPathSegmentRecord::copy(LocalPathSegmentRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->presenceVector = source.getPresenceVector();
	this->p1X_m.setName("LocalPosition");
	this->p1X_m.setOptional(true);
	this->p1X_m.setValue(source.getP1X_m()); 
 
	this->p1Y_m.setName("LocalPosition");
	this->p1Y_m.setOptional(true);
	this->p1Y_m.setValue(source.getP1Y_m()); 
 
	this->p1Z_m.setName("JausAltitude");
	this->p1Z_m.setOptional(true);
	this->p1Z_m.setValue(source.getP1Z_m()); 
 
	this->p2X_m.setName("LocalPosition");
	this->p2X_m.setOptional(true);
	this->p2X_m.setValue(source.getP2X_m()); 
 
	this->p2Y_m.setName("LocalPosition");
	this->p2Y_m.setOptional(true);
	this->p2Y_m.setValue(source.getP2Y_m()); 
 
	this->p2Z_m.setName("JausAltitude");
	this->p2Z_m.setOptional(true);
	this->p2Z_m.setValue(source.getP2Z_m()); 
 
	this->weightingFactor.setName("WeightingFactorRef");
	this->weightingFactor.setOptional(false);
	this->weightingFactor.setInterpretation("Where 0 is a straight line.");
	this->weightingFactor.setValue(source.getWeightingFactor()); 
 
	this->pathTolerance.setName("PathToleranceRef");
	this->pathTolerance.setOptional(true);
	this->pathTolerance.setInterpretation("A value of 0 is used for infinite tolerance.");
	this->pathTolerance.setValue(source.getPathTolerance()); 
 
}

} // namespace mobility
} // namespace openjaus

