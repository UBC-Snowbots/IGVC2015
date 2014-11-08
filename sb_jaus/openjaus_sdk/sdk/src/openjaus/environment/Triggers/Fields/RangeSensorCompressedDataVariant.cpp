

/**
\file RangeSensorCompressedDataVariant.h

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
#include "openjaus/environment/Triggers/Fields/RangeSensorCompressedDataVariant.h"

namespace openjaus
{
namespace environment
{

RangeSensorCompressedDataVariant::RangeSensorCompressedDataVariant() :
	type(),
	rangeSensorDataErrorRec(),
	rangeSensorCompressedDataRec()
{
}

RangeSensorCompressedDataVariant::~RangeSensorCompressedDataVariant()
{

}

void RangeSensorCompressedDataVariant::setType(RangeSensorCompressedDataVariant::TypeEnum type)
{
	this->type = type;
}

RangeSensorCompressedDataVariant::TypeEnum RangeSensorCompressedDataVariant::getType(void)
{
	return this->type;
}

std::string RangeSensorCompressedDataVariant::typeToString(void)
{
	switch(this->type)
	{
		case RANGESENSORDATAERRORREC:
			return "RANGESENSORDATAERRORREC";
		case RANGESENSORCOMPRESSEDDATAREC:
			return "RANGESENSORCOMPRESSEDDATAREC";
		default:
			return "Unknown";
	}
}

void RangeSensorCompressedDataVariant::copy(RangeSensorCompressedDataVariant& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	this->type = source.getType();
	switch(this->type)
	{
		case RANGESENSORDATAERRORREC:
			this->rangeSensorDataErrorRec.copy(source.getRangeSensorDataErrorRec());
			break;

		case RANGESENSORCOMPRESSEDDATAREC:
			this->rangeSensorCompressedDataRec.copy(source.getRangeSensorCompressedDataRec());
			break;

		default:
			THROW_EXCEPTION("Unknown RangeSensorCompressedDataVariant Type: " << this->type);
	}
}

int RangeSensorCompressedDataVariant::to(system::Buffer *dst)
{
	int result = 0;
	result += dst->pack(static_cast<uint8_t>(this->type));
	switch(this->type)
	{
		case RANGESENSORDATAERRORREC:
			result += dst->pack(rangeSensorDataErrorRec);
			return result;

		case RANGESENSORCOMPRESSEDDATAREC:
			result += dst->pack(rangeSensorCompressedDataRec);
			return result;

		default:
			THROW_EXCEPTION("Unknown RangeSensorCompressedDataVariant Type: " << this->type);
	}
}

int RangeSensorCompressedDataVariant::from(system::Buffer *src)
{
	int result = 0;

	uint8_t intValue;
	result += src->unpack(intValue);
	this->type = static_cast<RangeSensorCompressedDataVariant::TypeEnum>(intValue);

	switch(this->type)
	{
		case RANGESENSORDATAERRORREC:
			result += src->unpack(rangeSensorDataErrorRec);
			return result;

		case RANGESENSORCOMPRESSEDDATAREC:
			result += src->unpack(rangeSensorCompressedDataRec);
			return result;

		default:
			THROW_EXCEPTION("Unknown RangeSensorCompressedDataVariant Type: " << this->type);
	}
}

int RangeSensorCompressedDataVariant::length(void)
{
	int result = 0;
	result += sizeof(uint8_t); // Type
	switch(this->type)
	{
		case RANGESENSORDATAERRORREC:
			result += rangeSensorDataErrorRec.length();
			return result;

		case RANGESENSORCOMPRESSEDDATAREC:
			result += rangeSensorCompressedDataRec.length();
			return result;

		default:
			THROW_EXCEPTION("Unknown RangeSensorCompressedDataVariant Type: " << this->type);
	}
}

std::string RangeSensorCompressedDataVariant::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Variant name=\"" << this->name << "\">\n";

	switch(this->type)
	{
		case RANGESENSORDATAERRORREC:
			oss << prefix.str() << "\t" << "<type>RANGESENSORDATAERRORREC</type>\n";
			oss << rangeSensorDataErrorRec.toXml(level+1);
			oss << prefix.str() << "</Variant>\n";
			return oss.str();

		case RANGESENSORCOMPRESSEDDATAREC:
			oss << prefix.str() << "\t" << "<type>RANGESENSORCOMPRESSEDDATAREC</type>\n";
			oss << rangeSensorCompressedDataRec.toXml(level+1);
			oss << prefix.str() << "</Variant>\n";
			return oss.str();

		default:
			THROW_EXCEPTION("Unknown RangeSensorCompressedDataVariant Type: " << this->type);
	}
}




RangeSensorDataErrorRecRefRecord& RangeSensorCompressedDataVariant::getRangeSensorDataErrorRec(void)
{
	if(this->type != RANGESENSORDATAERRORREC)
	{
		LOG("Misinterpretation of Variant (RangeSensorCompressedDataVariant). Requesting object of type \"RANGESENSORDATAERRORREC\" while variant is of type \"" << this->typeToString() << "\"" << std::endl);
	}
	return this->rangeSensorDataErrorRec;
}

RangeSensorCompressedDataRecord& RangeSensorCompressedDataVariant::getRangeSensorCompressedDataRec(void)
{
	if(this->type != RANGESENSORCOMPRESSEDDATAREC)
	{
		LOG("Misinterpretation of Variant (RangeSensorCompressedDataVariant). Requesting object of type \"RANGESENSORCOMPRESSEDDATAREC\" while variant is of type \"" << this->typeToString() << "\"" << std::endl);
	}
	return this->rangeSensorCompressedDataRec;
}

} // namespace environment
} // namespace openjaus

