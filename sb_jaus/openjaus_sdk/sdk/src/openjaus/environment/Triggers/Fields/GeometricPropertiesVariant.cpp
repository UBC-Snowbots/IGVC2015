

/**
\file GeometricPropertiesVariant.h

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
#include "openjaus/environment/Triggers/Fields/GeometricPropertiesVariant.h"

namespace openjaus
{
namespace environment
{

GeometricPropertiesVariant::GeometricPropertiesVariant() :
	type(),
	noGeometricPropertiesVariant(),
	staticGeometricPropertiesRec(),
	manipulatorGeometricPropertiesRec()
{
}

GeometricPropertiesVariant::~GeometricPropertiesVariant()
{

}

void GeometricPropertiesVariant::setType(GeometricPropertiesVariant::TypeEnum type)
{
	this->type = type;
}

GeometricPropertiesVariant::TypeEnum GeometricPropertiesVariant::getType(void)
{
	return this->type;
}

std::string GeometricPropertiesVariant::typeToString(void)
{
	switch(this->type)
	{
		case NOGEOMETRICPROPERTIESVARIANT:
			return "NOGEOMETRICPROPERTIESVARIANT";
		case STATICGEOMETRICPROPERTIESREC:
			return "STATICGEOMETRICPROPERTIESREC";
		case MANIPULATORGEOMETRICPROPERTIESREC:
			return "MANIPULATORGEOMETRICPROPERTIESREC";
		default:
			return "Unknown";
	}
}

void GeometricPropertiesVariant::copy(GeometricPropertiesVariant& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	this->type = source.getType();
	switch(this->type)
	{
		case NOGEOMETRICPROPERTIESVARIANT:
			this->noGeometricPropertiesVariant.copy(source.getNoGeometricPropertiesVariant());
			break;

		case STATICGEOMETRICPROPERTIESREC:
			this->staticGeometricPropertiesRec.copy(source.getStaticGeometricPropertiesRec());
			break;

		case MANIPULATORGEOMETRICPROPERTIESREC:
			this->manipulatorGeometricPropertiesRec.copy(source.getManipulatorGeometricPropertiesRec());
			break;

		default:
			THROW_EXCEPTION("Unknown GeometricPropertiesVariant Type: " << this->type);
	}
}

int GeometricPropertiesVariant::to(system::Buffer *dst)
{
	int result = 0;
	result += dst->pack(static_cast<uint8_t>(this->type));
	switch(this->type)
	{
		case NOGEOMETRICPROPERTIESVARIANT:
			result += dst->pack(noGeometricPropertiesVariant);
			return result;

		case STATICGEOMETRICPROPERTIESREC:
			result += dst->pack(staticGeometricPropertiesRec);
			return result;

		case MANIPULATORGEOMETRICPROPERTIESREC:
			result += dst->pack(manipulatorGeometricPropertiesRec);
			return result;

		default:
			THROW_EXCEPTION("Unknown GeometricPropertiesVariant Type: " << this->type);
	}
}

int GeometricPropertiesVariant::from(system::Buffer *src)
{
	int result = 0;

	uint8_t intValue;
	result += src->unpack(intValue);
	this->type = static_cast<GeometricPropertiesVariant::TypeEnum>(intValue);

	switch(this->type)
	{
		case NOGEOMETRICPROPERTIESVARIANT:
			result += src->unpack(noGeometricPropertiesVariant);
			return result;

		case STATICGEOMETRICPROPERTIESREC:
			result += src->unpack(staticGeometricPropertiesRec);
			return result;

		case MANIPULATORGEOMETRICPROPERTIESREC:
			result += src->unpack(manipulatorGeometricPropertiesRec);
			return result;

		default:
			THROW_EXCEPTION("Unknown GeometricPropertiesVariant Type: " << this->type);
	}
}

int GeometricPropertiesVariant::length(void)
{
	int result = 0;
	result += sizeof(uint8_t); // Type
	switch(this->type)
	{
		case NOGEOMETRICPROPERTIESVARIANT:
			result += noGeometricPropertiesVariant.length();
			return result;

		case STATICGEOMETRICPROPERTIESREC:
			result += staticGeometricPropertiesRec.length();
			return result;

		case MANIPULATORGEOMETRICPROPERTIESREC:
			result += manipulatorGeometricPropertiesRec.length();
			return result;

		default:
			THROW_EXCEPTION("Unknown GeometricPropertiesVariant Type: " << this->type);
	}
}

std::string GeometricPropertiesVariant::toXml(unsigned char level) const
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
		case NOGEOMETRICPROPERTIESVARIANT:
			oss << prefix.str() << "\t" << "<type>NOGEOMETRICPROPERTIESVARIANT</type>\n";
			oss << noGeometricPropertiesVariant.toXml(level+1);
			oss << prefix.str() << "</Variant>\n";
			return oss.str();

		case STATICGEOMETRICPROPERTIESREC:
			oss << prefix.str() << "\t" << "<type>STATICGEOMETRICPROPERTIESREC</type>\n";
			oss << staticGeometricPropertiesRec.toXml(level+1);
			oss << prefix.str() << "</Variant>\n";
			return oss.str();

		case MANIPULATORGEOMETRICPROPERTIESREC:
			oss << prefix.str() << "\t" << "<type>MANIPULATORGEOMETRICPROPERTIESREC</type>\n";
			oss << manipulatorGeometricPropertiesRec.toXml(level+1);
			oss << prefix.str() << "</Variant>\n";
			return oss.str();

		default:
			THROW_EXCEPTION("Unknown GeometricPropertiesVariant Type: " << this->type);
	}
}




NoGeometricPropertiesVariant& GeometricPropertiesVariant::getNoGeometricPropertiesVariant(void)
{
	if(this->type != NOGEOMETRICPROPERTIESVARIANT)
	{
		LOG("Misinterpretation of Variant (GeometricPropertiesVariant). Requesting object of type \"NOGEOMETRICPROPERTIESVARIANT\" while variant is of type \"" << this->typeToString() << "\"" << std::endl);
	}
	return this->noGeometricPropertiesVariant;
}

StaticGeometricPropertiesRecord& GeometricPropertiesVariant::getStaticGeometricPropertiesRec(void)
{
	if(this->type != STATICGEOMETRICPROPERTIESREC)
	{
		LOG("Misinterpretation of Variant (GeometricPropertiesVariant). Requesting object of type \"STATICGEOMETRICPROPERTIESREC\" while variant is of type \"" << this->typeToString() << "\"" << std::endl);
	}
	return this->staticGeometricPropertiesRec;
}

ManipulatorGeometricPropertiesRecord& GeometricPropertiesVariant::getManipulatorGeometricPropertiesRec(void)
{
	if(this->type != MANIPULATORGEOMETRICPROPERTIESREC)
	{
		LOG("Misinterpretation of Variant (GeometricPropertiesVariant). Requesting object of type \"MANIPULATORGEOMETRICPROPERTIESREC\" while variant is of type \"" << this->typeToString() << "\"" << std::endl);
	}
	return this->manipulatorGeometricPropertiesRec;
}

} // namespace environment
} // namespace openjaus

