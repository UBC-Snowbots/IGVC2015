

/**
\file FirstJointParametersVariant.h

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
#include "openjaus/manipulator/Triggers/Fields/FirstJointParametersVariant.h"

namespace openjaus
{
namespace manipulator
{

FirstJointParametersVariant::FirstJointParametersVariant() :
	type(),
	revoluteJoint1OffsetRec(),
	prismaticJoint1AngleRec()
{
}

FirstJointParametersVariant::~FirstJointParametersVariant()
{

}

void FirstJointParametersVariant::setType(FirstJointParametersVariant::TypeEnum type)
{
	this->type = type;
}

FirstJointParametersVariant::TypeEnum FirstJointParametersVariant::getType(void)
{
	return this->type;
}

std::string FirstJointParametersVariant::typeToString(void)
{
	switch(this->type)
	{
		case REVOLUTEJOINT1OFFSETREC:
			return "REVOLUTEJOINT1OFFSETREC";
		case PRISMATICJOINT1ANGLEREC:
			return "PRISMATICJOINT1ANGLEREC";
		default:
			return "Unknown";
	}
}

void FirstJointParametersVariant::copy(FirstJointParametersVariant& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	this->type = source.getType();
	switch(this->type)
	{
		case REVOLUTEJOINT1OFFSETREC:
			this->revoluteJoint1OffsetRec.copy(source.getRevoluteJoint1OffsetRec());
			break;

		case PRISMATICJOINT1ANGLEREC:
			this->prismaticJoint1AngleRec.copy(source.getPrismaticJoint1AngleRec());
			break;

		default:
			THROW_EXCEPTION("Unknown FirstJointParametersVariant Type: " << this->type);
	}
}

int FirstJointParametersVariant::to(system::Buffer *dst)
{
	int result = 0;
	result += dst->pack(static_cast<uint8_t>(this->type));
	switch(this->type)
	{
		case REVOLUTEJOINT1OFFSETREC:
			result += dst->pack(revoluteJoint1OffsetRec);
			return result;

		case PRISMATICJOINT1ANGLEREC:
			result += dst->pack(prismaticJoint1AngleRec);
			return result;

		default:
			THROW_EXCEPTION("Unknown FirstJointParametersVariant Type: " << this->type);
	}
}

int FirstJointParametersVariant::from(system::Buffer *src)
{
	int result = 0;

	uint8_t intValue;
	result += src->unpack(intValue);
	this->type = static_cast<FirstJointParametersVariant::TypeEnum>(intValue);

	switch(this->type)
	{
		case REVOLUTEJOINT1OFFSETREC:
			result += src->unpack(revoluteJoint1OffsetRec);
			return result;

		case PRISMATICJOINT1ANGLEREC:
			result += src->unpack(prismaticJoint1AngleRec);
			return result;

		default:
			THROW_EXCEPTION("Unknown FirstJointParametersVariant Type: " << this->type);
	}
}

int FirstJointParametersVariant::length(void)
{
	int result = 0;
	result += sizeof(uint8_t); // Type
	switch(this->type)
	{
		case REVOLUTEJOINT1OFFSETREC:
			result += revoluteJoint1OffsetRec.length();
			return result;

		case PRISMATICJOINT1ANGLEREC:
			result += prismaticJoint1AngleRec.length();
			return result;

		default:
			THROW_EXCEPTION("Unknown FirstJointParametersVariant Type: " << this->type);
	}
}

std::string FirstJointParametersVariant::toXml(unsigned char level) const
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
		case REVOLUTEJOINT1OFFSETREC:
			oss << prefix.str() << "\t" << "<type>REVOLUTEJOINT1OFFSETREC</type>\n";
			oss << revoluteJoint1OffsetRec.toXml(level+1);
			oss << prefix.str() << "</Variant>\n";
			return oss.str();

		case PRISMATICJOINT1ANGLEREC:
			oss << prefix.str() << "\t" << "<type>PRISMATICJOINT1ANGLEREC</type>\n";
			oss << prismaticJoint1AngleRec.toXml(level+1);
			oss << prefix.str() << "</Variant>\n";
			return oss.str();

		default:
			THROW_EXCEPTION("Unknown FirstJointParametersVariant Type: " << this->type);
	}
}




RevoluteJoint1OffsetRecord& FirstJointParametersVariant::getRevoluteJoint1OffsetRec(void)
{
	if(this->type != REVOLUTEJOINT1OFFSETREC)
	{
		LOG("Misinterpretation of Variant (FirstJointParametersVariant). Requesting object of type \"REVOLUTEJOINT1OFFSETREC\" while variant is of type \"" << this->typeToString() << "\"" << std::endl);
	}
	return this->revoluteJoint1OffsetRec;
}

PrismaticJoint1AngleRecord& FirstJointParametersVariant::getPrismaticJoint1AngleRec(void)
{
	if(this->type != PRISMATICJOINT1ANGLEREC)
	{
		LOG("Misinterpretation of Variant (FirstJointParametersVariant). Requesting object of type \"PRISMATICJOINT1ANGLEREC\" while variant is of type \"" << this->typeToString() << "\"" << std::endl);
	}
	return this->prismaticJoint1AngleRec;
}

} // namespace manipulator
} // namespace openjaus

