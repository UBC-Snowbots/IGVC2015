

/**
\file JointSpecificationsVariant.h

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
#include "openjaus/manipulator/Triggers/Fields/JointSpecificationsVariant.h"

namespace openjaus
{
namespace manipulator
{

JointSpecificationsVariant::JointSpecificationsVariant() :
	type(),
	revoluteJointSpecificationRec(),
	prismaticJointSpecificationRec()
{
}

JointSpecificationsVariant::~JointSpecificationsVariant()
{

}

void JointSpecificationsVariant::setType(JointSpecificationsVariant::TypeEnum type)
{
	this->type = type;
}

JointSpecificationsVariant::TypeEnum JointSpecificationsVariant::getType(void)
{
	return this->type;
}

std::string JointSpecificationsVariant::typeToString(void)
{
	switch(this->type)
	{
		case REVOLUTEJOINTSPECIFICATIONREC:
			return "REVOLUTEJOINTSPECIFICATIONREC";
		case PRISMATICJOINTSPECIFICATIONREC:
			return "PRISMATICJOINTSPECIFICATIONREC";
		default:
			return "Unknown";
	}
}

void JointSpecificationsVariant::copy(JointSpecificationsVariant& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	this->type = source.getType();
	switch(this->type)
	{
		case REVOLUTEJOINTSPECIFICATIONREC:
			this->revoluteJointSpecificationRec.copy(source.getRevoluteJointSpecificationRec());
			break;

		case PRISMATICJOINTSPECIFICATIONREC:
			this->prismaticJointSpecificationRec.copy(source.getPrismaticJointSpecificationRec());
			break;

		default:
			THROW_EXCEPTION("Unknown JointSpecificationsVariant Type: " << this->type);
	}
}

int JointSpecificationsVariant::to(system::Buffer *dst)
{
	int result = 0;
	result += dst->pack(static_cast<uint8_t>(this->type));
	switch(this->type)
	{
		case REVOLUTEJOINTSPECIFICATIONREC:
			result += dst->pack(revoluteJointSpecificationRec);
			return result;

		case PRISMATICJOINTSPECIFICATIONREC:
			result += dst->pack(prismaticJointSpecificationRec);
			return result;

		default:
			THROW_EXCEPTION("Unknown JointSpecificationsVariant Type: " << this->type);
	}
}

int JointSpecificationsVariant::from(system::Buffer *src)
{
	int result = 0;

	uint8_t intValue;
	result += src->unpack(intValue);
	this->type = static_cast<JointSpecificationsVariant::TypeEnum>(intValue);

	switch(this->type)
	{
		case REVOLUTEJOINTSPECIFICATIONREC:
			result += src->unpack(revoluteJointSpecificationRec);
			return result;

		case PRISMATICJOINTSPECIFICATIONREC:
			result += src->unpack(prismaticJointSpecificationRec);
			return result;

		default:
			THROW_EXCEPTION("Unknown JointSpecificationsVariant Type: " << this->type);
	}
}

int JointSpecificationsVariant::length(void)
{
	int result = 0;
	result += sizeof(uint8_t); // Type
	switch(this->type)
	{
		case REVOLUTEJOINTSPECIFICATIONREC:
			result += revoluteJointSpecificationRec.length();
			return result;

		case PRISMATICJOINTSPECIFICATIONREC:
			result += prismaticJointSpecificationRec.length();
			return result;

		default:
			THROW_EXCEPTION("Unknown JointSpecificationsVariant Type: " << this->type);
	}
}

std::string JointSpecificationsVariant::toXml(unsigned char level) const
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
		case REVOLUTEJOINTSPECIFICATIONREC:
			oss << prefix.str() << "\t" << "<type>REVOLUTEJOINTSPECIFICATIONREC</type>\n";
			oss << revoluteJointSpecificationRec.toXml(level+1);
			oss << prefix.str() << "</Variant>\n";
			return oss.str();

		case PRISMATICJOINTSPECIFICATIONREC:
			oss << prefix.str() << "\t" << "<type>PRISMATICJOINTSPECIFICATIONREC</type>\n";
			oss << prismaticJointSpecificationRec.toXml(level+1);
			oss << prefix.str() << "</Variant>\n";
			return oss.str();

		default:
			THROW_EXCEPTION("Unknown JointSpecificationsVariant Type: " << this->type);
	}
}




RevoluteJointSpecificationRecord& JointSpecificationsVariant::getRevoluteJointSpecificationRec(void)
{
	if(this->type != REVOLUTEJOINTSPECIFICATIONREC)
	{
		LOG("Misinterpretation of Variant (JointSpecificationsVariant). Requesting object of type \"REVOLUTEJOINTSPECIFICATIONREC\" while variant is of type \"" << this->typeToString() << "\"" << std::endl);
	}
	return this->revoluteJointSpecificationRec;
}

PrismaticJointSpecificationRecord& JointSpecificationsVariant::getPrismaticJointSpecificationRec(void)
{
	if(this->type != PRISMATICJOINTSPECIFICATIONREC)
	{
		LOG("Misinterpretation of Variant (JointSpecificationsVariant). Requesting object of type \"PRISMATICJOINTSPECIFICATIONREC\" while variant is of type \"" << this->typeToString() << "\"" << std::endl);
	}
	return this->prismaticJointSpecificationRec;
}

} // namespace manipulator
} // namespace openjaus

