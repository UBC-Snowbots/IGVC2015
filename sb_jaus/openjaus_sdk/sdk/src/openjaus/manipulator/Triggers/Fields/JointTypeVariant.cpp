

/**
\file JointTypeVariant.h

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
#include "openjaus/manipulator/Triggers/Fields/JointTypeVariant.h"

namespace openjaus
{
namespace manipulator
{

JointTypeVariant::JointTypeVariant() :
	type(),
	revoluteJointMotionProfileRec(),
	prismaticJointMotionProfileRec()
{
}

JointTypeVariant::~JointTypeVariant()
{

}

void JointTypeVariant::setType(JointTypeVariant::TypeEnum type)
{
	this->type = type;
}

JointTypeVariant::TypeEnum JointTypeVariant::getType(void)
{
	return this->type;
}

std::string JointTypeVariant::typeToString(void)
{
	switch(this->type)
	{
		case REVOLUTEJOINTMOTIONPROFILEREC:
			return "REVOLUTEJOINTMOTIONPROFILEREC";
		case PRISMATICJOINTMOTIONPROFILEREC:
			return "PRISMATICJOINTMOTIONPROFILEREC";
		default:
			return "Unknown";
	}
}

void JointTypeVariant::copy(JointTypeVariant& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	this->type = source.getType();
	switch(this->type)
	{
		case REVOLUTEJOINTMOTIONPROFILEREC:
			this->revoluteJointMotionProfileRec.copy(source.getRevoluteJointMotionProfileRec());
			break;

		case PRISMATICJOINTMOTIONPROFILEREC:
			this->prismaticJointMotionProfileRec.copy(source.getPrismaticJointMotionProfileRec());
			break;

		default:
			THROW_EXCEPTION("Unknown JointTypeVariant Type: " << this->type);
	}
}

int JointTypeVariant::to(system::Buffer *dst)
{
	int result = 0;
	result += dst->pack(static_cast<uint8_t>(this->type));
	switch(this->type)
	{
		case REVOLUTEJOINTMOTIONPROFILEREC:
			result += dst->pack(revoluteJointMotionProfileRec);
			return result;

		case PRISMATICJOINTMOTIONPROFILEREC:
			result += dst->pack(prismaticJointMotionProfileRec);
			return result;

		default:
			THROW_EXCEPTION("Unknown JointTypeVariant Type: " << this->type);
	}
}

int JointTypeVariant::from(system::Buffer *src)
{
	int result = 0;

	uint8_t intValue;
	result += src->unpack(intValue);
	this->type = static_cast<JointTypeVariant::TypeEnum>(intValue);

	switch(this->type)
	{
		case REVOLUTEJOINTMOTIONPROFILEREC:
			result += src->unpack(revoluteJointMotionProfileRec);
			return result;

		case PRISMATICJOINTMOTIONPROFILEREC:
			result += src->unpack(prismaticJointMotionProfileRec);
			return result;

		default:
			THROW_EXCEPTION("Unknown JointTypeVariant Type: " << this->type);
	}
}

int JointTypeVariant::length(void)
{
	int result = 0;
	result += sizeof(uint8_t); // Type
	switch(this->type)
	{
		case REVOLUTEJOINTMOTIONPROFILEREC:
			result += revoluteJointMotionProfileRec.length();
			return result;

		case PRISMATICJOINTMOTIONPROFILEREC:
			result += prismaticJointMotionProfileRec.length();
			return result;

		default:
			THROW_EXCEPTION("Unknown JointTypeVariant Type: " << this->type);
	}
}

std::string JointTypeVariant::toXml(unsigned char level) const
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
		case REVOLUTEJOINTMOTIONPROFILEREC:
			oss << prefix.str() << "\t" << "<type>REVOLUTEJOINTMOTIONPROFILEREC</type>\n";
			oss << revoluteJointMotionProfileRec.toXml(level+1);
			oss << prefix.str() << "</Variant>\n";
			return oss.str();

		case PRISMATICJOINTMOTIONPROFILEREC:
			oss << prefix.str() << "\t" << "<type>PRISMATICJOINTMOTIONPROFILEREC</type>\n";
			oss << prismaticJointMotionProfileRec.toXml(level+1);
			oss << prefix.str() << "</Variant>\n";
			return oss.str();

		default:
			THROW_EXCEPTION("Unknown JointTypeVariant Type: " << this->type);
	}
}




RevoluteJointMotionProfileRecord& JointTypeVariant::getRevoluteJointMotionProfileRec(void)
{
	if(this->type != REVOLUTEJOINTMOTIONPROFILEREC)
	{
		LOG("Misinterpretation of Variant (JointTypeVariant). Requesting object of type \"REVOLUTEJOINTMOTIONPROFILEREC\" while variant is of type \"" << this->typeToString() << "\"" << std::endl);
	}
	return this->revoluteJointMotionProfileRec;
}

PrismaticJointMotionProfileRecord& JointTypeVariant::getPrismaticJointMotionProfileRec(void)
{
	if(this->type != PRISMATICJOINTMOTIONPROFILEREC)
	{
		LOG("Misinterpretation of Variant (JointTypeVariant). Requesting object of type \"PRISMATICJOINTMOTIONPROFILEREC\" while variant is of type \"" << this->typeToString() << "\"" << std::endl);
	}
	return this->prismaticJointMotionProfileRec;
}

} // namespace manipulator
} // namespace openjaus

