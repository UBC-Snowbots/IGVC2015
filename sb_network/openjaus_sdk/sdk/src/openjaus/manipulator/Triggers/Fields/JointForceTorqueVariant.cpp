

/**
\file JointForceTorqueVariant.h

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
#include "openjaus/manipulator/Triggers/Fields/JointForceTorqueVariant.h"

namespace openjaus
{
namespace manipulator
{

JointForceTorqueVariant::JointForceTorqueVariant() :
	type(),
	option0(),
	option1()
{
}

JointForceTorqueVariant::~JointForceTorqueVariant()
{

}

void JointForceTorqueVariant::setType(JointForceTorqueVariant::TypeEnum type)
{
	this->type = type;
}

JointForceTorqueVariant::TypeEnum JointForceTorqueVariant::getType(void)
{
	return this->type;
}

std::string JointForceTorqueVariant::typeToString(void)
{
	switch(this->type)
	{
		case OPTION0:
			return "OPTION0";
		case OPTION1:
			return "OPTION1";
		default:
			return "Unknown";
	}
}

void JointForceTorqueVariant::copy(JointForceTorqueVariant& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	this->type = source.getType();
	switch(this->type)
	{
		case OPTION0:
			this->option0.setName("Option0");
			this->option0.setOptional(false);
			this->option0.setValue(source.getOption0());
			break;

		case OPTION1:
			this->option1.setName("Option1");
			this->option1.setOptional(false);
			this->option1.setValue(source.getOption1());
			break;

		default:
			THROW_EXCEPTION("Unknown JointForceTorqueVariant Type: " << this->type);
	}
}

int JointForceTorqueVariant::to(system::Buffer *dst)
{
	int result = 0;
	result += dst->pack(static_cast<uint8_t>(this->type));
	switch(this->type)
	{
		case OPTION0:
			result += dst->pack(option0);
			return result;

		case OPTION1:
			result += dst->pack(option1);
			return result;

		default:
			THROW_EXCEPTION("Unknown JointForceTorqueVariant Type: " << this->type);
	}
}

int JointForceTorqueVariant::from(system::Buffer *src)
{
	int result = 0;

	uint8_t intValue;
	result += src->unpack(intValue);
	this->type = static_cast<JointForceTorqueVariant::TypeEnum>(intValue);

	switch(this->type)
	{
		case OPTION0:
			result += src->unpack(option0);
			return result;

		case OPTION1:
			result += src->unpack(option1);
			return result;

		default:
			THROW_EXCEPTION("Unknown JointForceTorqueVariant Type: " << this->type);
	}
}

int JointForceTorqueVariant::length(void)
{
	int result = 0;
	result += sizeof(uint8_t); // Type
	switch(this->type)
	{
		case OPTION0:
			result += option0.length();
			return result;

		case OPTION1:
			result += option1.length();
			return result;

		default:
			THROW_EXCEPTION("Unknown JointForceTorqueVariant Type: " << this->type);
	}
}

std::string JointForceTorqueVariant::toXml(unsigned char level) const
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
		case OPTION0:
			oss << prefix.str() << "\t" << "<type>OPTION0</type>\n";
			oss << option0.toXml(level+1);
			oss << prefix.str() << "</Variant>\n";
			return oss.str();

		case OPTION1:
			oss << prefix.str() << "\t" << "<type>OPTION1</type>\n";
			oss << option1.toXml(level+1);
			oss << prefix.str() << "</Variant>\n";
			return oss.str();

		default:
			THROW_EXCEPTION("Unknown JointForceTorqueVariant Type: " << this->type);
	}
}




double JointForceTorqueVariant::getOption0(void)
{
	if(this->type != OPTION0)
	{
		LOG("Misinterpretation of Variant (JointForceTorqueVariant). Requesting object of type \"OPTION0\" while variant is of type \"" << this->typeToString() << "\"" << std::endl);
	}
	return this->option0.getValue();
}

void JointForceTorqueVariant::setOption0(double value)
{
	this->option0.setValue(value);
}

double JointForceTorqueVariant::getOption1(void)
{
	if(this->type != OPTION1)
	{
		LOG("Misinterpretation of Variant (JointForceTorqueVariant). Requesting object of type \"OPTION1\" while variant is of type \"" << this->typeToString() << "\"" << std::endl);
	}
	return this->option1.getValue();
}

void JointForceTorqueVariant::setOption1(double value)
{
	this->option1.setValue(value);
}

} // namespace manipulator
} // namespace openjaus

