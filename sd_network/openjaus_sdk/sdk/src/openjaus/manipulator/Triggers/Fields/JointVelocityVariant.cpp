

/**
\file JointVelocityVariant.h

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
#include "openjaus/manipulator/Triggers/Fields/JointVelocityVariant.h"

namespace openjaus
{
namespace manipulator
{

JointVelocityVariant::JointVelocityVariant() :
	type(),
	option0_rps(),
	option1_mps()
{
}

JointVelocityVariant::~JointVelocityVariant()
{

}

void JointVelocityVariant::setType(JointVelocityVariant::TypeEnum type)
{
	this->type = type;
}

JointVelocityVariant::TypeEnum JointVelocityVariant::getType(void)
{
	return this->type;
}

std::string JointVelocityVariant::typeToString(void)
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

void JointVelocityVariant::copy(JointVelocityVariant& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	this->type = source.getType();
	switch(this->type)
	{
		case OPTION0:
			this->option0_rps.setName("Option0");
			this->option0_rps.setOptional(false);
			this->option0_rps.setValue(source.getOption0_rps());
			break;

		case OPTION1:
			this->option1_mps.setName("Option1");
			this->option1_mps.setOptional(false);
			this->option1_mps.setValue(source.getOption1_mps());
			break;

		default:
			THROW_EXCEPTION("Unknown JointVelocityVariant Type: " << this->type);
	}
}

int JointVelocityVariant::to(system::Buffer *dst)
{
	int result = 0;
	result += dst->pack(static_cast<uint8_t>(this->type));
	switch(this->type)
	{
		case OPTION0:
			result += dst->pack(option0_rps);
			return result;

		case OPTION1:
			result += dst->pack(option1_mps);
			return result;

		default:
			THROW_EXCEPTION("Unknown JointVelocityVariant Type: " << this->type);
	}
}

int JointVelocityVariant::from(system::Buffer *src)
{
	int result = 0;

	uint8_t intValue;
	result += src->unpack(intValue);
	this->type = static_cast<JointVelocityVariant::TypeEnum>(intValue);

	switch(this->type)
	{
		case OPTION0:
			result += src->unpack(option0_rps);
			return result;

		case OPTION1:
			result += src->unpack(option1_mps);
			return result;

		default:
			THROW_EXCEPTION("Unknown JointVelocityVariant Type: " << this->type);
	}
}

int JointVelocityVariant::length(void)
{
	int result = 0;
	result += sizeof(uint8_t); // Type
	switch(this->type)
	{
		case OPTION0:
			result += option0_rps.length();
			return result;

		case OPTION1:
			result += option1_mps.length();
			return result;

		default:
			THROW_EXCEPTION("Unknown JointVelocityVariant Type: " << this->type);
	}
}

std::string JointVelocityVariant::toXml(unsigned char level) const
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
			oss << option0_rps.toXml(level+1);
			oss << prefix.str() << "</Variant>\n";
			return oss.str();

		case OPTION1:
			oss << prefix.str() << "\t" << "<type>OPTION1</type>\n";
			oss << option1_mps.toXml(level+1);
			oss << prefix.str() << "</Variant>\n";
			return oss.str();

		default:
			THROW_EXCEPTION("Unknown JointVelocityVariant Type: " << this->type);
	}
}




double JointVelocityVariant::getOption0_rps(void)
{
	if(this->type != OPTION0)
	{
		LOG("Misinterpretation of Variant (JointVelocityVariant). Requesting object of type \"OPTION0_RPS\" while variant is of type \"" << this->typeToString() << "\"" << std::endl);
	}
	return this->option0_rps.getValue();
}

void JointVelocityVariant::setOption0_rps(double value)
{
	this->option0_rps.setValue(value);
}

double JointVelocityVariant::getOption1_mps(void)
{
	if(this->type != OPTION1)
	{
		LOG("Misinterpretation of Variant (JointVelocityVariant). Requesting object of type \"OPTION1_MPS\" while variant is of type \"" << this->typeToString() << "\"" << std::endl);
	}
	return this->option1_mps.getValue();
}

void JointVelocityVariant::setOption1_mps(double value)
{
	this->option1_mps.setValue(value);
}

} // namespace manipulator
} // namespace openjaus

