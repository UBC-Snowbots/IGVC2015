

/**
\file ConfirmSensorConfigurationVariant.h

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
#include "openjaus/environment/Triggers/Fields/ConfirmSensorConfigurationVariant.h"

namespace openjaus
{
namespace environment
{

ConfirmSensorConfigurationVariant::ConfirmSensorConfigurationVariant() :
	type(),
	sensorIdRec(),
	rangeSensorErrorRec(),
	visualSensorErrorRec(),
	digitalVideoSensorErrorRec(),
	analogVideoSensorErrorRec(),
	stillImageSensorErrorRec()
{
}

ConfirmSensorConfigurationVariant::~ConfirmSensorConfigurationVariant()
{

}

void ConfirmSensorConfigurationVariant::setType(ConfirmSensorConfigurationVariant::TypeEnum type)
{
	this->type = type;
}

ConfirmSensorConfigurationVariant::TypeEnum ConfirmSensorConfigurationVariant::getType(void)
{
	return this->type;
}

std::string ConfirmSensorConfigurationVariant::typeToString(void)
{
	switch(this->type)
	{
		case SENSORIDREC:
			return "SENSORIDREC";
		case RANGESENSORERRORREC:
			return "RANGESENSORERRORREC";
		case VISUALSENSORERRORREC:
			return "VISUALSENSORERRORREC";
		case DIGITALVIDEOSENSORERRORREC:
			return "DIGITALVIDEOSENSORERRORREC";
		case ANALOGVIDEOSENSORERRORREC:
			return "ANALOGVIDEOSENSORERRORREC";
		case STILLIMAGESENSORERRORREC:
			return "STILLIMAGESENSORERRORREC";
		default:
			return "Unknown";
	}
}

void ConfirmSensorConfigurationVariant::copy(ConfirmSensorConfigurationVariant& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	this->type = source.getType();
	switch(this->type)
	{
		case SENSORIDREC:
			this->sensorIdRec.copy(source.getSensorIdRec());
			break;

		case RANGESENSORERRORREC:
			this->rangeSensorErrorRec.copy(source.getRangeSensorErrorRec());
			break;

		case VISUALSENSORERRORREC:
			this->visualSensorErrorRec.copy(source.getVisualSensorErrorRec());
			break;

		case DIGITALVIDEOSENSORERRORREC:
			this->digitalVideoSensorErrorRec.copy(source.getDigitalVideoSensorErrorRec());
			break;

		case ANALOGVIDEOSENSORERRORREC:
			this->analogVideoSensorErrorRec.copy(source.getAnalogVideoSensorErrorRec());
			break;

		case STILLIMAGESENSORERRORREC:
			this->stillImageSensorErrorRec.copy(source.getStillImageSensorErrorRec());
			break;

		default:
			THROW_EXCEPTION("Unknown ConfirmSensorConfigurationVariant Type: " << this->type);
	}
}

int ConfirmSensorConfigurationVariant::to(system::Buffer *dst)
{
	int result = 0;
	result += dst->pack(static_cast<uint8_t>(this->type));
	switch(this->type)
	{
		case SENSORIDREC:
			result += dst->pack(sensorIdRec);
			return result;

		case RANGESENSORERRORREC:
			result += dst->pack(rangeSensorErrorRec);
			return result;

		case VISUALSENSORERRORREC:
			result += dst->pack(visualSensorErrorRec);
			return result;

		case DIGITALVIDEOSENSORERRORREC:
			result += dst->pack(digitalVideoSensorErrorRec);
			return result;

		case ANALOGVIDEOSENSORERRORREC:
			result += dst->pack(analogVideoSensorErrorRec);
			return result;

		case STILLIMAGESENSORERRORREC:
			result += dst->pack(stillImageSensorErrorRec);
			return result;

		default:
			THROW_EXCEPTION("Unknown ConfirmSensorConfigurationVariant Type: " << this->type);
	}
}

int ConfirmSensorConfigurationVariant::from(system::Buffer *src)
{
	int result = 0;

	uint8_t intValue;
	result += src->unpack(intValue);
	this->type = static_cast<ConfirmSensorConfigurationVariant::TypeEnum>(intValue);

	switch(this->type)
	{
		case SENSORIDREC:
			result += src->unpack(sensorIdRec);
			return result;

		case RANGESENSORERRORREC:
			result += src->unpack(rangeSensorErrorRec);
			return result;

		case VISUALSENSORERRORREC:
			result += src->unpack(visualSensorErrorRec);
			return result;

		case DIGITALVIDEOSENSORERRORREC:
			result += src->unpack(digitalVideoSensorErrorRec);
			return result;

		case ANALOGVIDEOSENSORERRORREC:
			result += src->unpack(analogVideoSensorErrorRec);
			return result;

		case STILLIMAGESENSORERRORREC:
			result += src->unpack(stillImageSensorErrorRec);
			return result;

		default:
			THROW_EXCEPTION("Unknown ConfirmSensorConfigurationVariant Type: " << this->type);
	}
}

int ConfirmSensorConfigurationVariant::length(void)
{
	int result = 0;
	result += sizeof(uint8_t); // Type
	switch(this->type)
	{
		case SENSORIDREC:
			result += sensorIdRec.length();
			return result;

		case RANGESENSORERRORREC:
			result += rangeSensorErrorRec.length();
			return result;

		case VISUALSENSORERRORREC:
			result += visualSensorErrorRec.length();
			return result;

		case DIGITALVIDEOSENSORERRORREC:
			result += digitalVideoSensorErrorRec.length();
			return result;

		case ANALOGVIDEOSENSORERRORREC:
			result += analogVideoSensorErrorRec.length();
			return result;

		case STILLIMAGESENSORERRORREC:
			result += stillImageSensorErrorRec.length();
			return result;

		default:
			THROW_EXCEPTION("Unknown ConfirmSensorConfigurationVariant Type: " << this->type);
	}
}

std::string ConfirmSensorConfigurationVariant::toXml(unsigned char level) const
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
		case SENSORIDREC:
			oss << prefix.str() << "\t" << "<type>SENSORIDREC</type>\n";
			oss << sensorIdRec.toXml(level+1);
			oss << prefix.str() << "</Variant>\n";
			return oss.str();

		case RANGESENSORERRORREC:
			oss << prefix.str() << "\t" << "<type>RANGESENSORERRORREC</type>\n";
			oss << rangeSensorErrorRec.toXml(level+1);
			oss << prefix.str() << "</Variant>\n";
			return oss.str();

		case VISUALSENSORERRORREC:
			oss << prefix.str() << "\t" << "<type>VISUALSENSORERRORREC</type>\n";
			oss << visualSensorErrorRec.toXml(level+1);
			oss << prefix.str() << "</Variant>\n";
			return oss.str();

		case DIGITALVIDEOSENSORERRORREC:
			oss << prefix.str() << "\t" << "<type>DIGITALVIDEOSENSORERRORREC</type>\n";
			oss << digitalVideoSensorErrorRec.toXml(level+1);
			oss << prefix.str() << "</Variant>\n";
			return oss.str();

		case ANALOGVIDEOSENSORERRORREC:
			oss << prefix.str() << "\t" << "<type>ANALOGVIDEOSENSORERRORREC</type>\n";
			oss << analogVideoSensorErrorRec.toXml(level+1);
			oss << prefix.str() << "</Variant>\n";
			return oss.str();

		case STILLIMAGESENSORERRORREC:
			oss << prefix.str() << "\t" << "<type>STILLIMAGESENSORERRORREC</type>\n";
			oss << stillImageSensorErrorRec.toXml(level+1);
			oss << prefix.str() << "</Variant>\n";
			return oss.str();

		default:
			THROW_EXCEPTION("Unknown ConfirmSensorConfigurationVariant Type: " << this->type);
	}
}




SensorIdRecord& ConfirmSensorConfigurationVariant::getSensorIdRec(void)
{
	if(this->type != SENSORIDREC)
	{
		LOG("Misinterpretation of Variant (ConfirmSensorConfigurationVariant). Requesting object of type \"SENSORIDREC\" while variant is of type \"" << this->typeToString() << "\"" << std::endl);
	}
	return this->sensorIdRec;
}

RangeSensorErrorRecord& ConfirmSensorConfigurationVariant::getRangeSensorErrorRec(void)
{
	if(this->type != RANGESENSORERRORREC)
	{
		LOG("Misinterpretation of Variant (ConfirmSensorConfigurationVariant). Requesting object of type \"RANGESENSORERRORREC\" while variant is of type \"" << this->typeToString() << "\"" << std::endl);
	}
	return this->rangeSensorErrorRec;
}

VisualSensorErrorRecord& ConfirmSensorConfigurationVariant::getVisualSensorErrorRec(void)
{
	if(this->type != VISUALSENSORERRORREC)
	{
		LOG("Misinterpretation of Variant (ConfirmSensorConfigurationVariant). Requesting object of type \"VISUALSENSORERRORREC\" while variant is of type \"" << this->typeToString() << "\"" << std::endl);
	}
	return this->visualSensorErrorRec;
}

DigitalVideoSensorErrorRecord& ConfirmSensorConfigurationVariant::getDigitalVideoSensorErrorRec(void)
{
	if(this->type != DIGITALVIDEOSENSORERRORREC)
	{
		LOG("Misinterpretation of Variant (ConfirmSensorConfigurationVariant). Requesting object of type \"DIGITALVIDEOSENSORERRORREC\" while variant is of type \"" << this->typeToString() << "\"" << std::endl);
	}
	return this->digitalVideoSensorErrorRec;
}

AnalogVideoSensorErrorRecord& ConfirmSensorConfigurationVariant::getAnalogVideoSensorErrorRec(void)
{
	if(this->type != ANALOGVIDEOSENSORERRORREC)
	{
		LOG("Misinterpretation of Variant (ConfirmSensorConfigurationVariant). Requesting object of type \"ANALOGVIDEOSENSORERRORREC\" while variant is of type \"" << this->typeToString() << "\"" << std::endl);
	}
	return this->analogVideoSensorErrorRec;
}

StillImageSensorErrorRecord& ConfirmSensorConfigurationVariant::getStillImageSensorErrorRec(void)
{
	if(this->type != STILLIMAGESENSORERRORREC)
	{
		LOG("Misinterpretation of Variant (ConfirmSensorConfigurationVariant). Requesting object of type \"STILLIMAGESENSORERRORREC\" while variant is of type \"" << this->typeToString() << "\"" << std::endl);
	}
	return this->stillImageSensorErrorRec;
}

} // namespace environment
} // namespace openjaus

