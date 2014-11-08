/**
\file VisualSensorCapabilitiesRecord.h

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
#include "openjaus/environment/Triggers/Fields/VisualSensorCapabilitiesRecord.h"

namespace openjaus
{
namespace environment
{

VisualSensorCapabilitiesRecord::VisualSensorCapabilitiesRecord():
	sensorID(),
	sensorName(),
	supportedStates(),
	zoomModes(),
	focusModes(),
	whiteBalance(),
	imagingModes(),
	exposureModes(),
	meteringModes(),
	minimumShutterSpeed_sec(),
	maximumShutterSpeed_sec(),
	minimumAperture(),
	maximumAperture(),
	minimumFocalLength_m(),
	maximumFocalLength_m(),
	lightSensitivityLevels(),
	imageStabilization()
{
	this->presenceVector = 0;

	fields.push_back(&sensorID);
	sensorID.setName("SensorID");
	sensorID.setOptional(false);
	sensorID.setValue(0);

	fields.push_back(&sensorName);
	sensorName.setName("SensorName");
	sensorName.setOptional(false);
	sensorName.setSizeType(model::fields::UNSIGNED_BYTE);

	fields.push_back(&supportedStates);
	supportedStates.setName("SupportedStates");
	supportedStates.setOptional(true);
	// Nothing

	fields.push_back(&zoomModes);
	zoomModes.setName("ZoomModes");
	zoomModes.setOptional(true);
	// Nothing

	fields.push_back(&focusModes);
	focusModes.setName("FocusModes");
	focusModes.setOptional(true);
	// Nothing

	fields.push_back(&whiteBalance);
	whiteBalance.setName("WhiteBalance");
	whiteBalance.setOptional(true);
	// Nothing

	fields.push_back(&imagingModes);
	imagingModes.setName("ImagingModes");
	imagingModes.setOptional(true);
	// Nothing

	fields.push_back(&exposureModes);
	exposureModes.setName("ExposureModes");
	exposureModes.setOptional(true);
	// Nothing

	fields.push_back(&meteringModes);
	meteringModes.setName("MeteringModes");
	meteringModes.setOptional(true);
	// Nothing

	fields.push_back(&minimumShutterSpeed_sec);
	minimumShutterSpeed_sec.setName("MinimumShutterSpeed");
	minimumShutterSpeed_sec.setOptional(true);
	// Nothing to init

	fields.push_back(&maximumShutterSpeed_sec);
	maximumShutterSpeed_sec.setName("MaximumShutterSpeed");
	maximumShutterSpeed_sec.setOptional(true);
	// Nothing to init

	fields.push_back(&minimumAperture);
	minimumAperture.setName("MinimumAperture");
	minimumAperture.setOptional(true);
	// Nothing to init

	fields.push_back(&maximumAperture);
	maximumAperture.setName("MaximumAperture");
	maximumAperture.setOptional(true);
	// Nothing to init

	fields.push_back(&minimumFocalLength_m);
	minimumFocalLength_m.setName("MinimumFocalLength");
	minimumFocalLength_m.setOptional(true);
	// Nothing to init

	fields.push_back(&maximumFocalLength_m);
	maximumFocalLength_m.setName("MaximumFocalLength");
	maximumFocalLength_m.setOptional(true);
	// Nothing to init

	fields.push_back(&lightSensitivityLevels);
	lightSensitivityLevels.setName("LightSensitivityLevels");
	lightSensitivityLevels.setOptional(true);
	// Nothing

	fields.push_back(&imageStabilization);
	imageStabilization.setName("ImageStabilization");
	imageStabilization.setOptional(true);
	// Nothing to init

}

VisualSensorCapabilitiesRecord::VisualSensorCapabilitiesRecord(const VisualSensorCapabilitiesRecord &source)
{
	this->copy(const_cast<VisualSensorCapabilitiesRecord&>(source));
}

VisualSensorCapabilitiesRecord::~VisualSensorCapabilitiesRecord()
{

}


uint16_t VisualSensorCapabilitiesRecord::getSensorID(void)
{
	return this->sensorID.getValue();
}

void VisualSensorCapabilitiesRecord::setSensorID(uint16_t value)
{
	this->sensorID.setValue(value);
}

std::string VisualSensorCapabilitiesRecord::getSensorName(void)
{
	return this->sensorName.getValue();
}

void VisualSensorCapabilitiesRecord::setSensorName(std::string value)
{
	this->sensorName.setValue(value);
}

SupportedStatesBitField& VisualSensorCapabilitiesRecord::getSupportedStates(void)
{
	return this->supportedStates;
}

ZoomModesBitField& VisualSensorCapabilitiesRecord::getZoomModes(void)
{
	return this->zoomModes;
}

FocusModesBitField& VisualSensorCapabilitiesRecord::getFocusModes(void)
{
	return this->focusModes;
}

WhiteBalanceBitField& VisualSensorCapabilitiesRecord::getWhiteBalance(void)
{
	return this->whiteBalance;
}

ImagingModesBitField& VisualSensorCapabilitiesRecord::getImagingModes(void)
{
	return this->imagingModes;
}

ExposureModesBitField& VisualSensorCapabilitiesRecord::getExposureModes(void)
{
	return this->exposureModes;
}

MeteringModesBitField& VisualSensorCapabilitiesRecord::getMeteringModes(void)
{
	return this->meteringModes;
}

double VisualSensorCapabilitiesRecord::getMinimumShutterSpeed_sec(void)
{
	return this->minimumShutterSpeed_sec.getValue();
}

void VisualSensorCapabilitiesRecord::setMinimumShutterSpeed_sec(double value)
{
	this->minimumShutterSpeed_sec.setValue(value);
}

double VisualSensorCapabilitiesRecord::getMaximumShutterSpeed_sec(void)
{
	return this->maximumShutterSpeed_sec.getValue();
}

void VisualSensorCapabilitiesRecord::setMaximumShutterSpeed_sec(double value)
{
	this->maximumShutterSpeed_sec.setValue(value);
}

double VisualSensorCapabilitiesRecord::getMinimumAperture(void)
{
	return this->minimumAperture.getValue();
}

void VisualSensorCapabilitiesRecord::setMinimumAperture(double value)
{
	this->minimumAperture.setValue(value);
}

double VisualSensorCapabilitiesRecord::getMaximumAperture(void)
{
	return this->maximumAperture.getValue();
}

void VisualSensorCapabilitiesRecord::setMaximumAperture(double value)
{
	this->maximumAperture.setValue(value);
}

double VisualSensorCapabilitiesRecord::getMinimumFocalLength_m(void)
{
	return this->minimumFocalLength_m.getValue();
}

void VisualSensorCapabilitiesRecord::setMinimumFocalLength_m(double value)
{
	this->minimumFocalLength_m.setValue(value);
}

double VisualSensorCapabilitiesRecord::getMaximumFocalLength_m(void)
{
	return this->maximumFocalLength_m.getValue();
}

void VisualSensorCapabilitiesRecord::setMaximumFocalLength_m(double value)
{
	this->maximumFocalLength_m.setValue(value);
}

LightSensitivityLevelsBitField& VisualSensorCapabilitiesRecord::getLightSensitivityLevels(void)
{
	return this->lightSensitivityLevels;
}

ImageStabilizationEnumeration::ImageStabilizationEnum VisualSensorCapabilitiesRecord::getImageStabilization(void)
{
	return this->imageStabilization.getValue();
}

void VisualSensorCapabilitiesRecord::setImageStabilization(ImageStabilizationEnumeration::ImageStabilizationEnum value)
{
	this->imageStabilization.setValue(value);
}

int VisualSensorCapabilitiesRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(this->presenceVector);
	byteSize += dst->pack(sensorID);
	byteSize += dst->pack(sensorName);
	if(this->isSupportedStatesEnabled())
	{
		byteSize += dst->pack(supportedStates);
	}
	if(this->isZoomModesEnabled())
	{
		byteSize += dst->pack(zoomModes);
	}
	if(this->isFocusModesEnabled())
	{
		byteSize += dst->pack(focusModes);
	}
	if(this->isWhiteBalanceEnabled())
	{
		byteSize += dst->pack(whiteBalance);
	}
	if(this->isImagingModesEnabled())
	{
		byteSize += dst->pack(imagingModes);
	}
	if(this->isExposureModesEnabled())
	{
		byteSize += dst->pack(exposureModes);
	}
	if(this->isMeteringModesEnabled())
	{
		byteSize += dst->pack(meteringModes);
	}
	if(this->isMinimumShutterSpeedEnabled())
	{
		byteSize += dst->pack(minimumShutterSpeed_sec);
	}
	if(this->isMaximumShutterSpeedEnabled())
	{
		byteSize += dst->pack(maximumShutterSpeed_sec);
	}
	if(this->isMinimumApertureEnabled())
	{
		byteSize += dst->pack(minimumAperture);
	}
	if(this->isMaximumApertureEnabled())
	{
		byteSize += dst->pack(maximumAperture);
	}
	if(this->isMinimumFocalLengthEnabled())
	{
		byteSize += dst->pack(minimumFocalLength_m);
	}
	if(this->isMaximumFocalLengthEnabled())
	{
		byteSize += dst->pack(maximumFocalLength_m);
	}
	if(this->isLightSensitivityLevelsEnabled())
	{
		byteSize += dst->pack(lightSensitivityLevels);
	}
	if(this->isImageStabilizationEnabled())
	{
		byteSize += dst->pack(imageStabilization);
	}
	return byteSize;
}
int VisualSensorCapabilitiesRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(this->presenceVector);
	byteSize += src->unpack(sensorID);
	byteSize += src->unpack(sensorName);
	if(this->isSupportedStatesEnabled())
	{
		byteSize += src->unpack(supportedStates);
	}
	if(this->isZoomModesEnabled())
	{
		byteSize += src->unpack(zoomModes);
	}
	if(this->isFocusModesEnabled())
	{
		byteSize += src->unpack(focusModes);
	}
	if(this->isWhiteBalanceEnabled())
	{
		byteSize += src->unpack(whiteBalance);
	}
	if(this->isImagingModesEnabled())
	{
		byteSize += src->unpack(imagingModes);
	}
	if(this->isExposureModesEnabled())
	{
		byteSize += src->unpack(exposureModes);
	}
	if(this->isMeteringModesEnabled())
	{
		byteSize += src->unpack(meteringModes);
	}
	if(this->isMinimumShutterSpeedEnabled())
	{
		byteSize += src->unpack(minimumShutterSpeed_sec);
	}
	if(this->isMaximumShutterSpeedEnabled())
	{
		byteSize += src->unpack(maximumShutterSpeed_sec);
	}
	if(this->isMinimumApertureEnabled())
	{
		byteSize += src->unpack(minimumAperture);
	}
	if(this->isMaximumApertureEnabled())
	{
		byteSize += src->unpack(maximumAperture);
	}
	if(this->isMinimumFocalLengthEnabled())
	{
		byteSize += src->unpack(minimumFocalLength_m);
	}
	if(this->isMaximumFocalLengthEnabled())
	{
		byteSize += src->unpack(maximumFocalLength_m);
	}
	if(this->isLightSensitivityLevelsEnabled())
	{
		byteSize += src->unpack(lightSensitivityLevels);
	}
	if(this->isImageStabilizationEnabled())
	{
		byteSize += src->unpack(imageStabilization);
	}
	return byteSize;
}

int VisualSensorCapabilitiesRecord::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // PresenceVector
	length += sensorID.length(); // sensorID
	length += sensorName.length(); // sensorName
	if(this->isSupportedStatesEnabled())
	{
		length += supportedStates.length(); // supportedStates
	}
	if(this->isZoomModesEnabled())
	{
		length += zoomModes.length(); // zoomModes
	}
	if(this->isFocusModesEnabled())
	{
		length += focusModes.length(); // focusModes
	}
	if(this->isWhiteBalanceEnabled())
	{
		length += whiteBalance.length(); // whiteBalance
	}
	if(this->isImagingModesEnabled())
	{
		length += imagingModes.length(); // imagingModes
	}
	if(this->isExposureModesEnabled())
	{
		length += exposureModes.length(); // exposureModes
	}
	if(this->isMeteringModesEnabled())
	{
		length += meteringModes.length(); // meteringModes
	}
	if(this->isMinimumShutterSpeedEnabled())
	{
		length += minimumShutterSpeed_sec.length(); // minimumShutterSpeed_sec
	}
	if(this->isMaximumShutterSpeedEnabled())
	{
		length += maximumShutterSpeed_sec.length(); // maximumShutterSpeed_sec
	}
	if(this->isMinimumApertureEnabled())
	{
		length += minimumAperture.length(); // minimumAperture
	}
	if(this->isMaximumApertureEnabled())
	{
		length += maximumAperture.length(); // maximumAperture
	}
	if(this->isMinimumFocalLengthEnabled())
	{
		length += minimumFocalLength_m.length(); // minimumFocalLength_m
	}
	if(this->isMaximumFocalLengthEnabled())
	{
		length += maximumFocalLength_m.length(); // maximumFocalLength_m
	}
	if(this->isLightSensitivityLevelsEnabled())
	{
		length += lightSensitivityLevels.length(); // lightSensitivityLevels
	}
	if(this->isImageStabilizationEnabled())
	{
		length += imageStabilization.length(); // imageStabilization
	}
	return length;
}

std::string VisualSensorCapabilitiesRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"VisualSensorCapabilitiesRecord\">\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint16_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isSupportedStatesEnabled value=\"" << std::boolalpha << this->isSupportedStatesEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isZoomModesEnabled value=\"" << std::boolalpha << this->isZoomModesEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isFocusModesEnabled value=\"" << std::boolalpha << this->isFocusModesEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isWhiteBalanceEnabled value=\"" << std::boolalpha << this->isWhiteBalanceEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isImagingModesEnabled value=\"" << std::boolalpha << this->isImagingModesEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isExposureModesEnabled value=\"" << std::boolalpha << this->isExposureModesEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMeteringModesEnabled value=\"" << std::boolalpha << this->isMeteringModesEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMinimumShutterSpeedEnabled value=\"" << std::boolalpha << this->isMinimumShutterSpeedEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMaximumShutterSpeedEnabled value=\"" << std::boolalpha << this->isMaximumShutterSpeedEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMinimumApertureEnabled value=\"" << std::boolalpha << this->isMinimumApertureEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMaximumApertureEnabled value=\"" << std::boolalpha << this->isMaximumApertureEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMinimumFocalLengthEnabled value=\"" << std::boolalpha << this->isMinimumFocalLengthEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMaximumFocalLengthEnabled value=\"" << std::boolalpha << this->isMaximumFocalLengthEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isLightSensitivityLevelsEnabled value=\"" << std::boolalpha << this->isLightSensitivityLevelsEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isImageStabilizationEnabled value=\"" << std::boolalpha << this->isImageStabilizationEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	oss << sensorID.toXml(level+1); // sensorID
	oss << sensorName.toXml(level+1); // sensorName
	if(this->isSupportedStatesEnabled())
	{
		oss << supportedStates.toXml(level+1); // supportedStates
	}
	if(this->isZoomModesEnabled())
	{
		oss << zoomModes.toXml(level+1); // zoomModes
	}
	if(this->isFocusModesEnabled())
	{
		oss << focusModes.toXml(level+1); // focusModes
	}
	if(this->isWhiteBalanceEnabled())
	{
		oss << whiteBalance.toXml(level+1); // whiteBalance
	}
	if(this->isImagingModesEnabled())
	{
		oss << imagingModes.toXml(level+1); // imagingModes
	}
	if(this->isExposureModesEnabled())
	{
		oss << exposureModes.toXml(level+1); // exposureModes
	}
	if(this->isMeteringModesEnabled())
	{
		oss << meteringModes.toXml(level+1); // meteringModes
	}
	if(this->isMinimumShutterSpeedEnabled())
	{
		oss << minimumShutterSpeed_sec.toXml(level+1); // minimumShutterSpeed_sec
	}
	if(this->isMaximumShutterSpeedEnabled())
	{
		oss << maximumShutterSpeed_sec.toXml(level+1); // maximumShutterSpeed_sec
	}
	if(this->isMinimumApertureEnabled())
	{
		oss << minimumAperture.toXml(level+1); // minimumAperture
	}
	if(this->isMaximumApertureEnabled())
	{
		oss << maximumAperture.toXml(level+1); // maximumAperture
	}
	if(this->isMinimumFocalLengthEnabled())
	{
		oss << minimumFocalLength_m.toXml(level+1); // minimumFocalLength_m
	}
	if(this->isMaximumFocalLengthEnabled())
	{
		oss << maximumFocalLength_m.toXml(level+1); // maximumFocalLength_m
	}
	if(this->isLightSensitivityLevelsEnabled())
	{
		oss << lightSensitivityLevels.toXml(level+1); // lightSensitivityLevels
	}
	if(this->isImageStabilizationEnabled())
	{
		oss << imageStabilization.toXml(level+1); // imageStabilization
	}
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}


void VisualSensorCapabilitiesRecord::setPresenceVector(uint16_t value)
{
	this->presenceVector = value;
}

uint16_t VisualSensorCapabilitiesRecord::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool VisualSensorCapabilitiesRecord::isSupportedStatesEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorCapabilitiesRecord::SUPPORTEDSTATES));
}

void VisualSensorCapabilitiesRecord::enableSupportedStates(void)
{
	this->presenceVector |= 0x01 << VisualSensorCapabilitiesRecord::SUPPORTEDSTATES;
}

void VisualSensorCapabilitiesRecord::disableSupportedStates(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorCapabilitiesRecord::SUPPORTEDSTATES);
}

bool VisualSensorCapabilitiesRecord::isZoomModesEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorCapabilitiesRecord::ZOOMMODES));
}

void VisualSensorCapabilitiesRecord::enableZoomModes(void)
{
	this->presenceVector |= 0x01 << VisualSensorCapabilitiesRecord::ZOOMMODES;
}

void VisualSensorCapabilitiesRecord::disableZoomModes(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorCapabilitiesRecord::ZOOMMODES);
}

bool VisualSensorCapabilitiesRecord::isFocusModesEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorCapabilitiesRecord::FOCUSMODES));
}

void VisualSensorCapabilitiesRecord::enableFocusModes(void)
{
	this->presenceVector |= 0x01 << VisualSensorCapabilitiesRecord::FOCUSMODES;
}

void VisualSensorCapabilitiesRecord::disableFocusModes(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorCapabilitiesRecord::FOCUSMODES);
}

bool VisualSensorCapabilitiesRecord::isWhiteBalanceEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorCapabilitiesRecord::WHITEBALANCE));
}

void VisualSensorCapabilitiesRecord::enableWhiteBalance(void)
{
	this->presenceVector |= 0x01 << VisualSensorCapabilitiesRecord::WHITEBALANCE;
}

void VisualSensorCapabilitiesRecord::disableWhiteBalance(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorCapabilitiesRecord::WHITEBALANCE);
}

bool VisualSensorCapabilitiesRecord::isImagingModesEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorCapabilitiesRecord::IMAGINGMODES));
}

void VisualSensorCapabilitiesRecord::enableImagingModes(void)
{
	this->presenceVector |= 0x01 << VisualSensorCapabilitiesRecord::IMAGINGMODES;
}

void VisualSensorCapabilitiesRecord::disableImagingModes(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorCapabilitiesRecord::IMAGINGMODES);
}

bool VisualSensorCapabilitiesRecord::isExposureModesEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorCapabilitiesRecord::EXPOSUREMODES));
}

void VisualSensorCapabilitiesRecord::enableExposureModes(void)
{
	this->presenceVector |= 0x01 << VisualSensorCapabilitiesRecord::EXPOSUREMODES;
}

void VisualSensorCapabilitiesRecord::disableExposureModes(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorCapabilitiesRecord::EXPOSUREMODES);
}

bool VisualSensorCapabilitiesRecord::isMeteringModesEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorCapabilitiesRecord::METERINGMODES));
}

void VisualSensorCapabilitiesRecord::enableMeteringModes(void)
{
	this->presenceVector |= 0x01 << VisualSensorCapabilitiesRecord::METERINGMODES;
}

void VisualSensorCapabilitiesRecord::disableMeteringModes(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorCapabilitiesRecord::METERINGMODES);
}

bool VisualSensorCapabilitiesRecord::isMinimumShutterSpeedEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorCapabilitiesRecord::MINIMUMSHUTTERSPEED_SEC));
}

void VisualSensorCapabilitiesRecord::enableMinimumShutterSpeed(void)
{
	this->presenceVector |= 0x01 << VisualSensorCapabilitiesRecord::MINIMUMSHUTTERSPEED_SEC;
}

void VisualSensorCapabilitiesRecord::disableMinimumShutterSpeed(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorCapabilitiesRecord::MINIMUMSHUTTERSPEED_SEC);
}

bool VisualSensorCapabilitiesRecord::isMaximumShutterSpeedEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorCapabilitiesRecord::MAXIMUMSHUTTERSPEED_SEC));
}

void VisualSensorCapabilitiesRecord::enableMaximumShutterSpeed(void)
{
	this->presenceVector |= 0x01 << VisualSensorCapabilitiesRecord::MAXIMUMSHUTTERSPEED_SEC;
}

void VisualSensorCapabilitiesRecord::disableMaximumShutterSpeed(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorCapabilitiesRecord::MAXIMUMSHUTTERSPEED_SEC);
}

bool VisualSensorCapabilitiesRecord::isMinimumApertureEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorCapabilitiesRecord::MINIMUMAPERTURE));
}

void VisualSensorCapabilitiesRecord::enableMinimumAperture(void)
{
	this->presenceVector |= 0x01 << VisualSensorCapabilitiesRecord::MINIMUMAPERTURE;
}

void VisualSensorCapabilitiesRecord::disableMinimumAperture(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorCapabilitiesRecord::MINIMUMAPERTURE);
}

bool VisualSensorCapabilitiesRecord::isMaximumApertureEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorCapabilitiesRecord::MAXIMUMAPERTURE));
}

void VisualSensorCapabilitiesRecord::enableMaximumAperture(void)
{
	this->presenceVector |= 0x01 << VisualSensorCapabilitiesRecord::MAXIMUMAPERTURE;
}

void VisualSensorCapabilitiesRecord::disableMaximumAperture(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorCapabilitiesRecord::MAXIMUMAPERTURE);
}

bool VisualSensorCapabilitiesRecord::isMinimumFocalLengthEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorCapabilitiesRecord::MINIMUMFOCALLENGTH_M));
}

void VisualSensorCapabilitiesRecord::enableMinimumFocalLength(void)
{
	this->presenceVector |= 0x01 << VisualSensorCapabilitiesRecord::MINIMUMFOCALLENGTH_M;
}

void VisualSensorCapabilitiesRecord::disableMinimumFocalLength(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorCapabilitiesRecord::MINIMUMFOCALLENGTH_M);
}

bool VisualSensorCapabilitiesRecord::isMaximumFocalLengthEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorCapabilitiesRecord::MAXIMUMFOCALLENGTH_M));
}

void VisualSensorCapabilitiesRecord::enableMaximumFocalLength(void)
{
	this->presenceVector |= 0x01 << VisualSensorCapabilitiesRecord::MAXIMUMFOCALLENGTH_M;
}

void VisualSensorCapabilitiesRecord::disableMaximumFocalLength(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorCapabilitiesRecord::MAXIMUMFOCALLENGTH_M);
}

bool VisualSensorCapabilitiesRecord::isLightSensitivityLevelsEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorCapabilitiesRecord::LIGHTSENSITIVITYLEVELS));
}

void VisualSensorCapabilitiesRecord::enableLightSensitivityLevels(void)
{
	this->presenceVector |= 0x01 << VisualSensorCapabilitiesRecord::LIGHTSENSITIVITYLEVELS;
}

void VisualSensorCapabilitiesRecord::disableLightSensitivityLevels(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorCapabilitiesRecord::LIGHTSENSITIVITYLEVELS);
}

bool VisualSensorCapabilitiesRecord::isImageStabilizationEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorCapabilitiesRecord::IMAGESTABILIZATION));
}

void VisualSensorCapabilitiesRecord::enableImageStabilization(void)
{
	this->presenceVector |= 0x01 << VisualSensorCapabilitiesRecord::IMAGESTABILIZATION;
}

void VisualSensorCapabilitiesRecord::disableImageStabilization(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorCapabilitiesRecord::IMAGESTABILIZATION);
}


void VisualSensorCapabilitiesRecord::copy(VisualSensorCapabilitiesRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->presenceVector = source.getPresenceVector();
	this->sensorID.setName("SensorID");
	this->sensorID.setOptional(false);
	this->sensorID.setValue(source.getSensorID()); 
 
	this->sensorName.setName("SensorName");
	this->sensorName.setOptional(false);
	this->sensorName.setValue(source.getSensorName());
	this->sensorName.setSizeType(model::fields::UNSIGNED_BYTE); 
 
	this->supportedStates.copy(source.getSupportedStates()); 
 
	this->zoomModes.copy(source.getZoomModes()); 
 
	this->focusModes.copy(source.getFocusModes()); 
 
	this->whiteBalance.copy(source.getWhiteBalance()); 
 
	this->imagingModes.copy(source.getImagingModes()); 
 
	this->exposureModes.copy(source.getExposureModes()); 
 
	this->meteringModes.copy(source.getMeteringModes()); 
 
	this->minimumShutterSpeed_sec.setName("MinimumShutterSpeed");
	this->minimumShutterSpeed_sec.setOptional(true);
	this->minimumShutterSpeed_sec.setValue(source.getMinimumShutterSpeed_sec()); 
 
	this->maximumShutterSpeed_sec.setName("MaximumShutterSpeed");
	this->maximumShutterSpeed_sec.setOptional(true);
	this->maximumShutterSpeed_sec.setValue(source.getMaximumShutterSpeed_sec()); 
 
	this->minimumAperture.setName("MinimumAperture");
	this->minimumAperture.setOptional(true);
	this->minimumAperture.setValue(source.getMinimumAperture()); 
 
	this->maximumAperture.setName("MaximumAperture");
	this->maximumAperture.setOptional(true);
	this->maximumAperture.setValue(source.getMaximumAperture()); 
 
	this->minimumFocalLength_m.setName("MinimumFocalLength");
	this->minimumFocalLength_m.setOptional(true);
	this->minimumFocalLength_m.setValue(source.getMinimumFocalLength_m()); 
 
	this->maximumFocalLength_m.setName("MaximumFocalLength");
	this->maximumFocalLength_m.setOptional(true);
	this->maximumFocalLength_m.setValue(source.getMaximumFocalLength_m()); 
 
	this->lightSensitivityLevels.copy(source.getLightSensitivityLevels()); 
 
	this->imageStabilization.setName("ImageStabilization");
	this->imageStabilization.setOptional(true);
	this->imageStabilization.setValue(source.getImageStabilization()); 
 
}

} // namespace environment
} // namespace openjaus

