/**
\file VisualSensorConfigurationRecord.h

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
#include "openjaus/environment/Triggers/Fields/VisualSensorConfigurationRecord.h"

namespace openjaus
{
namespace environment
{

VisualSensorConfigurationRecord::VisualSensorConfigurationRecord():
	sensorID(),
	sensorState(),
	zoomMode(),
	zoomLevel(),
	focalLength_m(),
	horizontalFieldOfView_rad(),
	verticalFieldOfView_rad(),
	focusMode(),
	focusValue(),
	whiteBalance(),
	imagingMode(),
	exposureMode(),
	meteringMode(),
	shutterSpeed_sec(),
	aperture(),
	lightSensitivity(),
	imageStablization()
{
	this->presenceVector = 0;

	fields.push_back(&sensorID);
	sensorID.setName("SensorID");
	sensorID.setOptional(false);
	sensorID.setValue(0);

	fields.push_back(&sensorState);
	sensorState.setName("SensorState");
	sensorState.setOptional(true);
	// Nothing to init

	fields.push_back(&zoomMode);
	zoomMode.setName("ZoomMode");
	zoomMode.setOptional(true);
	// Nothing to init

	fields.push_back(&zoomLevel);
	zoomLevel.setName("ZoomLevel");
	zoomLevel.setOptional(true);
	// Nothing to init

	fields.push_back(&focalLength_m);
	focalLength_m.setName("FocalLength");
	focalLength_m.setOptional(true);
	// Nothing to init

	fields.push_back(&horizontalFieldOfView_rad);
	horizontalFieldOfView_rad.setName("HorizontalFieldOfView");
	horizontalFieldOfView_rad.setOptional(true);
	// Nothing to init

	fields.push_back(&verticalFieldOfView_rad);
	verticalFieldOfView_rad.setName("VerticalFieldOfView");
	verticalFieldOfView_rad.setOptional(true);
	// Nothing to init

	fields.push_back(&focusMode);
	focusMode.setName("FocusMode");
	focusMode.setOptional(true);
	// Nothing to init

	fields.push_back(&focusValue);
	focusValue.setName("FocusValue");
	focusValue.setOptional(true);
	// Nothing to init

	fields.push_back(&whiteBalance);
	whiteBalance.setName("WhiteBalance");
	whiteBalance.setOptional(true);
	// Nothing to init

	fields.push_back(&imagingMode);
	imagingMode.setName("ImagingMode");
	imagingMode.setOptional(true);
	// Nothing to init

	fields.push_back(&exposureMode);
	exposureMode.setName("ExposureMode");
	exposureMode.setOptional(true);
	// Nothing to init

	fields.push_back(&meteringMode);
	meteringMode.setName("MeteringMode");
	meteringMode.setOptional(true);
	// Nothing to init

	fields.push_back(&shutterSpeed_sec);
	shutterSpeed_sec.setName("ShutterSpeed");
	shutterSpeed_sec.setOptional(true);
	// Nothing to init

	fields.push_back(&aperture);
	aperture.setName("Aperture");
	aperture.setOptional(true);
	// Nothing to init

	fields.push_back(&lightSensitivity);
	lightSensitivity.setName("LightSensitivity");
	lightSensitivity.setOptional(true);
	// Nothing to init

	fields.push_back(&imageStablization);
	imageStablization.setName("ImageStablization");
	imageStablization.setOptional(true);
	// Nothing to init

}

VisualSensorConfigurationRecord::VisualSensorConfigurationRecord(const VisualSensorConfigurationRecord &source)
{
	this->copy(const_cast<VisualSensorConfigurationRecord&>(source));
}

VisualSensorConfigurationRecord::~VisualSensorConfigurationRecord()
{

}


uint16_t VisualSensorConfigurationRecord::getSensorID(void)
{
	return this->sensorID.getValue();
}

void VisualSensorConfigurationRecord::setSensorID(uint16_t value)
{
	this->sensorID.setValue(value);
}

SensorStateEnumeration::SensorStateEnum VisualSensorConfigurationRecord::getSensorState(void)
{
	return this->sensorState.getValue();
}

void VisualSensorConfigurationRecord::setSensorState(SensorStateEnumeration::SensorStateEnum value)
{
	this->sensorState.setValue(value);
}

ZoomModeEnumeration::ZoomModeEnum VisualSensorConfigurationRecord::getZoomMode(void)
{
	return this->zoomMode.getValue();
}

void VisualSensorConfigurationRecord::setZoomMode(ZoomModeEnumeration::ZoomModeEnum value)
{
	this->zoomMode.setValue(value);
}

double VisualSensorConfigurationRecord::getZoomLevel(void)
{
	return this->zoomLevel.getValue();
}

void VisualSensorConfigurationRecord::setZoomLevel(double value)
{
	this->zoomLevel.setValue(value);
}

double VisualSensorConfigurationRecord::getFocalLength_m(void)
{
	return this->focalLength_m.getValue();
}

void VisualSensorConfigurationRecord::setFocalLength_m(double value)
{
	this->focalLength_m.setValue(value);
}

double VisualSensorConfigurationRecord::getHorizontalFieldOfView_rad(void)
{
	return this->horizontalFieldOfView_rad.getValue();
}

void VisualSensorConfigurationRecord::setHorizontalFieldOfView_rad(double value)
{
	this->horizontalFieldOfView_rad.setValue(value);
}

double VisualSensorConfigurationRecord::getVerticalFieldOfView_rad(void)
{
	return this->verticalFieldOfView_rad.getValue();
}

void VisualSensorConfigurationRecord::setVerticalFieldOfView_rad(double value)
{
	this->verticalFieldOfView_rad.setValue(value);
}

FocusModeEnumeration::FocusModeEnum VisualSensorConfigurationRecord::getFocusMode(void)
{
	return this->focusMode.getValue();
}

void VisualSensorConfigurationRecord::setFocusMode(FocusModeEnumeration::FocusModeEnum value)
{
	this->focusMode.setValue(value);
}

double VisualSensorConfigurationRecord::getFocusValue(void)
{
	return this->focusValue.getValue();
}

void VisualSensorConfigurationRecord::setFocusValue(double value)
{
	this->focusValue.setValue(value);
}

WhiteBalanceEnumeration::WhiteBalanceEnum VisualSensorConfigurationRecord::getWhiteBalance(void)
{
	return this->whiteBalance.getValue();
}

void VisualSensorConfigurationRecord::setWhiteBalance(WhiteBalanceEnumeration::WhiteBalanceEnum value)
{
	this->whiteBalance.setValue(value);
}

ImagingModeEnumeration::ImagingModeEnum VisualSensorConfigurationRecord::getImagingMode(void)
{
	return this->imagingMode.getValue();
}

void VisualSensorConfigurationRecord::setImagingMode(ImagingModeEnumeration::ImagingModeEnum value)
{
	this->imagingMode.setValue(value);
}

ExposureModeEnumeration::ExposureModeEnum VisualSensorConfigurationRecord::getExposureMode(void)
{
	return this->exposureMode.getValue();
}

void VisualSensorConfigurationRecord::setExposureMode(ExposureModeEnumeration::ExposureModeEnum value)
{
	this->exposureMode.setValue(value);
}

MeteringModeEnumeration::MeteringModeEnum VisualSensorConfigurationRecord::getMeteringMode(void)
{
	return this->meteringMode.getValue();
}

void VisualSensorConfigurationRecord::setMeteringMode(MeteringModeEnumeration::MeteringModeEnum value)
{
	this->meteringMode.setValue(value);
}

double VisualSensorConfigurationRecord::getShutterSpeed_sec(void)
{
	return this->shutterSpeed_sec.getValue();
}

void VisualSensorConfigurationRecord::setShutterSpeed_sec(double value)
{
	this->shutterSpeed_sec.setValue(value);
}

double VisualSensorConfigurationRecord::getAperture(void)
{
	return this->aperture.getValue();
}

void VisualSensorConfigurationRecord::setAperture(double value)
{
	this->aperture.setValue(value);
}

LightSensitivityEnumeration::LightSensitivityEnum VisualSensorConfigurationRecord::getLightSensitivity(void)
{
	return this->lightSensitivity.getValue();
}

void VisualSensorConfigurationRecord::setLightSensitivity(LightSensitivityEnumeration::LightSensitivityEnum value)
{
	this->lightSensitivity.setValue(value);
}

ImageStablizationEnumeration::ImageStablizationEnum VisualSensorConfigurationRecord::getImageStablization(void)
{
	return this->imageStablization.getValue();
}

void VisualSensorConfigurationRecord::setImageStablization(ImageStablizationEnumeration::ImageStablizationEnum value)
{
	this->imageStablization.setValue(value);
}

int VisualSensorConfigurationRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(this->presenceVector);
	byteSize += dst->pack(sensorID);
	if(this->isSensorStateEnabled())
	{
		byteSize += dst->pack(sensorState);
	}
	if(this->isZoomModeEnabled())
	{
		byteSize += dst->pack(zoomMode);
	}
	if(this->isZoomLevelEnabled())
	{
		byteSize += dst->pack(zoomLevel);
	}
	if(this->isFocalLengthEnabled())
	{
		byteSize += dst->pack(focalLength_m);
	}
	if(this->isHorizontalFieldOfViewEnabled())
	{
		byteSize += dst->pack(horizontalFieldOfView_rad);
	}
	if(this->isVerticalFieldOfViewEnabled())
	{
		byteSize += dst->pack(verticalFieldOfView_rad);
	}
	if(this->isFocusModeEnabled())
	{
		byteSize += dst->pack(focusMode);
	}
	if(this->isFocusValueEnabled())
	{
		byteSize += dst->pack(focusValue);
	}
	if(this->isWhiteBalanceEnabled())
	{
		byteSize += dst->pack(whiteBalance);
	}
	if(this->isImagingModeEnabled())
	{
		byteSize += dst->pack(imagingMode);
	}
	if(this->isExposureModeEnabled())
	{
		byteSize += dst->pack(exposureMode);
	}
	if(this->isMeteringModeEnabled())
	{
		byteSize += dst->pack(meteringMode);
	}
	if(this->isShutterSpeedEnabled())
	{
		byteSize += dst->pack(shutterSpeed_sec);
	}
	if(this->isApertureEnabled())
	{
		byteSize += dst->pack(aperture);
	}
	if(this->isLightSensitivityEnabled())
	{
		byteSize += dst->pack(lightSensitivity);
	}
	if(this->isImageStablizationEnabled())
	{
		byteSize += dst->pack(imageStablization);
	}
	return byteSize;
}
int VisualSensorConfigurationRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(this->presenceVector);
	byteSize += src->unpack(sensorID);
	if(this->isSensorStateEnabled())
	{
		byteSize += src->unpack(sensorState);
	}
	if(this->isZoomModeEnabled())
	{
		byteSize += src->unpack(zoomMode);
	}
	if(this->isZoomLevelEnabled())
	{
		byteSize += src->unpack(zoomLevel);
	}
	if(this->isFocalLengthEnabled())
	{
		byteSize += src->unpack(focalLength_m);
	}
	if(this->isHorizontalFieldOfViewEnabled())
	{
		byteSize += src->unpack(horizontalFieldOfView_rad);
	}
	if(this->isVerticalFieldOfViewEnabled())
	{
		byteSize += src->unpack(verticalFieldOfView_rad);
	}
	if(this->isFocusModeEnabled())
	{
		byteSize += src->unpack(focusMode);
	}
	if(this->isFocusValueEnabled())
	{
		byteSize += src->unpack(focusValue);
	}
	if(this->isWhiteBalanceEnabled())
	{
		byteSize += src->unpack(whiteBalance);
	}
	if(this->isImagingModeEnabled())
	{
		byteSize += src->unpack(imagingMode);
	}
	if(this->isExposureModeEnabled())
	{
		byteSize += src->unpack(exposureMode);
	}
	if(this->isMeteringModeEnabled())
	{
		byteSize += src->unpack(meteringMode);
	}
	if(this->isShutterSpeedEnabled())
	{
		byteSize += src->unpack(shutterSpeed_sec);
	}
	if(this->isApertureEnabled())
	{
		byteSize += src->unpack(aperture);
	}
	if(this->isLightSensitivityEnabled())
	{
		byteSize += src->unpack(lightSensitivity);
	}
	if(this->isImageStablizationEnabled())
	{
		byteSize += src->unpack(imageStablization);
	}
	return byteSize;
}

int VisualSensorConfigurationRecord::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // PresenceVector
	length += sensorID.length(); // sensorID
	if(this->isSensorStateEnabled())
	{
		length += sensorState.length(); // sensorState
	}
	if(this->isZoomModeEnabled())
	{
		length += zoomMode.length(); // zoomMode
	}
	if(this->isZoomLevelEnabled())
	{
		length += zoomLevel.length(); // zoomLevel
	}
	if(this->isFocalLengthEnabled())
	{
		length += focalLength_m.length(); // focalLength_m
	}
	if(this->isHorizontalFieldOfViewEnabled())
	{
		length += horizontalFieldOfView_rad.length(); // horizontalFieldOfView_rad
	}
	if(this->isVerticalFieldOfViewEnabled())
	{
		length += verticalFieldOfView_rad.length(); // verticalFieldOfView_rad
	}
	if(this->isFocusModeEnabled())
	{
		length += focusMode.length(); // focusMode
	}
	if(this->isFocusValueEnabled())
	{
		length += focusValue.length(); // focusValue
	}
	if(this->isWhiteBalanceEnabled())
	{
		length += whiteBalance.length(); // whiteBalance
	}
	if(this->isImagingModeEnabled())
	{
		length += imagingMode.length(); // imagingMode
	}
	if(this->isExposureModeEnabled())
	{
		length += exposureMode.length(); // exposureMode
	}
	if(this->isMeteringModeEnabled())
	{
		length += meteringMode.length(); // meteringMode
	}
	if(this->isShutterSpeedEnabled())
	{
		length += shutterSpeed_sec.length(); // shutterSpeed_sec
	}
	if(this->isApertureEnabled())
	{
		length += aperture.length(); // aperture
	}
	if(this->isLightSensitivityEnabled())
	{
		length += lightSensitivity.length(); // lightSensitivity
	}
	if(this->isImageStablizationEnabled())
	{
		length += imageStablization.length(); // imageStablization
	}
	return length;
}

std::string VisualSensorConfigurationRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"VisualSensorConfigurationRecord\">\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint16_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isSensorStateEnabled value=\"" << std::boolalpha << this->isSensorStateEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isZoomModeEnabled value=\"" << std::boolalpha << this->isZoomModeEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isZoomLevelEnabled value=\"" << std::boolalpha << this->isZoomLevelEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isFocalLengthEnabled value=\"" << std::boolalpha << this->isFocalLengthEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isHorizontalFieldOfViewEnabled value=\"" << std::boolalpha << this->isHorizontalFieldOfViewEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isVerticalFieldOfViewEnabled value=\"" << std::boolalpha << this->isVerticalFieldOfViewEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isFocusModeEnabled value=\"" << std::boolalpha << this->isFocusModeEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isFocusValueEnabled value=\"" << std::boolalpha << this->isFocusValueEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isWhiteBalanceEnabled value=\"" << std::boolalpha << this->isWhiteBalanceEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isImagingModeEnabled value=\"" << std::boolalpha << this->isImagingModeEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isExposureModeEnabled value=\"" << std::boolalpha << this->isExposureModeEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMeteringModeEnabled value=\"" << std::boolalpha << this->isMeteringModeEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isShutterSpeedEnabled value=\"" << std::boolalpha << this->isShutterSpeedEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isApertureEnabled value=\"" << std::boolalpha << this->isApertureEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isLightSensitivityEnabled value=\"" << std::boolalpha << this->isLightSensitivityEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isImageStablizationEnabled value=\"" << std::boolalpha << this->isImageStablizationEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	oss << sensorID.toXml(level+1); // sensorID
	if(this->isSensorStateEnabled())
	{
		oss << sensorState.toXml(level+1); // sensorState
	}
	if(this->isZoomModeEnabled())
	{
		oss << zoomMode.toXml(level+1); // zoomMode
	}
	if(this->isZoomLevelEnabled())
	{
		oss << zoomLevel.toXml(level+1); // zoomLevel
	}
	if(this->isFocalLengthEnabled())
	{
		oss << focalLength_m.toXml(level+1); // focalLength_m
	}
	if(this->isHorizontalFieldOfViewEnabled())
	{
		oss << horizontalFieldOfView_rad.toXml(level+1); // horizontalFieldOfView_rad
	}
	if(this->isVerticalFieldOfViewEnabled())
	{
		oss << verticalFieldOfView_rad.toXml(level+1); // verticalFieldOfView_rad
	}
	if(this->isFocusModeEnabled())
	{
		oss << focusMode.toXml(level+1); // focusMode
	}
	if(this->isFocusValueEnabled())
	{
		oss << focusValue.toXml(level+1); // focusValue
	}
	if(this->isWhiteBalanceEnabled())
	{
		oss << whiteBalance.toXml(level+1); // whiteBalance
	}
	if(this->isImagingModeEnabled())
	{
		oss << imagingMode.toXml(level+1); // imagingMode
	}
	if(this->isExposureModeEnabled())
	{
		oss << exposureMode.toXml(level+1); // exposureMode
	}
	if(this->isMeteringModeEnabled())
	{
		oss << meteringMode.toXml(level+1); // meteringMode
	}
	if(this->isShutterSpeedEnabled())
	{
		oss << shutterSpeed_sec.toXml(level+1); // shutterSpeed_sec
	}
	if(this->isApertureEnabled())
	{
		oss << aperture.toXml(level+1); // aperture
	}
	if(this->isLightSensitivityEnabled())
	{
		oss << lightSensitivity.toXml(level+1); // lightSensitivity
	}
	if(this->isImageStablizationEnabled())
	{
		oss << imageStablization.toXml(level+1); // imageStablization
	}
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}


void VisualSensorConfigurationRecord::setPresenceVector(uint16_t value)
{
	this->presenceVector = value;
}

uint16_t VisualSensorConfigurationRecord::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool VisualSensorConfigurationRecord::isSensorStateEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorConfigurationRecord::SENSORSTATE));
}

void VisualSensorConfigurationRecord::enableSensorState(void)
{
	this->presenceVector |= 0x01 << VisualSensorConfigurationRecord::SENSORSTATE;
}

void VisualSensorConfigurationRecord::disableSensorState(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorConfigurationRecord::SENSORSTATE);
}

bool VisualSensorConfigurationRecord::isZoomModeEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorConfigurationRecord::ZOOMMODE));
}

void VisualSensorConfigurationRecord::enableZoomMode(void)
{
	this->presenceVector |= 0x01 << VisualSensorConfigurationRecord::ZOOMMODE;
}

void VisualSensorConfigurationRecord::disableZoomMode(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorConfigurationRecord::ZOOMMODE);
}

bool VisualSensorConfigurationRecord::isZoomLevelEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorConfigurationRecord::ZOOMLEVEL));
}

void VisualSensorConfigurationRecord::enableZoomLevel(void)
{
	this->presenceVector |= 0x01 << VisualSensorConfigurationRecord::ZOOMLEVEL;
}

void VisualSensorConfigurationRecord::disableZoomLevel(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorConfigurationRecord::ZOOMLEVEL);
}

bool VisualSensorConfigurationRecord::isFocalLengthEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorConfigurationRecord::FOCALLENGTH_M));
}

void VisualSensorConfigurationRecord::enableFocalLength(void)
{
	this->presenceVector |= 0x01 << VisualSensorConfigurationRecord::FOCALLENGTH_M;
}

void VisualSensorConfigurationRecord::disableFocalLength(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorConfigurationRecord::FOCALLENGTH_M);
}

bool VisualSensorConfigurationRecord::isHorizontalFieldOfViewEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorConfigurationRecord::HORIZONTALFIELDOFVIEW_RAD));
}

void VisualSensorConfigurationRecord::enableHorizontalFieldOfView(void)
{
	this->presenceVector |= 0x01 << VisualSensorConfigurationRecord::HORIZONTALFIELDOFVIEW_RAD;
}

void VisualSensorConfigurationRecord::disableHorizontalFieldOfView(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorConfigurationRecord::HORIZONTALFIELDOFVIEW_RAD);
}

bool VisualSensorConfigurationRecord::isVerticalFieldOfViewEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorConfigurationRecord::VERTICALFIELDOFVIEW_RAD));
}

void VisualSensorConfigurationRecord::enableVerticalFieldOfView(void)
{
	this->presenceVector |= 0x01 << VisualSensorConfigurationRecord::VERTICALFIELDOFVIEW_RAD;
}

void VisualSensorConfigurationRecord::disableVerticalFieldOfView(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorConfigurationRecord::VERTICALFIELDOFVIEW_RAD);
}

bool VisualSensorConfigurationRecord::isFocusModeEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorConfigurationRecord::FOCUSMODE));
}

void VisualSensorConfigurationRecord::enableFocusMode(void)
{
	this->presenceVector |= 0x01 << VisualSensorConfigurationRecord::FOCUSMODE;
}

void VisualSensorConfigurationRecord::disableFocusMode(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorConfigurationRecord::FOCUSMODE);
}

bool VisualSensorConfigurationRecord::isFocusValueEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorConfigurationRecord::FOCUSVALUE));
}

void VisualSensorConfigurationRecord::enableFocusValue(void)
{
	this->presenceVector |= 0x01 << VisualSensorConfigurationRecord::FOCUSVALUE;
}

void VisualSensorConfigurationRecord::disableFocusValue(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorConfigurationRecord::FOCUSVALUE);
}

bool VisualSensorConfigurationRecord::isWhiteBalanceEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorConfigurationRecord::WHITEBALANCE));
}

void VisualSensorConfigurationRecord::enableWhiteBalance(void)
{
	this->presenceVector |= 0x01 << VisualSensorConfigurationRecord::WHITEBALANCE;
}

void VisualSensorConfigurationRecord::disableWhiteBalance(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorConfigurationRecord::WHITEBALANCE);
}

bool VisualSensorConfigurationRecord::isImagingModeEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorConfigurationRecord::IMAGINGMODE));
}

void VisualSensorConfigurationRecord::enableImagingMode(void)
{
	this->presenceVector |= 0x01 << VisualSensorConfigurationRecord::IMAGINGMODE;
}

void VisualSensorConfigurationRecord::disableImagingMode(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorConfigurationRecord::IMAGINGMODE);
}

bool VisualSensorConfigurationRecord::isExposureModeEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorConfigurationRecord::EXPOSUREMODE));
}

void VisualSensorConfigurationRecord::enableExposureMode(void)
{
	this->presenceVector |= 0x01 << VisualSensorConfigurationRecord::EXPOSUREMODE;
}

void VisualSensorConfigurationRecord::disableExposureMode(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorConfigurationRecord::EXPOSUREMODE);
}

bool VisualSensorConfigurationRecord::isMeteringModeEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorConfigurationRecord::METERINGMODE));
}

void VisualSensorConfigurationRecord::enableMeteringMode(void)
{
	this->presenceVector |= 0x01 << VisualSensorConfigurationRecord::METERINGMODE;
}

void VisualSensorConfigurationRecord::disableMeteringMode(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorConfigurationRecord::METERINGMODE);
}

bool VisualSensorConfigurationRecord::isShutterSpeedEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorConfigurationRecord::SHUTTERSPEED_SEC));
}

void VisualSensorConfigurationRecord::enableShutterSpeed(void)
{
	this->presenceVector |= 0x01 << VisualSensorConfigurationRecord::SHUTTERSPEED_SEC;
}

void VisualSensorConfigurationRecord::disableShutterSpeed(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorConfigurationRecord::SHUTTERSPEED_SEC);
}

bool VisualSensorConfigurationRecord::isApertureEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorConfigurationRecord::APERTURE));
}

void VisualSensorConfigurationRecord::enableAperture(void)
{
	this->presenceVector |= 0x01 << VisualSensorConfigurationRecord::APERTURE;
}

void VisualSensorConfigurationRecord::disableAperture(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorConfigurationRecord::APERTURE);
}

bool VisualSensorConfigurationRecord::isLightSensitivityEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorConfigurationRecord::LIGHTSENSITIVITY));
}

void VisualSensorConfigurationRecord::enableLightSensitivity(void)
{
	this->presenceVector |= 0x01 << VisualSensorConfigurationRecord::LIGHTSENSITIVITY;
}

void VisualSensorConfigurationRecord::disableLightSensitivity(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorConfigurationRecord::LIGHTSENSITIVITY);
}

bool VisualSensorConfigurationRecord::isImageStablizationEnabled(void) const
{
	return (this->presenceVector & (0x01 << VisualSensorConfigurationRecord::IMAGESTABLIZATION));
}

void VisualSensorConfigurationRecord::enableImageStablization(void)
{
	this->presenceVector |= 0x01 << VisualSensorConfigurationRecord::IMAGESTABLIZATION;
}

void VisualSensorConfigurationRecord::disableImageStablization(void)
{
	this->presenceVector &= ~(0x01 << VisualSensorConfigurationRecord::IMAGESTABLIZATION);
}


void VisualSensorConfigurationRecord::copy(VisualSensorConfigurationRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->presenceVector = source.getPresenceVector();
	this->sensorID.setName("SensorID");
	this->sensorID.setOptional(false);
	this->sensorID.setValue(source.getSensorID()); 
 
	this->sensorState.setName("SensorState");
	this->sensorState.setOptional(true);
	this->sensorState.setValue(source.getSensorState()); 
 
	this->zoomMode.setName("ZoomMode");
	this->zoomMode.setOptional(true);
	this->zoomMode.setValue(source.getZoomMode()); 
 
	this->zoomLevel.setName("ZoomLevel");
	this->zoomLevel.setOptional(true);
	this->zoomLevel.setValue(source.getZoomLevel()); 
 
	this->focalLength_m.setName("FocalLength");
	this->focalLength_m.setOptional(true);
	this->focalLength_m.setValue(source.getFocalLength_m()); 
 
	this->horizontalFieldOfView_rad.setName("HorizontalFieldOfView");
	this->horizontalFieldOfView_rad.setOptional(true);
	this->horizontalFieldOfView_rad.setValue(source.getHorizontalFieldOfView_rad()); 
 
	this->verticalFieldOfView_rad.setName("VerticalFieldOfView");
	this->verticalFieldOfView_rad.setOptional(true);
	this->verticalFieldOfView_rad.setValue(source.getVerticalFieldOfView_rad()); 
 
	this->focusMode.setName("FocusMode");
	this->focusMode.setOptional(true);
	this->focusMode.setValue(source.getFocusMode()); 
 
	this->focusValue.setName("FocusValue");
	this->focusValue.setOptional(true);
	this->focusValue.setValue(source.getFocusValue()); 
 
	this->whiteBalance.setName("WhiteBalance");
	this->whiteBalance.setOptional(true);
	this->whiteBalance.setValue(source.getWhiteBalance()); 
 
	this->imagingMode.setName("ImagingMode");
	this->imagingMode.setOptional(true);
	this->imagingMode.setValue(source.getImagingMode()); 
 
	this->exposureMode.setName("ExposureMode");
	this->exposureMode.setOptional(true);
	this->exposureMode.setValue(source.getExposureMode()); 
 
	this->meteringMode.setName("MeteringMode");
	this->meteringMode.setOptional(true);
	this->meteringMode.setValue(source.getMeteringMode()); 
 
	this->shutterSpeed_sec.setName("ShutterSpeed");
	this->shutterSpeed_sec.setOptional(true);
	this->shutterSpeed_sec.setValue(source.getShutterSpeed_sec()); 
 
	this->aperture.setName("Aperture");
	this->aperture.setOptional(true);
	this->aperture.setValue(source.getAperture()); 
 
	this->lightSensitivity.setName("LightSensitivity");
	this->lightSensitivity.setOptional(true);
	this->lightSensitivity.setValue(source.getLightSensitivity()); 
 
	this->imageStablization.setName("ImageStablization");
	this->imageStablization.setOptional(true);
	this->imageStablization.setValue(source.getImageStablization()); 
 
}

} // namespace environment
} // namespace openjaus

