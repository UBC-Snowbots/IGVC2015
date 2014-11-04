
/**
\file VisualSensorConfigurationRec.h

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

#ifndef VISUALSENSORCONFIGURATIONRECORD_H
#define VISUALSENSORCONFIGURATIONRECORD_H

#include <openjaus.h>
#include "openjaus/environment/Triggers/Fields/SensorStateEnumeration.h"
#include "openjaus/environment/Triggers/Fields/ZoomModeEnumeration.h"
#include "openjaus/environment/Triggers/Fields/ZoomLevelScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/FocalLengthScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/HorizontalFieldOfViewScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/VerticalFieldOfViewScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/FocusModeEnumeration.h"
#include "openjaus/environment/Triggers/Fields/FocusValueScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/WhiteBalanceEnumeration.h"
#include "openjaus/environment/Triggers/Fields/ImagingModeEnumeration.h"
#include "openjaus/environment/Triggers/Fields/ExposureModeEnumeration.h"
#include "openjaus/environment/Triggers/Fields/MeteringModeEnumeration.h"
#include "openjaus/environment/Triggers/Fields/ShutterSpeedScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/ApertureScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/LightSensitivityEnumeration.h"
#include "openjaus/environment/Triggers/Fields/ImageStablizationEnumeration.h"

namespace openjaus
{
namespace environment
{

class OPENJAUS_EXPORT VisualSensorConfigurationRecord : public openjaus::model::fields::Record
{
public:
	VisualSensorConfigurationRecord();
	VisualSensorConfigurationRecord(const VisualSensorConfigurationRecord &source);
	~VisualSensorConfigurationRecord();

	void copy(VisualSensorConfigurationRecord& source);
	virtual int to(system::Buffer *dst);
	virtual int from(system::Buffer *src);
	virtual int length(void);
	std::string toXml(unsigned char level=0) const;
	
	void setPresenceVector(uint16_t value);
	uint16_t getPresenceVector(void) const;
	bool isSensorStateEnabled(void) const;
	void enableSensorState(void);
	void disableSensorState(void);

	bool isZoomModeEnabled(void) const;
	void enableZoomMode(void);
	void disableZoomMode(void);

	bool isZoomLevelEnabled(void) const;
	void enableZoomLevel(void);
	void disableZoomLevel(void);

	bool isFocalLengthEnabled(void) const;
	void enableFocalLength(void);
	void disableFocalLength(void);

	bool isHorizontalFieldOfViewEnabled(void) const;
	void enableHorizontalFieldOfView(void);
	void disableHorizontalFieldOfView(void);

	bool isVerticalFieldOfViewEnabled(void) const;
	void enableVerticalFieldOfView(void);
	void disableVerticalFieldOfView(void);

	bool isFocusModeEnabled(void) const;
	void enableFocusMode(void);
	void disableFocusMode(void);

	bool isFocusValueEnabled(void) const;
	void enableFocusValue(void);
	void disableFocusValue(void);

	bool isWhiteBalanceEnabled(void) const;
	void enableWhiteBalance(void);
	void disableWhiteBalance(void);

	bool isImagingModeEnabled(void) const;
	void enableImagingMode(void);
	void disableImagingMode(void);

	bool isExposureModeEnabled(void) const;
	void enableExposureMode(void);
	void disableExposureMode(void);

	bool isMeteringModeEnabled(void) const;
	void enableMeteringMode(void);
	void disableMeteringMode(void);

	bool isShutterSpeedEnabled(void) const;
	void enableShutterSpeed(void);
	void disableShutterSpeed(void);

	bool isApertureEnabled(void) const;
	void enableAperture(void);
	void disableAperture(void);

	bool isLightSensitivityEnabled(void) const;
	void enableLightSensitivity(void);
	void disableLightSensitivity(void);

	bool isImageStablizationEnabled(void) const;
	void enableImageStablization(void);
	void disableImageStablization(void);


	uint16_t getSensorID(void);
	void setSensorID(uint16_t value);

	SensorStateEnumeration::SensorStateEnum getSensorState(void);
	void setSensorState(SensorStateEnumeration::SensorStateEnum value);

	ZoomModeEnumeration::ZoomModeEnum getZoomMode(void);
	void setZoomMode(ZoomModeEnumeration::ZoomModeEnum value);

	double getZoomLevel(void);
	void setZoomLevel(double value);

	double getFocalLength_m(void);
	void setFocalLength_m(double value);

	double getHorizontalFieldOfView_rad(void);
	void setHorizontalFieldOfView_rad(double value);

	double getVerticalFieldOfView_rad(void);
	void setVerticalFieldOfView_rad(double value);

	FocusModeEnumeration::FocusModeEnum getFocusMode(void);
	void setFocusMode(FocusModeEnumeration::FocusModeEnum value);

	double getFocusValue(void);
	void setFocusValue(double value);

	WhiteBalanceEnumeration::WhiteBalanceEnum getWhiteBalance(void);
	void setWhiteBalance(WhiteBalanceEnumeration::WhiteBalanceEnum value);

	ImagingModeEnumeration::ImagingModeEnum getImagingMode(void);
	void setImagingMode(ImagingModeEnumeration::ImagingModeEnum value);

	ExposureModeEnumeration::ExposureModeEnum getExposureMode(void);
	void setExposureMode(ExposureModeEnumeration::ExposureModeEnum value);

	MeteringModeEnumeration::MeteringModeEnum getMeteringMode(void);
	void setMeteringMode(MeteringModeEnumeration::MeteringModeEnum value);

	double getShutterSpeed_sec(void);
	void setShutterSpeed_sec(double value);

	double getAperture(void);
	void setAperture(double value);

	LightSensitivityEnumeration::LightSensitivityEnum getLightSensitivity(void);
	void setLightSensitivity(LightSensitivityEnumeration::LightSensitivityEnum value);

	ImageStablizationEnumeration::ImageStablizationEnum getImageStablization(void);
	void setImageStablization(ImageStablizationEnumeration::ImageStablizationEnum value);

protected:
	model::fields::UnsignedShort sensorID;
	SensorStateEnumeration sensorState;
	ZoomModeEnumeration zoomMode;
	ZoomLevelScaledInteger zoomLevel;
	FocalLengthScaledInteger focalLength_m;
	HorizontalFieldOfViewScaledInteger horizontalFieldOfView_rad;
	VerticalFieldOfViewScaledInteger verticalFieldOfView_rad;
	FocusModeEnumeration focusMode;
	FocusValueScaledInteger focusValue;
	WhiteBalanceEnumeration whiteBalance;
	ImagingModeEnumeration imagingMode;
	ExposureModeEnumeration exposureMode;
	MeteringModeEnumeration meteringMode;
	ShutterSpeedScaledInteger shutterSpeed_sec;
	ApertureScaledInteger aperture;
	LightSensitivityEnumeration lightSensitivity;
	ImageStablizationEnumeration imageStablization;

	uint16_t presenceVector;
	enum pvEnum {SENSORSTATE = 0, ZOOMMODE = 1, ZOOMLEVEL = 2, FOCALLENGTH_M = 3, HORIZONTALFIELDOFVIEW_RAD = 4, VERTICALFIELDOFVIEW_RAD = 5, FOCUSMODE = 6, FOCUSVALUE = 7, WHITEBALANCE = 8, IMAGINGMODE = 9, EXPOSUREMODE = 10, METERINGMODE = 11, SHUTTERSPEED_SEC = 12, APERTURE = 13, LIGHTSENSITIVITY = 14, IMAGESTABLIZATION = 15};
};

} // namespace environment
} // namespace openjaus

#endif // VISUALSENSORCONFIGURATIONRECORD_H

