
/**
\file VisualSensorCapabilitiesRec.h

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

#ifndef VISUALSENSORCAPABILITIESRECORD_H
#define VISUALSENSORCAPABILITIESRECORD_H

#include <openjaus.h>
#include "openjaus/environment/Triggers/Fields/SupportedStatesBitField.h"
#include "openjaus/environment/Triggers/Fields/ZoomModesBitField.h"
#include "openjaus/environment/Triggers/Fields/FocusModesBitField.h"
#include "openjaus/environment/Triggers/Fields/WhiteBalanceBitField.h"
#include "openjaus/environment/Triggers/Fields/ImagingModesBitField.h"
#include "openjaus/environment/Triggers/Fields/ExposureModesBitField.h"
#include "openjaus/environment/Triggers/Fields/MeteringModesBitField.h"
#include "openjaus/environment/Triggers/Fields/MinimumShutterSpeedScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/MaximumShutterSpeedScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/MinimumApertureScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/MaximumApertureScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/MinimumFocalLengthScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/MaximumFocalLengthScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/LightSensitivityLevelsBitField.h"
#include "openjaus/environment/Triggers/Fields/ImageStabilizationEnumeration.h"

namespace openjaus
{
namespace environment
{

class OPENJAUS_EXPORT VisualSensorCapabilitiesRecord : public openjaus::model::fields::Record
{
public:
	VisualSensorCapabilitiesRecord();
	VisualSensorCapabilitiesRecord(const VisualSensorCapabilitiesRecord &source);
	~VisualSensorCapabilitiesRecord();

	void copy(VisualSensorCapabilitiesRecord& source);
	virtual int to(system::Buffer *dst);
	virtual int from(system::Buffer *src);
	virtual int length(void);
	std::string toXml(unsigned char level=0) const;
	
	void setPresenceVector(uint16_t value);
	uint16_t getPresenceVector(void) const;
	bool isSupportedStatesEnabled(void) const;
	void enableSupportedStates(void);
	void disableSupportedStates(void);

	bool isZoomModesEnabled(void) const;
	void enableZoomModes(void);
	void disableZoomModes(void);

	bool isFocusModesEnabled(void) const;
	void enableFocusModes(void);
	void disableFocusModes(void);

	bool isWhiteBalanceEnabled(void) const;
	void enableWhiteBalance(void);
	void disableWhiteBalance(void);

	bool isImagingModesEnabled(void) const;
	void enableImagingModes(void);
	void disableImagingModes(void);

	bool isExposureModesEnabled(void) const;
	void enableExposureModes(void);
	void disableExposureModes(void);

	bool isMeteringModesEnabled(void) const;
	void enableMeteringModes(void);
	void disableMeteringModes(void);

	bool isMinimumShutterSpeedEnabled(void) const;
	void enableMinimumShutterSpeed(void);
	void disableMinimumShutterSpeed(void);

	bool isMaximumShutterSpeedEnabled(void) const;
	void enableMaximumShutterSpeed(void);
	void disableMaximumShutterSpeed(void);

	bool isMinimumApertureEnabled(void) const;
	void enableMinimumAperture(void);
	void disableMinimumAperture(void);

	bool isMaximumApertureEnabled(void) const;
	void enableMaximumAperture(void);
	void disableMaximumAperture(void);

	bool isMinimumFocalLengthEnabled(void) const;
	void enableMinimumFocalLength(void);
	void disableMinimumFocalLength(void);

	bool isMaximumFocalLengthEnabled(void) const;
	void enableMaximumFocalLength(void);
	void disableMaximumFocalLength(void);

	bool isLightSensitivityLevelsEnabled(void) const;
	void enableLightSensitivityLevels(void);
	void disableLightSensitivityLevels(void);

	bool isImageStabilizationEnabled(void) const;
	void enableImageStabilization(void);
	void disableImageStabilization(void);


	uint16_t getSensorID(void);
	void setSensorID(uint16_t value);

	std::string getSensorName(void);
	void setSensorName(std::string value);

	SupportedStatesBitField& getSupportedStates(void);

	ZoomModesBitField& getZoomModes(void);

	FocusModesBitField& getFocusModes(void);

	WhiteBalanceBitField& getWhiteBalance(void);

	ImagingModesBitField& getImagingModes(void);

	ExposureModesBitField& getExposureModes(void);

	MeteringModesBitField& getMeteringModes(void);

	double getMinimumShutterSpeed_sec(void);
	void setMinimumShutterSpeed_sec(double value);

	double getMaximumShutterSpeed_sec(void);
	void setMaximumShutterSpeed_sec(double value);

	double getMinimumAperture(void);
	void setMinimumAperture(double value);

	double getMaximumAperture(void);
	void setMaximumAperture(double value);

	double getMinimumFocalLength_m(void);
	void setMinimumFocalLength_m(double value);

	double getMaximumFocalLength_m(void);
	void setMaximumFocalLength_m(double value);

	LightSensitivityLevelsBitField& getLightSensitivityLevels(void);

	ImageStabilizationEnumeration::ImageStabilizationEnum getImageStabilization(void);
	void setImageStabilization(ImageStabilizationEnumeration::ImageStabilizationEnum value);

protected:
	model::fields::UnsignedShort sensorID;
	model::fields::VariableLengthString sensorName;
	SupportedStatesBitField supportedStates;
	ZoomModesBitField zoomModes;
	FocusModesBitField focusModes;
	WhiteBalanceBitField whiteBalance;
	ImagingModesBitField imagingModes;
	ExposureModesBitField exposureModes;
	MeteringModesBitField meteringModes;
	MinimumShutterSpeedScaledInteger minimumShutterSpeed_sec;
	MaximumShutterSpeedScaledInteger maximumShutterSpeed_sec;
	MinimumApertureScaledInteger minimumAperture;
	MaximumApertureScaledInteger maximumAperture;
	MinimumFocalLengthScaledInteger minimumFocalLength_m;
	MaximumFocalLengthScaledInteger maximumFocalLength_m;
	LightSensitivityLevelsBitField lightSensitivityLevels;
	ImageStabilizationEnumeration imageStabilization;

	uint16_t presenceVector;
	enum pvEnum {SUPPORTEDSTATES = 0, ZOOMMODES = 1, FOCUSMODES = 2, WHITEBALANCE = 3, IMAGINGMODES = 4, EXPOSUREMODES = 5, METERINGMODES = 6, MINIMUMSHUTTERSPEED_SEC = 7, MAXIMUMSHUTTERSPEED_SEC = 8, MINIMUMAPERTURE = 9, MAXIMUMAPERTURE = 10, MINIMUMFOCALLENGTH_M = 11, MAXIMUMFOCALLENGTH_M = 12, LIGHTSENSITIVITYLEVELS = 13, IMAGESTABILIZATION = 14};
};

} // namespace environment
} // namespace openjaus

#endif // VISUALSENSORCAPABILITIESRECORD_H

