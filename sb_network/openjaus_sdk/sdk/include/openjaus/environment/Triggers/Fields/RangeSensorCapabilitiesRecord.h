
/**
\file RangeSensorCapabilitiesRec.h

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

#ifndef RANGESENSORCAPABILITIESRECORD_H
#define RANGESENSORCAPABILITIESRECORD_H

#include <openjaus.h>
#include "openjaus/environment/Triggers/Fields/SupportedStatesBitField.h"
#include "openjaus/environment/Triggers/Fields/MinimumHorizontalFieldOfViewStartAngleScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/MaximumHorizontalFieldOfViewStopAngleScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/MinimumVerticalFieldOfViewStartAngleScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/MaximumVerticalFieldOfViewStopAngleScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/MiniumumUpdateRateScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/MaximumUpdateRateScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/MinimumRangeScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/MaximumRangeScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/SupportedCompressionBitField.h"
#include "openjaus/environment/Triggers/Fields/CoordinateTransformationSupportedEnumeration.h"

namespace openjaus
{
namespace environment
{

class OPENJAUS_EXPORT RangeSensorCapabilitiesRecord : public openjaus::model::fields::Record
{
public:
	RangeSensorCapabilitiesRecord();
	RangeSensorCapabilitiesRecord(const RangeSensorCapabilitiesRecord &source);
	~RangeSensorCapabilitiesRecord();

	void copy(RangeSensorCapabilitiesRecord& source);
	virtual int to(system::Buffer *dst);
	virtual int from(system::Buffer *src);
	virtual int length(void);
	std::string toXml(unsigned char level=0) const;
	
	void setPresenceVector(uint16_t value);
	uint16_t getPresenceVector(void) const;
	bool isSupportedStatesEnabled(void) const;
	void enableSupportedStates(void);
	void disableSupportedStates(void);

	bool isMinimumHorizontalFieldOfViewStartAngleEnabled(void) const;
	void enableMinimumHorizontalFieldOfViewStartAngle(void);
	void disableMinimumHorizontalFieldOfViewStartAngle(void);

	bool isMaximumHorizontalFieldOfViewStopAngleEnabled(void) const;
	void enableMaximumHorizontalFieldOfViewStopAngle(void);
	void disableMaximumHorizontalFieldOfViewStopAngle(void);

	bool isMinimumVerticalFieldOfViewStartAngleEnabled(void) const;
	void enableMinimumVerticalFieldOfViewStartAngle(void);
	void disableMinimumVerticalFieldOfViewStartAngle(void);

	bool isMaximumVerticalFieldOfViewStopAngleEnabled(void) const;
	void enableMaximumVerticalFieldOfViewStopAngle(void);
	void disableMaximumVerticalFieldOfViewStopAngle(void);

	bool isMiniumumUpdateRateEnabled(void) const;
	void enableMiniumumUpdateRate(void);
	void disableMiniumumUpdateRate(void);

	bool isMaximumUpdateRateEnabled(void) const;
	void enableMaximumUpdateRate(void);
	void disableMaximumUpdateRate(void);

	bool isMinimumRangeEnabled(void) const;
	void enableMinimumRange(void);
	void disableMinimumRange(void);

	bool isMaximumRangeEnabled(void) const;
	void enableMaximumRange(void);
	void disableMaximumRange(void);

	bool isSupportedCompressionEnabled(void) const;
	void enableSupportedCompression(void);
	void disableSupportedCompression(void);

	bool isCoordinateTransformationSupportedEnabled(void) const;
	void enableCoordinateTransformationSupported(void);
	void disableCoordinateTransformationSupported(void);


	uint16_t getSensorID(void);
	void setSensorID(uint16_t value);

	std::string getSensorName(void);
	void setSensorName(std::string value);

	SupportedStatesBitField& getSupportedStates(void);

	double getMinimumHorizontalFieldOfViewStartAngle_rad(void);
	void setMinimumHorizontalFieldOfViewStartAngle_rad(double value);

	double getMaximumHorizontalFieldOfViewStopAngle_rad(void);
	void setMaximumHorizontalFieldOfViewStopAngle_rad(double value);

	double getMinimumVerticalFieldOfViewStartAngle_rad(void);
	void setMinimumVerticalFieldOfViewStartAngle_rad(double value);

	double getMaximumVerticalFieldOfViewStopAngle_rad(void);
	void setMaximumVerticalFieldOfViewStopAngle_rad(double value);

	double getMiniumumUpdateRate_Hz(void);
	void setMiniumumUpdateRate_Hz(double value);

	double getMaximumUpdateRate_Hz(void);
	void setMaximumUpdateRate_Hz(double value);

	double getMinimumRange_m(void);
	void setMinimumRange_m(double value);

	double getMaximumRange_m(void);
	void setMaximumRange_m(double value);

	SupportedCompressionBitField& getSupportedCompression(void);

	CoordinateTransformationSupportedEnumeration::CoordinateTransformationSupportedEnum getCoordinateTransformationSupported(void);
	void setCoordinateTransformationSupported(CoordinateTransformationSupportedEnumeration::CoordinateTransformationSupportedEnum value);

protected:
	model::fields::UnsignedShort sensorID;
	model::fields::VariableLengthString sensorName;
	SupportedStatesBitField supportedStates;
	MinimumHorizontalFieldOfViewStartAngleScaledInteger minimumHorizontalFieldOfViewStartAngle_rad;
	MaximumHorizontalFieldOfViewStopAngleScaledInteger maximumHorizontalFieldOfViewStopAngle_rad;
	MinimumVerticalFieldOfViewStartAngleScaledInteger minimumVerticalFieldOfViewStartAngle_rad;
	MaximumVerticalFieldOfViewStopAngleScaledInteger maximumVerticalFieldOfViewStopAngle_rad;
	MiniumumUpdateRateScaledInteger miniumumUpdateRate_Hz;
	MaximumUpdateRateScaledInteger maximumUpdateRate_Hz;
	MinimumRangeScaledInteger minimumRange_m;
	MaximumRangeScaledInteger maximumRange_m;
	SupportedCompressionBitField supportedCompression;
	CoordinateTransformationSupportedEnumeration coordinateTransformationSupported;

	uint16_t presenceVector;
	enum pvEnum {SUPPORTEDSTATES = 0, MINIMUMHORIZONTALFIELDOFVIEWSTARTANGLE_RAD = 1, MAXIMUMHORIZONTALFIELDOFVIEWSTOPANGLE_RAD = 2, MINIMUMVERTICALFIELDOFVIEWSTARTANGLE_RAD = 3, MAXIMUMVERTICALFIELDOFVIEWSTOPANGLE_RAD = 4, MINIUMUMUPDATERATE_HZ = 5, MAXIMUMUPDATERATE_HZ = 6, MINIMUMRANGE_M = 7, MAXIMUMRANGE_M = 8, SUPPORTEDCOMPRESSION = 9, COORDINATETRANSFORMATIONSUPPORTED = 10};
};

} // namespace environment
} // namespace openjaus

#endif // RANGESENSORCAPABILITIESRECORD_H

