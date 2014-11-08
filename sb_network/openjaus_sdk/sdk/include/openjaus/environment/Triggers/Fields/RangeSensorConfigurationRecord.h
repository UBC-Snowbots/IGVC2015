
/**
\file RangeSensorConfigurationRec.h

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

#ifndef RANGESENSORCONFIGURATIONRECORD_H
#define RANGESENSORCONFIGURATIONRECORD_H

#include <openjaus.h>
#include "openjaus/environment/Triggers/Fields/HorizontalFieldOfViewStartAngleScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/HorizontalFieldOfViewStopAngleScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/VerticalFieldOfViewStartAngleScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/VerticalFieldOfViewStopAngleScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/UpdateRateScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/MinimumRangeScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/MaximumRangeScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/SensorStateEnumeration.h"

namespace openjaus
{
namespace environment
{

class OPENJAUS_EXPORT RangeSensorConfigurationRecord : public openjaus::model::fields::Record
{
public:
	RangeSensorConfigurationRecord();
	RangeSensorConfigurationRecord(const RangeSensorConfigurationRecord &source);
	~RangeSensorConfigurationRecord();

	void copy(RangeSensorConfigurationRecord& source);
	virtual int to(system::Buffer *dst);
	virtual int from(system::Buffer *src);
	virtual int length(void);
	std::string toXml(unsigned char level=0) const;
	
	void setPresenceVector(uint8_t value);
	uint8_t getPresenceVector(void) const;
	bool isHorizontalFieldOfViewStartAngleEnabled(void) const;
	void enableHorizontalFieldOfViewStartAngle(void);
	void disableHorizontalFieldOfViewStartAngle(void);

	bool isHorizontalFieldOfViewStopAngleEnabled(void) const;
	void enableHorizontalFieldOfViewStopAngle(void);
	void disableHorizontalFieldOfViewStopAngle(void);

	bool isVerticalFieldOfViewStartAngleEnabled(void) const;
	void enableVerticalFieldOfViewStartAngle(void);
	void disableVerticalFieldOfViewStartAngle(void);

	bool isVerticalFieldOfViewStopAngleEnabled(void) const;
	void enableVerticalFieldOfViewStopAngle(void);
	void disableVerticalFieldOfViewStopAngle(void);

	bool isUpdateRateEnabled(void) const;
	void enableUpdateRate(void);
	void disableUpdateRate(void);

	bool isMinimumRangeEnabled(void) const;
	void enableMinimumRange(void);
	void disableMinimumRange(void);

	bool isMaximumRangeEnabled(void) const;
	void enableMaximumRange(void);
	void disableMaximumRange(void);

	bool isSensorStateEnabled(void) const;
	void enableSensorState(void);
	void disableSensorState(void);


	uint16_t getSensorID(void);
	void setSensorID(uint16_t value);

	double getHorizontalFieldOfViewStartAngle_rad(void);
	void setHorizontalFieldOfViewStartAngle_rad(double value);

	double getHorizontalFieldOfViewStopAngle_rad(void);
	void setHorizontalFieldOfViewStopAngle_rad(double value);

	double getVerticalFieldOfViewStartAngle_rad(void);
	void setVerticalFieldOfViewStartAngle_rad(double value);

	double getVerticalFieldOfViewStopAngle_rad(void);
	void setVerticalFieldOfViewStopAngle_rad(double value);

	double getUpdateRate_Hz(void);
	void setUpdateRate_Hz(double value);

	double getMinimumRange_m(void);
	void setMinimumRange_m(double value);

	double getMaximumRange_m(void);
	void setMaximumRange_m(double value);

	SensorStateEnumeration::SensorStateEnum getSensorState(void);
	void setSensorState(SensorStateEnumeration::SensorStateEnum value);

protected:
	model::fields::UnsignedShort sensorID;
	HorizontalFieldOfViewStartAngleScaledInteger horizontalFieldOfViewStartAngle_rad;
	HorizontalFieldOfViewStopAngleScaledInteger horizontalFieldOfViewStopAngle_rad;
	VerticalFieldOfViewStartAngleScaledInteger verticalFieldOfViewStartAngle_rad;
	VerticalFieldOfViewStopAngleScaledInteger verticalFieldOfViewStopAngle_rad;
	UpdateRateScaledInteger updateRate_Hz;
	MinimumRangeScaledInteger minimumRange_m;
	MaximumRangeScaledInteger maximumRange_m;
	SensorStateEnumeration sensorState;

	uint8_t presenceVector;
	enum pvEnum {HORIZONTALFIELDOFVIEWSTARTANGLE_RAD = 0, HORIZONTALFIELDOFVIEWSTOPANGLE_RAD = 1, VERTICALFIELDOFVIEWSTARTANGLE_RAD = 2, VERTICALFIELDOFVIEWSTOPANGLE_RAD = 3, UPDATERATE_HZ = 4, MINIMUMRANGE_M = 5, MAXIMUMRANGE_M = 6, SENSORSTATE = 7};
};

} // namespace environment
} // namespace openjaus

#endif // RANGESENSORCONFIGURATIONRECORD_H

