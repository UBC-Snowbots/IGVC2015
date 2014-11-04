
/**
\file RangeSensorDataPointRec.h

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

#ifndef RANGESENSORDATAPOINTRECORD_H
#define RANGESENSORDATAPOINTRECORD_H

#include <openjaus.h>
#include "openjaus/environment/Triggers/Fields/RangeScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/RangeValidityEnumeration.h"
#include "openjaus/environment/Triggers/Fields/RangeErrorRMSScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/BearingScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/BearingValidityEnumeration.h"
#include "openjaus/environment/Triggers/Fields/BearingErrorRMSScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/InclinationScaledInteger.h"
#include "openjaus/environment/Triggers/Fields/InclinationValidityEnumeration.h"
#include "openjaus/environment/Triggers/Fields/InclinationErrorRMSScaledInteger.h"

namespace openjaus
{
namespace environment
{

class OPENJAUS_EXPORT RangeSensorDataPointRecord : public openjaus::model::fields::Record
{
public:
	RangeSensorDataPointRecord();
	RangeSensorDataPointRecord(const RangeSensorDataPointRecord &source);
	~RangeSensorDataPointRecord();

	void copy(RangeSensorDataPointRecord& source);
	virtual int to(system::Buffer *dst);
	virtual int from(system::Buffer *src);
	virtual int length(void);
	std::string toXml(unsigned char level=0) const;
	
	void setPresenceVector(uint8_t value);
	uint8_t getPresenceVector(void) const;
	bool isPointIDEnabled(void) const;
	void enablePointID(void);
	void disablePointID(void);

	bool isRangeValidityEnabled(void) const;
	void enableRangeValidity(void);
	void disableRangeValidity(void);

	bool isRangeErrorRMSEnabled(void) const;
	void enableRangeErrorRMS(void);
	void disableRangeErrorRMS(void);

	bool isBearingValidityEnabled(void) const;
	void enableBearingValidity(void);
	void disableBearingValidity(void);

	bool isBearingErrorRMSEnabled(void) const;
	void enableBearingErrorRMS(void);
	void disableBearingErrorRMS(void);

	bool isInclinationValidityEnabled(void) const;
	void enableInclinationValidity(void);
	void disableInclinationValidity(void);

	bool isInclinationErrorRMSEnabled(void) const;
	void enableInclinationErrorRMS(void);
	void disableInclinationErrorRMS(void);


	uint32_t getPointID(void);
	void setPointID(uint32_t value);

	double getRange_m(void);
	void setRange_m(double value);

	RangeValidityEnumeration::RangeValidityEnum getRangeValidity(void);
	void setRangeValidity(RangeValidityEnumeration::RangeValidityEnum value);

	double getRangeErrorRMS_m(void);
	void setRangeErrorRMS_m(double value);

	double getBearing_rad(void);
	void setBearing_rad(double value);

	BearingValidityEnumeration::BearingValidityEnum getBearingValidity(void);
	void setBearingValidity(BearingValidityEnumeration::BearingValidityEnum value);

	double getBearingErrorRMS_rad(void);
	void setBearingErrorRMS_rad(double value);

	double getInclination_rad(void);
	void setInclination_rad(double value);

	InclinationValidityEnumeration::InclinationValidityEnum getInclinationValidity(void);
	void setInclinationValidity(InclinationValidityEnumeration::InclinationValidityEnum value);

	double getInclinationErrorRMS_rad(void);
	void setInclinationErrorRMS_rad(double value);

protected:
	model::fields::UnsignedInteger pointID;
	RangeScaledInteger range_m;
	RangeValidityEnumeration rangeValidity;
	RangeErrorRMSScaledInteger rangeErrorRMS_m;
	BearingScaledInteger bearing_rad;
	BearingValidityEnumeration bearingValidity;
	BearingErrorRMSScaledInteger bearingErrorRMS_rad;
	InclinationScaledInteger inclination_rad;
	InclinationValidityEnumeration inclinationValidity;
	InclinationErrorRMSScaledInteger inclinationErrorRMS_rad;

	uint8_t presenceVector;
	enum pvEnum {POINTID = 0, RANGEVALIDITY = 1, RANGEERRORRMS_M = 2, BEARINGVALIDITY = 3, BEARINGERRORRMS_RAD = 4, INCLINATIONVALIDITY = 5, INCLINATIONERRORRMS_RAD = 6};
};

} // namespace environment
} // namespace openjaus

#endif // RANGESENSORDATAPOINTRECORD_H

