
/**
\file RevoluteJointSpecificationRec.h

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

#ifndef REVOLUTEJOINTSPECIFICATIONRECORD_H
#define REVOLUTEJOINTSPECIFICATIONRECORD_H

#include <openjaus.h>
#include "openjaus/manipulator/Triggers/Fields/LinkLengthScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/TwistAngleScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/JointOffsetScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/MinValueScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/MaxValueScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/MaxSpeedScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/MaxTorqueScaledInteger.h"

namespace openjaus
{
namespace manipulator
{

class OPENJAUS_EXPORT RevoluteJointSpecificationRecord : public openjaus::model::fields::Record
{
public:
	RevoluteJointSpecificationRecord();
	RevoluteJointSpecificationRecord(const RevoluteJointSpecificationRecord &source);
	~RevoluteJointSpecificationRecord();

	void copy(RevoluteJointSpecificationRecord& source);
	virtual int to(system::Buffer *dst);
	virtual int from(system::Buffer *src);
	virtual int length(void);
	std::string toXml(unsigned char level=0) const;
	
	void setPresenceVector(uint8_t value);
	uint8_t getPresenceVector(void) const;
	bool isMinValueEnabled(void) const;
	void enableMinValue(void);
	void disableMinValue(void);

	bool isMaxValueEnabled(void) const;
	void enableMaxValue(void);
	void disableMaxValue(void);

	bool isMaxSpeedEnabled(void) const;
	void enableMaxSpeed(void);
	void disableMaxSpeed(void);

	bool isMaxTorqueEnabled(void) const;
	void enableMaxTorque(void);
	void disableMaxTorque(void);


	double getLinkLength_m(void);
	void setLinkLength_m(double value);

	double getTwistAngle_rad(void);
	void setTwistAngle_rad(double value);

	double getJointOffset_m(void);
	void setJointOffset_m(double value);

	double getMinValue_rad(void);
	void setMinValue_rad(double value);

	double getMaxValue_rad(void);
	void setMaxValue_rad(double value);

	double getMaxSpeed_rps(void);
	void setMaxSpeed_rps(double value);

	double getMaxTorque(void);
	void setMaxTorque(double value);

protected:
	LinkLengthScaledInteger linkLength_m;
	TwistAngleScaledInteger twistAngle_rad;
	JointOffsetScaledInteger jointOffset_m;
	MinValueScaledInteger minValue_rad;
	MaxValueScaledInteger maxValue_rad;
	MaxSpeedScaledInteger maxSpeed_rps;
	MaxTorqueScaledInteger maxTorque;

	uint8_t presenceVector;
	enum pvEnum {MINVALUE_RAD = 0, MAXVALUE_RAD = 1, MAXSPEED_RPS = 2, MAXTORQUE = 3};
};

} // namespace manipulator
} // namespace openjaus

#endif // REVOLUTEJOINTSPECIFICATIONRECORD_H

