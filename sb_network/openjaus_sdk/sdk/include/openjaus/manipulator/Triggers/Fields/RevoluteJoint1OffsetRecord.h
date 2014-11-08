
/**
\file RevoluteJoint1OffsetRec.h

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

#ifndef REVOLUTEJOINT1OFFSETRECORD_H
#define REVOLUTEJOINT1OFFSETRECORD_H

#include <openjaus.h>
#include "openjaus/manipulator/Triggers/Fields/Joint1OffsetScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/Joint1MinValueScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/Joint1MaxValueScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/Joint1MaxSpeedScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/Joint1MaxTorqueScaledInteger.h"

namespace openjaus
{
namespace manipulator
{

class OPENJAUS_EXPORT RevoluteJoint1OffsetRecord : public openjaus::model::fields::Record
{
public:
	RevoluteJoint1OffsetRecord();
	RevoluteJoint1OffsetRecord(const RevoluteJoint1OffsetRecord &source);
	~RevoluteJoint1OffsetRecord();

	void copy(RevoluteJoint1OffsetRecord& source);
	virtual int to(system::Buffer *dst);
	virtual int from(system::Buffer *src);
	virtual int length(void);
	std::string toXml(unsigned char level=0) const;
	
	void setPresenceVector(uint8_t value);
	uint8_t getPresenceVector(void) const;
	bool isJoint1MinValueEnabled(void) const;
	void enableJoint1MinValue(void);
	void disableJoint1MinValue(void);

	bool isJoint1MaxValueEnabled(void) const;
	void enableJoint1MaxValue(void);
	void disableJoint1MaxValue(void);

	bool isJoint1MaxSpeedEnabled(void) const;
	void enableJoint1MaxSpeed(void);
	void disableJoint1MaxSpeed(void);

	bool isJoint1MaxTorqueEnabled(void) const;
	void enableJoint1MaxTorque(void);
	void disableJoint1MaxTorque(void);


	double getJoint1Offset_m(void);
	void setJoint1Offset_m(double value);

	double getJoint1MinValue_rad(void);
	void setJoint1MinValue_rad(double value);

	double getJoint1MaxValue_rad(void);
	void setJoint1MaxValue_rad(double value);

	double getJoint1MaxSpeed_rps(void);
	void setJoint1MaxSpeed_rps(double value);

	double getJoint1MaxTorque(void);
	void setJoint1MaxTorque(double value);

protected:
	Joint1OffsetScaledInteger joint1Offset_m;
	Joint1MinValueScaledInteger joint1MinValue_rad;
	Joint1MaxValueScaledInteger joint1MaxValue_rad;
	Joint1MaxSpeedScaledInteger joint1MaxSpeed_rps;
	Joint1MaxTorqueScaledInteger joint1MaxTorque;

	uint8_t presenceVector;
	enum pvEnum {JOINT1MINVALUE_RAD = 0, JOINT1MAXVALUE_RAD = 1, JOINT1MAXSPEED_RPS = 2, JOINT1MAXTORQUE = 3};
};

} // namespace manipulator
} // namespace openjaus

#endif // REVOLUTEJOINT1OFFSETRECORD_H

