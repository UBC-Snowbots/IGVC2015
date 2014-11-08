
/**
\file PrismaticJoint1AngleRec.h

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

#ifndef PRISMATICJOINT1ANGLERECORD_H
#define PRISMATICJOINT1ANGLERECORD_H

#include <openjaus.h>
#include "openjaus/manipulator/Triggers/Fields/Joint1AngleScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/Joint1MinValueScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/Joint1MaxValueScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/Joint1MaxSpeedScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/Joint1MaxForceScaledInteger.h"

namespace openjaus
{
namespace manipulator
{

class OPENJAUS_EXPORT PrismaticJoint1AngleRecord : public openjaus::model::fields::Record
{
public:
	PrismaticJoint1AngleRecord();
	PrismaticJoint1AngleRecord(const PrismaticJoint1AngleRecord &source);
	~PrismaticJoint1AngleRecord();

	void copy(PrismaticJoint1AngleRecord& source);
	virtual int to(system::Buffer *dst);
	virtual int from(system::Buffer *src);
	virtual int length(void);
	std::string toXml(unsigned char level=0) const;
	
	void setPresenceVector(uint8_t value);
	uint8_t getPresenceVector(void) const;
	bool isJoint1MaxSpeedEnabled(void) const;
	void enableJoint1MaxSpeed(void);
	void disableJoint1MaxSpeed(void);

	bool isJoint1MaxForceEnabled(void) const;
	void enableJoint1MaxForce(void);
	void disableJoint1MaxForce(void);


	double getJoint1Angle_rad(void);
	void setJoint1Angle_rad(double value);

	double getJoint1MinValue_m(void);
	void setJoint1MinValue_m(double value);

	double getJoint1MaxValue_m(void);
	void setJoint1MaxValue_m(double value);

	double getJoint1MaxSpeed_mps(void);
	void setJoint1MaxSpeed_mps(double value);

	double getJoint1MaxForce(void);
	void setJoint1MaxForce(double value);

protected:
	Joint1AngleScaledInteger joint1Angle_rad;
	Joint1MinValueScaledInteger joint1MinValue_m;
	Joint1MaxValueScaledInteger joint1MaxValue_m;
	Joint1MaxSpeedScaledInteger joint1MaxSpeed_mps;
	Joint1MaxForceScaledInteger joint1MaxForce;

	uint8_t presenceVector;
	enum pvEnum {JOINT1MAXSPEED_MPS = 0, JOINT1MAXFORCE = 1};
};

} // namespace manipulator
} // namespace openjaus

#endif // PRISMATICJOINT1ANGLERECORD_H

