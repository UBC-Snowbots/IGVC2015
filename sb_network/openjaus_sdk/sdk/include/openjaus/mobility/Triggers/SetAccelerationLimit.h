
/**
\file SetAccelerationLimit.h

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

#ifndef SETACCELERATIONLIMIT_H
#define SETACCELERATIONLIMIT_H

#include <openjaus.h>

#include "openjaus/mobility/Triggers/Fields/AccelerationCommandTypeEnumeration.h"
#include "openjaus/mobility/Triggers/Fields/LinearAccelerationScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/LinearAccelerationScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/LinearAccelerationScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/AngularAccelerationScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/AngularAccelerationScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/AngularAccelerationScaledInteger.h"

namespace openjaus
{
namespace mobility
{

/// \class SetAccelerationLimit SetAccelerationLimit.h
/// \brief SetAccelerationLimit Message Implementation.
/// This message is used to command the linear and rotational acceleration limits of the platform.
class OPENJAUS_EXPORT SetAccelerationLimit : public model::Message
{
public:
	SetAccelerationLimit();
	SetAccelerationLimit(model::Message *message);
	~SetAccelerationLimit();
	
	static const uint16_t ID = 0x0416;

	/// \brief Pack this message to the given openjaus::system::Buffer. 
	/// \copybrief
	/// \param[out] dst - The destination openjaus::system::Buffer to which this message will be packed.
	/// \return The number of bytes packed into the destination buffer
	virtual int to(system::Buffer *dst);	

	/// \brief Unpack this message from the given openjaus::system::Buffer.
	/// \copybrief
	/// \param[in] src - The source openjaus::system::Buffer from which this message will be unpacked.
	/// \return The number of bytes unpacked from the source buffer
	virtual int from(system::Buffer *src);

	/// \brief Get the number of bytes this message would occupy in a serialized buffer. 
	/// \copybrief
	/// \return The number of bytes this message would occupy in a buffer
	virtual int length();
	
	/// \brief Used to serialize the runtime state of the message to an XML format. 
	/// \copybrief
	/// \param[in] level - Used to determine how many tabs are inserted per line for pretty formating. 
	/// \return The serialized XML string
	std::string toXml(unsigned char level=0) const;

	void setPresenceVector(uint8_t value);
	uint8_t getPresenceVector(void) const;
	bool isAccelerationXEnabled(void) const;
	void enableAccelerationX(void);
	void disableAccelerationX(void);
	bool isAccelerationYEnabled(void) const;
	void enableAccelerationY(void);
	void disableAccelerationY(void);
	bool isAccelerationZEnabled(void) const;
	void enableAccelerationZ(void);
	void disableAccelerationZ(void);
	bool isRollAccelerationEnabled(void) const;
	void enableRollAcceleration(void);
	void disableRollAcceleration(void);
	bool isPitchAccelerationEnabled(void) const;
	void enablePitchAcceleration(void);
	void disablePitchAcceleration(void);
	bool isYawAccelerationEnabled(void) const;
	void enableYawAcceleration(void);
	void disableYawAcceleration(void);


	AccelerationCommandTypeEnumeration::AccelerationCommandTypeEnum getCommandType(void);
	void setCommandType(AccelerationCommandTypeEnumeration::AccelerationCommandTypeEnum value);

	double getAccelerationX_mps2(void);
	void setAccelerationX_mps2(double value);

	double getAccelerationY_mps2(void);
	void setAccelerationY_mps2(double value);

	double getAccelerationZ_mps2(void);
	void setAccelerationZ_mps2(double value);

	double getRollAcceleration_rps2(void);
	void setRollAcceleration_rps2(double value);

	double getPitchAcceleration_rps2(void);
	void setPitchAcceleration_rps2(double value);

	double getYawAcceleration_rps2(void);
	void setYawAcceleration_rps2(double value);

private:
	AccelerationCommandTypeEnumeration commandType;
	LinearAccelerationScaledInteger accelerationX_mps2;
	LinearAccelerationScaledInteger accelerationY_mps2;
	LinearAccelerationScaledInteger accelerationZ_mps2;
	AngularAccelerationScaledInteger rollAcceleration_rps2;
	AngularAccelerationScaledInteger pitchAcceleration_rps2;
	AngularAccelerationScaledInteger yawAcceleration_rps2;

	uint8_t presenceVector;
	enum pvEnum {ACCELERATIONX_MPS2 = 0, ACCELERATIONY_MPS2 = 1, ACCELERATIONZ_MPS2 = 2, ROLLACCELERATION_RPS2 = 3, PITCHACCELERATION_RPS2 = 4, YAWACCELERATION_RPS2 = 5};
};

} // namespace mobility
} // namespace openjaus

#endif // SETACCELERATIONLIMIT_H


