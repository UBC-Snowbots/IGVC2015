
/**
\file ReportVelocityState.h

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

#ifndef REPORTVELOCITYSTATE_H
#define REPORTVELOCITYSTATE_H

#include <openjaus.h>

#include "openjaus/mobility/Triggers/Fields/LinearVelocityScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/LinearVelocityScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/LinearVelocityScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/LinearVelocityRmsScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/AngularVelocityScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/AngularVelocityScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/AngularVelocityScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/AngularVelocityRmsScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/JausTimeStampBitField.h"

namespace openjaus
{
namespace mobility
{

/// \class ReportVelocityState ReportVelocityState.h
/// \brief ReportVelocityState Message Implementation.
/// This message is used to provide the receiver the linear velocity and rotational rate of the platform
class OPENJAUS_EXPORT ReportVelocityState : public model::Message
{
public:
	ReportVelocityState();
	ReportVelocityState(model::Message *message);
	~ReportVelocityState();
	
	static const uint16_t ID = 0x4404;

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
	bool isVelocityXEnabled(void) const;
	void enableVelocityX(void);
	void disableVelocityX(void);

	bool isVelocityYEnabled(void) const;
	void enableVelocityY(void);
	void disableVelocityY(void);

	bool isVelocityZEnabled(void) const;
	void enableVelocityZ(void);
	void disableVelocityZ(void);

	bool isVelocityRmsEnabled(void) const;
	void enableVelocityRms(void);
	void disableVelocityRms(void);

	bool isRollRateEnabled(void) const;
	void enableRollRate(void);
	void disableRollRate(void);

	bool isPitchRateEnabled(void) const;
	void enablePitchRate(void);
	void disablePitchRate(void);

	bool isYawRateEnabled(void) const;
	void enableYawRate(void);
	void disableYawRate(void);

	bool isTimeStampEnabled(void) const;
	void enableTimeStamp(void);
	void disableTimeStamp(void);



	double getVelocityX_mps(void);
	void setVelocityX_mps(double value);

	double getVelocityY_mps(void);
	void setVelocityY_mps(double value);

	double getVelocityZ_mps(void);
	void setVelocityZ_mps(double value);

	double getVelocityRms_mps(void);
	void setVelocityRms_mps(double value);

	double getRollRate_rps(void);
	void setRollRate_rps(double value);

	double getPitchRate_rps(void);
	void setPitchRate_rps(double value);

	double getYawRate_rps(void);
	void setYawRate_rps(double value);

	double getRateRms_rps(void);
	void setRateRms_rps(double value);

	JausTimeStampBitField& getTimeStamp(void);

private:
	LinearVelocityScaledInteger velocityX_mps;
	LinearVelocityScaledInteger velocityY_mps;
	LinearVelocityScaledInteger velocityZ_mps;
	LinearVelocityRmsScaledInteger velocityRms_mps;
	AngularVelocityScaledInteger rollRate_rps;
	AngularVelocityScaledInteger pitchRate_rps;
	AngularVelocityScaledInteger yawRate_rps;
	AngularVelocityRmsScaledInteger rateRms_rps;
	JausTimeStampBitField timeStamp;

	uint8_t presenceVector;
	enum pvEnum {VELOCITYX_MPS = 0, VELOCITYY_MPS = 1, VELOCITYZ_MPS = 2, VELOCITYRMS_MPS = 3, ROLLRATE_RPS = 4, PITCHRATE_RPS = 5, YAWRATE_RPS = 6, TIMESTAMP = 7};
};

} // namespace mobility
} // namespace openjaus

#endif // REPORTVELOCITYSTATE_H


