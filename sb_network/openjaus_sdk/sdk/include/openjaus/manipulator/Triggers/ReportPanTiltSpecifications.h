
/**
\file ReportPanTiltSpecifications.h

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

#ifndef REPORTPANTILTSPECIFICATIONS_H
#define REPORTPANTILTSPECIFICATIONS_H

#include <openjaus.h>

#include "openjaus/manipulator/Triggers/Fields/PanTiltCoordinateSysXScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/PanTiltCoordinateSysYScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/PanTiltCoordinateSysZScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/DComponentOfUnitQuaternionQScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/AComponentOfUnitQuaternionQScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/BComponentOfUnitQuaternionQScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/CComponentOfUnitQuaternionQScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/Joint1MinValueScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/Joint1MaxValueScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/Joint1MaxSpeedScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/Joint2MinValueScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/Joint2MaxValueScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/Joint2MaxSpeedScaledInteger.h"

namespace openjaus
{
namespace manipulator
{

/// \class ReportPanTiltSpecifications ReportPanTiltSpecifications.h
/// \brief ReportPanTiltSpecifications Message Implementation.
/// This message provides the joint angle and joint velocity limits for the pan tilt mechanism.
class OPENJAUS_EXPORT ReportPanTiltSpecifications : public model::Message
{
public:
	ReportPanTiltSpecifications();
	ReportPanTiltSpecifications(model::Message *message);
	~ReportPanTiltSpecifications();
	
	static const uint16_t ID = 0x4620;

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
	bool isPanTiltCoordinateSysXEnabled(void) const;
	void enablePanTiltCoordinateSysX(void);
	void disablePanTiltCoordinateSysX(void);

	bool isPanTiltCoordinateSysYEnabled(void) const;
	void enablePanTiltCoordinateSysY(void);
	void disablePanTiltCoordinateSysY(void);

	bool isPanTiltCoordinateSysZEnabled(void) const;
	void enablePanTiltCoordinateSysZ(void);
	void disablePanTiltCoordinateSysZ(void);

	bool isDComponentOfUnitQuaternionQEnabled(void) const;
	void enableDComponentOfUnitQuaternionQ(void);
	void disableDComponentOfUnitQuaternionQ(void);

	bool isAComponentOfUnitQuaternionQEnabled(void) const;
	void enableAComponentOfUnitQuaternionQ(void);
	void disableAComponentOfUnitQuaternionQ(void);

	bool isBComponentOfUnitQuaternionQEnabled(void) const;
	void enableBComponentOfUnitQuaternionQ(void);
	void disableBComponentOfUnitQuaternionQ(void);

	bool isCComponentOfUnitQuaternionQEnabled(void) const;
	void enableCComponentOfUnitQuaternionQ(void);
	void disableCComponentOfUnitQuaternionQ(void);



	double getPanTiltCoordinateSysX_m(void);
	void setPanTiltCoordinateSysX_m(double value);

	double getPanTiltCoordinateSysY_m(void);
	void setPanTiltCoordinateSysY_m(double value);

	double getPanTiltCoordinateSysZ_m(void);
	void setPanTiltCoordinateSysZ_m(double value);

	double getDComponentOfUnitQuaternionQ(void);
	void setDComponentOfUnitQuaternionQ(double value);

	double getAComponentOfUnitQuaternionQ(void);
	void setAComponentOfUnitQuaternionQ(double value);

	double getBComponentOfUnitQuaternionQ(void);
	void setBComponentOfUnitQuaternionQ(double value);

	double getCComponentOfUnitQuaternionQ(void);
	void setCComponentOfUnitQuaternionQ(double value);

	double getJoint1MinValue_rad(void);
	void setJoint1MinValue_rad(double value);

	double getJoint1MaxValue_rad(void);
	void setJoint1MaxValue_rad(double value);

	double getJoint1MaxSpeed_rps(void);
	void setJoint1MaxSpeed_rps(double value);

	double getJoint2MinValue_rad(void);
	void setJoint2MinValue_rad(double value);

	double getJoint2MaxValue_rad(void);
	void setJoint2MaxValue_rad(double value);

	double getJoint2MaxSpeed_rps(void);
	void setJoint2MaxSpeed_rps(double value);

private:
	PanTiltCoordinateSysXScaledInteger panTiltCoordinateSysX_m;
	PanTiltCoordinateSysYScaledInteger panTiltCoordinateSysY_m;
	PanTiltCoordinateSysZScaledInteger panTiltCoordinateSysZ_m;
	DComponentOfUnitQuaternionQScaledInteger dComponentOfUnitQuaternionQ;
	AComponentOfUnitQuaternionQScaledInteger aComponentOfUnitQuaternionQ;
	BComponentOfUnitQuaternionQScaledInteger bComponentOfUnitQuaternionQ;
	CComponentOfUnitQuaternionQScaledInteger cComponentOfUnitQuaternionQ;
	Joint1MinValueScaledInteger joint1MinValue_rad;
	Joint1MaxValueScaledInteger joint1MaxValue_rad;
	Joint1MaxSpeedScaledInteger joint1MaxSpeed_rps;
	Joint2MinValueScaledInteger joint2MinValue_rad;
	Joint2MaxValueScaledInteger joint2MaxValue_rad;
	Joint2MaxSpeedScaledInteger joint2MaxSpeed_rps;

	uint8_t presenceVector;
	enum pvEnum {PANTILTCOORDINATESYSX_M = 0, PANTILTCOORDINATESYSY_M = 1, PANTILTCOORDINATESYSZ_M = 2, DCOMPONENTOFUNITQUATERNIONQ = 3, ACOMPONENTOFUNITQUATERNIONQ = 4, BCOMPONENTOFUNITQUATERNIONQ = 5, CCOMPONENTOFUNITQUATERNIONQ = 6};
};

} // namespace manipulator
} // namespace openjaus

#endif // REPORTPANTILTSPECIFICATIONS_H


