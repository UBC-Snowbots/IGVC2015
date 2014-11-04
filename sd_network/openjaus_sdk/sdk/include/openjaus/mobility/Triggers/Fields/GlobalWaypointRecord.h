
/**
\file GlobalWaypointRec.h

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

#ifndef GLOBALWAYPOINTRECORD_H
#define GLOBALWAYPOINTRECORD_H

#include <openjaus.h>
#include "openjaus/mobility/Triggers/Fields/JausLatitudeScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/JausLongitudeScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/JausAltitudeScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/OrientationScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/OrientationScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/OrientationScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/WaypointToleranceRefScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/PathToleranceRefScaledInteger.h"

namespace openjaus
{
namespace mobility
{

class OPENJAUS_EXPORT GlobalWaypointRecord : public openjaus::model::fields::Record
{
public:
	GlobalWaypointRecord();
	GlobalWaypointRecord(const GlobalWaypointRecord &source);
	~GlobalWaypointRecord();

	void copy(GlobalWaypointRecord& source);
	virtual int to(system::Buffer *dst);
	virtual int from(system::Buffer *src);
	virtual int length(void);
	std::string toXml(unsigned char level=0) const;
	
	void setPresenceVector(uint8_t value);
	uint8_t getPresenceVector(void) const;
	bool isAltitudeEnabled(void) const;
	void enableAltitude(void);
	void disableAltitude(void);

	bool isRollEnabled(void) const;
	void enableRoll(void);
	void disableRoll(void);

	bool isPitchEnabled(void) const;
	void enablePitch(void);
	void disablePitch(void);

	bool isYawEnabled(void) const;
	void enableYaw(void);
	void disableYaw(void);

	bool isWaypointToleranceEnabled(void) const;
	void enableWaypointTolerance(void);
	void disableWaypointTolerance(void);

	bool isPathToleranceEnabled(void) const;
	void enablePathTolerance(void);
	void disablePathTolerance(void);


	double getLatitude_deg(void);
	void setLatitude_deg(double value);

	double getLongitude_deg(void);
	void setLongitude_deg(double value);

	double getAltitude_m(void);
	void setAltitude_m(double value);

	double getRoll_rad(void);
	void setRoll_rad(double value);

	double getPitch_rad(void);
	void setPitch_rad(double value);

	double getYaw_rad(void);
	void setYaw_rad(double value);

	double getWaypointTolerance_m(void);
	void setWaypointTolerance_m(double value);

	double getPathTolerance(void);
	void setPathTolerance(double value);

protected:
	JausLatitudeScaledInteger latitude_deg;
	JausLongitudeScaledInteger longitude_deg;
	JausAltitudeScaledInteger altitude_m;
	OrientationScaledInteger roll_rad;
	OrientationScaledInteger pitch_rad;
	OrientationScaledInteger yaw_rad;
	WaypointToleranceRefScaledInteger waypointTolerance_m;
	PathToleranceRefScaledInteger pathTolerance;

	uint8_t presenceVector;
	enum pvEnum {ALTITUDE_M = 0, ROLL_RAD = 1, PITCH_RAD = 2, YAW_RAD = 3, WAYPOINTTOLERANCE_M = 4, PATHTOLERANCE = 5};
};

} // namespace mobility
} // namespace openjaus

#endif // GLOBALWAYPOINTRECORD_H

