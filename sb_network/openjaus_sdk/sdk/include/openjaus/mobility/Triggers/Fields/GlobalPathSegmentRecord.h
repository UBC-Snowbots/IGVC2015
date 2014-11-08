
/**
\file GlobalPathSegmentRec.h

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

#ifndef GLOBALPATHSEGMENTRECORD_H
#define GLOBALPATHSEGMENTRECORD_H

#include <openjaus.h>
#include "openjaus/mobility/Triggers/Fields/JausLatitudeScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/JausLongitudeScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/JausAltitudeScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/JausLatitudeScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/JausLongitudeScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/JausAltitudeScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/WeightingFactorRefScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/PathToleranceRefScaledInteger.h"

namespace openjaus
{
namespace mobility
{

class OPENJAUS_EXPORT GlobalPathSegmentRecord : public openjaus::model::fields::Record
{
public:
	GlobalPathSegmentRecord();
	GlobalPathSegmentRecord(const GlobalPathSegmentRecord &source);
	~GlobalPathSegmentRecord();

	void copy(GlobalPathSegmentRecord& source);
	virtual int to(system::Buffer *dst);
	virtual int from(system::Buffer *src);
	virtual int length(void);
	std::string toXml(unsigned char level=0) const;
	
	void setPresenceVector(uint8_t value);
	uint8_t getPresenceVector(void) const;
	bool isP1AltitudeEnabled(void) const;
	void enableP1Altitude(void);
	void disableP1Altitude(void);

	bool isP2AltitudeEnabled(void) const;
	void enableP2Altitude(void);
	void disableP2Altitude(void);

	bool isPathToleranceEnabled(void) const;
	void enablePathTolerance(void);
	void disablePathTolerance(void);


	double getP1Latitude_deg(void);
	void setP1Latitude_deg(double value);

	double getP1Longitude_deg(void);
	void setP1Longitude_deg(double value);

	double getP1Altitude_m(void);
	void setP1Altitude_m(double value);

	double getP2Latitude_deg(void);
	void setP2Latitude_deg(double value);

	double getP2Longitude_deg(void);
	void setP2Longitude_deg(double value);

	double getP2Altitude_m(void);
	void setP2Altitude_m(double value);

	double getWeightingFactor(void);
	void setWeightingFactor(double value);

	double getPathTolerance(void);
	void setPathTolerance(double value);

protected:
	JausLatitudeScaledInteger p1Latitude_deg;
	JausLongitudeScaledInteger p1Longitude_deg;
	JausAltitudeScaledInteger p1Altitude_m;
	JausLatitudeScaledInteger p2Latitude_deg;
	JausLongitudeScaledInteger p2Longitude_deg;
	JausAltitudeScaledInteger p2Altitude_m;
	WeightingFactorRefScaledInteger weightingFactor;
	PathToleranceRefScaledInteger pathTolerance;

	uint8_t presenceVector;
	enum pvEnum {P1ALTITUDE_M = 0, P2ALTITUDE_M = 1, PATHTOLERANCE = 2};
};

} // namespace mobility
} // namespace openjaus

#endif // GLOBALPATHSEGMENTRECORD_H

