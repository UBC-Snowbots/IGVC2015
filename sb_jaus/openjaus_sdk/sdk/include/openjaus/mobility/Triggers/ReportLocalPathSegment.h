
/**
\file ReportLocalPathSegment.h

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

#ifndef REPORTLOCALPATHSEGMENT_H
#define REPORTLOCALPATHSEGMENT_H

#include <openjaus.h>

#include "openjaus/mobility/Triggers/Fields/LocalPositionScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/LocalPositionScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/JausAltitudeScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/LocalPositionScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/LocalPositionScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/JausAltitudeScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/WeightingFactorRefScaledInteger.h"
#include "openjaus/mobility/Triggers/Fields/PathToleranceRefScaledInteger.h"

namespace openjaus
{
namespace mobility
{

/// \class ReportLocalPathSegment ReportLocalPathSegment.h
/// \brief ReportLocalPathSegment Message Implementation.
/// This message is used to provide the receiver the values of the current path segment. The message data and mapping of
/// the presence vector of this message are identical to ID 0410h: SetLocalPathSegment.
class OPENJAUS_EXPORT ReportLocalPathSegment : public model::Message
{
public:
	ReportLocalPathSegment();
	ReportLocalPathSegment(model::Message *message);
	~ReportLocalPathSegment();
	
	static const uint16_t ID = 0x4410;

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
	bool isP1ZEnabled(void) const;
	void enableP1Z(void);
	void disableP1Z(void);
	bool isP2ZEnabled(void) const;
	void enableP2Z(void);
	void disableP2Z(void);
	bool isPathToleranceEnabled(void) const;
	void enablePathTolerance(void);
	void disablePathTolerance(void);


	double getP1X_m(void);
	void setP1X_m(double value);

	double getP1Y_m(void);
	void setP1Y_m(double value);

	double getP1Z_m(void);
	void setP1Z_m(double value);

	double getP2X_m(void);
	void setP2X_m(double value);

	double getP2Y_m(void);
	void setP2Y_m(double value);

	double getP2Z_m(void);
	void setP2Z_m(double value);

	double getWeightingFactor(void);
	void setWeightingFactor(double value);

	double getPathTolerance(void);
	void setPathTolerance(double value);

private:
	LocalPositionScaledInteger p1X_m;
	LocalPositionScaledInteger p1Y_m;
	JausAltitudeScaledInteger p1Z_m;
	LocalPositionScaledInteger p2X_m;
	LocalPositionScaledInteger p2Y_m;
	JausAltitudeScaledInteger p2Z_m;
	WeightingFactorRefScaledInteger weightingFactor;
	PathToleranceRefScaledInteger pathTolerance;

	uint8_t presenceVector;
	enum pvEnum {P1Z_M = 0, P2Z_M = 1, PATHTOLERANCE = 2};
};

} // namespace mobility
} // namespace openjaus

#endif // REPORTLOCALPATHSEGMENT_H


