
/**
\file ReportManipulatorSpecifications.h

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

#ifndef REPORTMANIPULATORSPECIFICATIONS_H
#define REPORTMANIPULATORSPECIFICATIONS_H

#include <openjaus.h>

#include "openjaus/manipulator/Triggers/Fields/ManipulatorCoordinateSystemRecord.h"
#include "openjaus/manipulator/Triggers/Fields/FirstJointParametersVariant.h"
#include "openjaus/manipulator/Triggers/Fields/JointSpecificationList.h"

namespace openjaus
{
namespace manipulator
{

/// \class ReportManipulatorSpecifications ReportManipulatorSpecifications.h
/// \brief ReportManipulatorSpecifications Message Implementation.
/// This message provides the specification of the manipulator including the number of joints, the link length and twist
/// angle of each link, the joint offset (for revolute joints) or joint angle (for prismatic joints), the minimum and
/// maximum value for each joint, and the minimum and maximum speed for each joint. The record
/// ManipulatorCoordinateSystemRec establishes the position and orientation relationship of the manipulator base
/// coordinate system to the vehicle coordinate system. It specifies the location (x,y,z) and orientation (described by
/// the quaternion) of the manipulator coordinate system relative to the base coordinate system.  The variant
/// FirstJointParameters specifies the offset distance from the origin of the manipulator coordinate system to the first
/// link axis (L1 in Figure 1) or the joint angle between the x-axis of the manipulator coordinate system and the line
/// along the first link (φ1 in Figure 1), depending on whether the first joint in JointSpecificationList is revolute or
/// prismatic.  The record RevoluteJointSpecificationRec specifies the link length and twist angle for a link and the
/// joint offset for the revolute joint that comes after the link.  The record PrismaticJointSpecificationRec specifies
/// the link length and twist angle for a link and the joint angle for the prismatic joint that comes after the link.
/// The order in which the joints are encoded into the list called JointSpecificationList must correspond with the order
/// of the joints from the manipulator base to the end effector.
class OPENJAUS_EXPORT ReportManipulatorSpecifications : public model::Message
{
public:
	ReportManipulatorSpecifications();
	ReportManipulatorSpecifications(model::Message *message);
	~ReportManipulatorSpecifications();
	
	static const uint16_t ID = 0x4600;

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
	bool isManipulatorCoordinateSystemRecEnabled(void) const;
	void enableManipulatorCoordinateSystemRec(void);
	void disableManipulatorCoordinateSystemRec(void);



	ManipulatorCoordinateSystemRecord& getManipulatorCoordinateSystemRec(void);

	FirstJointParametersVariant& getFirstJointParameters(void);

	JointSpecificationList& getJointSpecificationList(void);

private:
	ManipulatorCoordinateSystemRecord manipulatorCoordinateSystemRec;
	FirstJointParametersVariant firstJointParameters;
	JointSpecificationList jointSpecificationList;

	uint8_t presenceVector;
	enum pvEnum {MANIPULATORCOORDINATESYSTEMREC = 0};
};

} // namespace manipulator
} // namespace openjaus

#endif // REPORTMANIPULATORSPECIFICATIONS_H


