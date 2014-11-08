
/**
\file ReportPlatformSpecifications.h

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

#ifndef REPORTPLATFORMSPECIFICATIONS_H
#define REPORTPLATFORMSPECIFICATIONS_H

#include <openjaus.h>


namespace openjaus
{
namespace ugv
{

/// \class ReportPlatformSpecifications ReportPlatformSpecifications.h
/// \brief ReportPlatformSpecifications Message Implementation.
/// Sends PlatformSpecifications data
class OPENJAUS_EXPORT ReportPlatformSpecifications : public model::Message
{
public:
	ReportPlatformSpecifications();
	ReportPlatformSpecifications(model::Message *message);
	~ReportPlatformSpecifications();
	
	static const uint16_t ID = 0x4502;

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

	void setPresenceVector(uint16_t value);
	uint16_t getPresenceVector(void) const;
	bool isFrontEnabled(void) const;
	void enableFront(void);
	void disableFront(void);

	bool isBackEnabled(void) const;
	void enableBack(void);
	void disableBack(void);

	bool isRightEnabled(void) const;
	void enableRight(void);
	void disableRight(void);

	bool isLeftEnabled(void) const;
	void enableLeft(void);
	void disableLeft(void);

	bool isBottomEnabled(void) const;
	void enableBottom(void);
	void disableBottom(void);

	bool isTopEnabled(void) const;
	void enableTop(void);
	void disableTop(void);

	bool isXcgEnabled(void) const;
	void enableXcg(void);
	void disableXcg(void);

	bool isYcgEnabled(void) const;
	void enableYcg(void);
	void disableYcg(void);

	bool isZcgEnabled(void) const;
	void enableZcg(void);
	void disableZcg(void);

	bool isWheelBaseEnabled(void) const;
	void enableWheelBase(void);
	void disableWheelBase(void);

	bool isStaticPitchOverEnabled(void) const;
	void enableStaticPitchOver(void);
	void disableStaticPitchOver(void);

	bool isStaticRollOverEnabled(void) const;
	void enableStaticRollOver(void);
	void disableStaticRollOver(void);



	std::string getMobilityPlatformName(void);
	void setMobilityPlatformName(std::string value);

	uint16_t getFront_m(void);
	void setFront_m(uint16_t value);

	uint16_t getBack_m(void);
	void setBack_m(uint16_t value);

	uint16_t getRight_m(void);
	void setRight_m(uint16_t value);

	uint16_t getLeft_m(void);
	void setLeft_m(uint16_t value);

	uint16_t getBottom_m(void);
	void setBottom_m(uint16_t value);

	uint16_t getTop_m(void);
	void setTop_m(uint16_t value);

	uint16_t getXcg_m(void);
	void setXcg_m(uint16_t value);

	uint16_t getYcg_m(void);
	void setYcg_m(uint16_t value);

	uint16_t getZcg_m(void);
	void setZcg_m(uint16_t value);

	uint16_t getWheelBase_m(void);
	void setWheelBase_m(uint16_t value);

	uint16_t getStaticPitchOver_rad(void);
	void setStaticPitchOver_rad(uint16_t value);

	uint16_t getStaticRollOver_rad(void);
	void setStaticRollOver_rad(uint16_t value);

private:
	model::fields::FixedLengthString mobilityPlatformName;
	model::fields::UnsignedShort front_m;
	model::fields::UnsignedShort back_m;
	model::fields::UnsignedShort right_m;
	model::fields::UnsignedShort left_m;
	model::fields::UnsignedShort bottom_m;
	model::fields::UnsignedShort top_m;
	model::fields::UnsignedShort xcg_m;
	model::fields::UnsignedShort ycg_m;
	model::fields::UnsignedShort zcg_m;
	model::fields::UnsignedShort wheelBase_m;
	model::fields::UnsignedShort staticPitchOver_rad;
	model::fields::UnsignedShort staticRollOver_rad;

	uint16_t presenceVector;
	enum pvEnum {FRONT_M = 0, BACK_M = 1, RIGHT_M = 2, LEFT_M = 3, BOTTOM_M = 4, TOP_M = 5, XCG_M = 6, YCG_M = 7, ZCG_M = 8, WHEELBASE_M = 9, STATICPITCHOVER_RAD = 10, STATICROLLOVER_RAD = 11};
};

} // namespace ugv
} // namespace openjaus

#endif // REPORTPLATFORMSPECIFICATIONS_H


