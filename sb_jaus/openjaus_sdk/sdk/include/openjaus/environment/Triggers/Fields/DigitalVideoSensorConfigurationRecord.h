
/**
\file DigitalVideoSensorConfigurationRec.h

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

#ifndef DIGITALVIDEOSENSORCONFIGURATIONRECORD_H
#define DIGITALVIDEOSENSORCONFIGURATIONRECORD_H

#include <openjaus.h>
#include "openjaus/environment/Triggers/Fields/FrameSizeRefEnumeration.h"
#include "openjaus/environment/Triggers/Fields/DigitalFormatEnumeration.h"

namespace openjaus
{
namespace environment
{

class OPENJAUS_EXPORT DigitalVideoSensorConfigurationRecord : public openjaus::model::fields::Record
{
public:
	DigitalVideoSensorConfigurationRecord();
	DigitalVideoSensorConfigurationRecord(const DigitalVideoSensorConfigurationRecord &source);
	~DigitalVideoSensorConfigurationRecord();

	void copy(DigitalVideoSensorConfigurationRecord& source);
	virtual int to(system::Buffer *dst);
	virtual int from(system::Buffer *src);
	virtual int length(void);
	std::string toXml(unsigned char level=0) const;
	
	void setPresenceVector(uint8_t value);
	uint8_t getPresenceVector(void) const;
	bool isMinimumBitRateEnabled(void) const;
	void enableMinimumBitRate(void);
	void disableMinimumBitRate(void);

	bool isMaximumBitRateEnabled(void) const;
	void enableMaximumBitRate(void);
	void disableMaximumBitRate(void);

	bool isFrameRateEnabled(void) const;
	void enableFrameRate(void);
	void disableFrameRate(void);

	bool isFrameSizeEnabled(void) const;
	void enableFrameSize(void);
	void disableFrameSize(void);

	bool isDigitalFormatEnabled(void) const;
	void enableDigitalFormat(void);
	void disableDigitalFormat(void);


	uint16_t getSensorID(void);
	void setSensorID(uint16_t value);

	uint16_t getMinimumBitRate(void);
	void setMinimumBitRate(uint16_t value);

	uint16_t getMaximumBitRate(void);
	void setMaximumBitRate(uint16_t value);

	uint8_t getFrameRate(void);
	void setFrameRate(uint8_t value);

	FrameSizeRefEnumeration::FrameSizeRefEnum getFrameSize(void);
	void setFrameSize(FrameSizeRefEnumeration::FrameSizeRefEnum value);

	DigitalFormatEnumeration::DigitalFormatEnum getDigitalFormat(void);
	void setDigitalFormat(DigitalFormatEnumeration::DigitalFormatEnum value);

protected:
	model::fields::UnsignedShort sensorID;
	model::fields::UnsignedShort minimumBitRate;
	model::fields::UnsignedShort maximumBitRate;
	model::fields::UnsignedByte frameRate;
	FrameSizeRefEnumeration frameSize;
	DigitalFormatEnumeration digitalFormat;

	uint8_t presenceVector;
	enum pvEnum {MINIMUMBITRATE = 0, MAXIMUMBITRATE = 1, FRAMERATE = 2, FRAMESIZE = 3, DIGITALFORMAT = 4};
};

} // namespace environment
} // namespace openjaus

#endif // DIGITALVIDEOSENSORCONFIGURATIONRECORD_H

