/**
\file TimeStampRef.h

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


#ifndef TIMESTAMPREF_BITFIELD_H
#define TIMESTAMPREF_BITFIELD_H

#include <openjaus.h>

namespace openjaus
{
namespace environment
{

class OPENJAUS_EXPORT TimeStampRefBitField : public model::fields::BitField
{
public:
	TimeStampRefBitField();
	~TimeStampRefBitField();



    bool setMilliseconds(long value);
    long getMilliseconds(void) const;
    bool setSeconds(long value);
    long getSeconds(void) const;
    bool setMinutes(long value);
    long getMinutes(void) const;
    bool setHour(long value);
    long getHour(void) const;
    bool setDay(long value);
    long getDay(void) const;

	virtual int to(system::Buffer *dst);	
	virtual int from(system::Buffer *src);
	virtual int length();
	void copy(TimeStampRefBitField& source);
	std::string toXml(unsigned char level=0) const;
	
protected:
    long milliseconds;
    long seconds;
    long minutes;
    long hour;
    long day;

    static const long MILLISECONDS_START_BIT = 0;
    static const long MILLISECONDS_BIT_MASK = 0x3FF;
    
    static const long SECONDS_START_BIT = 10;
    static const long SECONDS_BIT_MASK = 0x3F;
    
    static const long MINUTES_START_BIT = 16;
    static const long MINUTES_BIT_MASK = 0x3F;
    
    static const long HOUR_START_BIT = 22;
    static const long HOUR_BIT_MASK = 0x1F;
    
    static const long DAY_START_BIT = 27;
    static const long DAY_BIT_MASK = 0x1F;
    
    static const long MILLISECONDS_MIN_VALUE = 0;
    static const long MILLISECONDS_MAX_VALUE = 999;
    
    static const long SECONDS_MIN_VALUE = 0;
    static const long SECONDS_MAX_VALUE = 59;
    
    static const long MINUTES_MIN_VALUE = 0;
    static const long MINUTES_MAX_VALUE = 59;
    
    static const long HOUR_MIN_VALUE = 0;
    static const long HOUR_MAX_VALUE = 23;
    
    static const long DAY_MIN_VALUE = 1;
    static const long DAY_MAX_VALUE = 31;
    
};

} // namespace environment
} // namespace openjaus

#endif // TIMESTAMPREF_BITFIELD_H

