/**
\file Date.h

\par Copyright
Copyright (c) 2011, OpenJAUS, LLC
All rights reserved.

This file is part of OpenJAUS. OpenJAUS is distributed under the OpenJAUS
SDK Commercial End User License Agreement. See the LICENSE.txt file for more 
details.
 
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
- [2011-06-16] - First Commercial Release 

*/


#ifndef DATE_BITFIELD_H
#define DATE_BITFIELD_H

#include <openjaus.h>

namespace openjaus
{
namespace core
{

class OPENJAUS_EXPORT DateBitField : public model::fields::BitField
{
public:
	DateBitField();
	~DateBitField();



    bool setDay(long value);
    long getDay(void) const;
    bool setMonth(long value);
    long getMonth(void) const;
    bool setYear(long value);
    long getYear(void) const;

	virtual int to(system::Buffer *dst);	
	virtual int from(system::Buffer *src);
	virtual int length();
	void copy(DateBitField& source);
	std::string toXml(unsigned char level=0) const;
	
protected:
    long day;
    long month;
    long year;

    static const long DAY_START_BIT = 0;
    static const long DAY_BIT_MASK = 0x1F;
    
    static const long MONTH_START_BIT = 5;
    static const long MONTH_BIT_MASK = 0xF;
    
    static const long YEAR_START_BIT = 9;
    static const long YEAR_BIT_MASK = 0x7F;
    
    static const long DAY_MIN_VALUE = 1;
    static const long DAY_MAX_VALUE = 31;
    
    static const long MONTH_MIN_VALUE = 1;
    static const long MONTH_MAX_VALUE = 12;
    
    static const long YEAR_MIN_VALUE = 0;
    static const long YEAR_MAX_VALUE = 127;
    
};

} // namespace core
} // namespace openjaus

#endif // DATE_BITFIELD_H

