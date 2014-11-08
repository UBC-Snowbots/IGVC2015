/**
\file Units.h

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
#ifndef UNITS_H
#define UNITS_H

#include <istream>
#include <ostream>
#include <string>

namespace openjaus
{
namespace model
{
namespace fields
{

enum Units
{
	ONE = 0,
	METER = 1,
	KILOGRAM = 2,
	SECOND = 3,
	AMPERE = 4,
	KELVIN = 5,
	MOLE = 6,
	CANDELA = 7,
	SQUARE_METER = 8,
	CUBIC_METER = 9,
	METER_PER_SECOND = 10,
	METER_PER_SECOND_SQUARED = 11,
	RECIPROCAL_METER = 12,
	KILOGRAM_PER_CUBIC_METER = 13,
	CUBIC_METER_PER_KILOGRAM = 14,
	HERTZ = 15,
	MINUTE = 16,
	DEGREES = 17,
	RADIANS = 18,
	RADIANS_PER_SECOND = 19,
	RADIANS_PER_SECOND_SQUARED = 20,
	PERCENT = 21,
	NEWTON = 22,
	NEWTON_METER = 23
};

inline std::ostream& operator<<(std::ostream& output, const Units& enumValue)
{
	switch(enumValue)
	{
		case ONE:
			output << "ONE";
			break;
			
		case METER:
			output << "METER";
			break;
			
		case KILOGRAM:
			output << "KILOGRAM";
			break;
			
		case SECOND:
			output << "SECOND";
			break;
			
		case AMPERE:
			output << "AMPERE";
			break;
			
		case KELVIN:
			output << "KELVIN";
			break;
			
		case MOLE:
			output << "MOLE";
			break;
			
		case CANDELA:
			output << "CANDELA";
			break;
			
		case SQUARE_METER:
			output << "SQUARE_METER";
			break;
			
		case CUBIC_METER:
			output << "CUBIC_METER";
			break;
			
		case METER_PER_SECOND:
			output << "METER_PER_SECOND";
			break;
			
		case METER_PER_SECOND_SQUARED:
			output << "METER_PER_SECOND_SQUARED";
			break;
			
		case RECIPROCAL_METER:
			output << "RECIPROCAL_METER";
			break;
			
		case KILOGRAM_PER_CUBIC_METER:
			output << "KILOGRAM_PER_CUBIC_METER";
			break;
			
		case CUBIC_METER_PER_KILOGRAM:
			output << "CUBIC_METER_PER_KILOGRAM";
			break;
			
		case HERTZ:
			output << "HERTZ";
			break;
			
		case MINUTE:
			output << "MINUTE";
			break;
			
		case DEGREES:
			output << "DEGREES";
			break;
			
		case RADIANS:
			output << "RADIANS";
			break;
			
		case RADIANS_PER_SECOND:
			output << "RADIANS_PER_SECOND";
			break;
			
		case RADIANS_PER_SECOND_SQUARED:
			output << "RADIANS_PER_SECOND_SQUARED";
			break;
			
		case PERCENT:
			output << "PERCENT";
			break;
			
		case NEWTON:
			output << "NEWTON";
			break;
			
		case NEWTON_METER:
			output << "NEWTON_METER";
			break;
			
	}
	return output;
}

inline std::istream& operator>>(std::istream& input, Units& enumValue)
{
	std::string enumString;
	input >> enumString;
	if(enumString.compare("ONE") == 0)
	{
		enumValue = ONE;
	}
	if(enumString.compare("METER") == 0)
	{
		enumValue = METER;
	}
	if(enumString.compare("KILOGRAM") == 0)
	{
		enumValue = KILOGRAM;
	}
	if(enumString.compare("SECOND") == 0)
	{
		enumValue = SECOND;
	}
	if(enumString.compare("AMPERE") == 0)
	{
		enumValue = AMPERE;
	}
	if(enumString.compare("KELVIN") == 0)
	{
		enumValue = KELVIN;
	}
	if(enumString.compare("MOLE") == 0)
	{
		enumValue = MOLE;
	}
	if(enumString.compare("CANDELA") == 0)
	{
		enumValue = CANDELA;
	}
	if(enumString.compare("SQUARE_METER") == 0)
	{
		enumValue = SQUARE_METER;
	}
	if(enumString.compare("CUBIC_METER") == 0)
	{
		enumValue = CUBIC_METER;
	}
	if(enumString.compare("METER_PER_SECOND") == 0)
	{
		enumValue = METER_PER_SECOND;
	}
	if(enumString.compare("METER_PER_SECOND_SQUARED") == 0)
	{
		enumValue = METER_PER_SECOND_SQUARED;
	}
	if(enumString.compare("RECIPROCAL_METER") == 0)
	{
		enumValue = RECIPROCAL_METER;
	}
	if(enumString.compare("KILOGRAM_PER_CUBIC_METER") == 0)
	{
		enumValue = KILOGRAM_PER_CUBIC_METER;
	}
	if(enumString.compare("CUBIC_METER_PER_KILOGRAM") == 0)
	{
		enumValue = CUBIC_METER_PER_KILOGRAM;
	}
	if(enumString.compare("HERTZ") == 0)
	{
		enumValue = HERTZ;
	}
	if(enumString.compare("MINUTE") == 0)
	{
		enumValue = MINUTE;
	}
	if(enumString.compare("DEGREES") == 0)
	{
		enumValue = DEGREES;
	}
	if(enumString.compare("RADIANS") == 0)
	{
		enumValue = RADIANS;
	}
	if(enumString.compare("RADIANS_PER_SECOND") == 0)
	{
		enumValue = RADIANS_PER_SECOND;
	}
	if(enumString.compare("RADIANS_PER_SECOND_SQUARED") == 0)
	{
		enumValue = RADIANS_PER_SECOND_SQUARED;
	}
	if(enumString.compare("PERCENT") == 0)
	{
		enumValue = PERCENT;
	}
	if(enumString.compare("NEWTON") == 0)
	{
		enumValue = NEWTON;
	}
	if(enumString.compare("NEWTON_METER") == 0)
	{
		enumValue = NEWTON_METER;
	}
    return input;
}

} // namespace fields
} // namespace model
} // namespace openjaus

#endif // UNITS_H

