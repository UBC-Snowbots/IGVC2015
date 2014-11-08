/**
\file ControlResponseType.h

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
#ifndef CONTROLRESPONSETYPE_H
#define CONTROLRESPONSETYPE_H

#include <istream>
#include <ostream>
#include <string>

namespace openjaus
{
namespace model
{

enum ControlResponseType
{
	NOT_AVAILABLE = 0,
	INSUFFICIENT_AUTHORITY = 1,
	CONTROL_RELEASED = 2,
	CONTROL_ACCEPTED = 3
};

inline std::ostream& operator<<(std::ostream& output, const ControlResponseType& enumValue)
{
	switch(enumValue)
	{
		case NOT_AVAILABLE:
			output << "NOT_AVAILABLE";
			break;
			
		case INSUFFICIENT_AUTHORITY:
			output << "INSUFFICIENT_AUTHORITY";
			break;
			
		case CONTROL_RELEASED:
			output << "CONTROL_RELEASED";
			break;
			
		case CONTROL_ACCEPTED:
			output << "CONTROL_ACCEPTED";
			break;
			
	}
	return output;
}

inline std::istream& operator>>(std::istream& input, ControlResponseType& enumValue)
{
	std::string enumString;
	input >> enumString;
	if(enumString.compare("NOT_AVAILABLE") == 0)
	{
		enumValue = NOT_AVAILABLE;
	}
	if(enumString.compare("INSUFFICIENT_AUTHORITY") == 0)
	{
		enumValue = INSUFFICIENT_AUTHORITY;
	}
	if(enumString.compare("CONTROL_RELEASED") == 0)
	{
		enumValue = CONTROL_RELEASED;
	}
	if(enumString.compare("CONTROL_ACCEPTED") == 0)
	{
		enumValue = CONTROL_ACCEPTED;
	}
    return input;
}

} // namespace model
} // namespace openjaus

#endif // CONTROLRESPONSETYPE_H

