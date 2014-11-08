/**
\file Setting.h

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

#include "openjaus/system/Setting.h"
#include <sstream>
// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace system
{

// Start of user code for default constructor:
Setting::Setting()
{
}
// End of user code

// Start of user code for default destructor:
Setting::~Setting()
{
}
// End of user code

std::string Setting::getKey() const
{
	// Start of user code for accessor getKey:
	
	return key;
	// End of user code
}

bool Setting::setKey(std::string key)
{
	// Start of user code for accessor setKey:
	this->key = key;
	return true;
	// End of user code
}


std::string Setting::getValueStr() const
{
	// Start of user code for accessor getValueStr:
	
	return valueStr;
	// End of user code
}

bool Setting::setValueStr(std::string valueStr)
{
	// Start of user code for accessor setValueStr:
	this->valueStr = valueStr;
	return true;
	// End of user code
}


std::string Setting::getComment() const
{
	// Start of user code for accessor getComment:
	
	return comment;
	// End of user code
}

bool Setting::setComment(std::string comment)
{
	// Start of user code for accessor setComment:
	this->comment = comment;
	return true;
	// End of user code
}


bool Setting::isSaveDefault() const
{
	// Start of user code for accessor getSaveDefault:
	
	return saveDefault;
	// End of user code
}

bool Setting::setSaveDefault(bool saveDefault)
{
	// Start of user code for accessor setSaveDefault:
	this->saveDefault = saveDefault;
	return true;
	// End of user code
}



// Class Methods


std::string Setting::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const Setting& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
// End of user code

} // namespace system
} // namespace openjaus

