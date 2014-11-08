/**
\file ArrayDimension.h

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

#include "openjaus/model/fields/ArrayDimension.h"
#include <sstream>
// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace model
{
namespace fields
{

// Start of user code for default constructor:
ArrayDimension::ArrayDimension()
{
}
// End of user code

// Start of user code for default destructor:
ArrayDimension::~ArrayDimension()
{
}
// End of user code

std::string ArrayDimension::getName() const
{
	// Start of user code for accessor getName:
	
	return name;
	// End of user code
}

bool ArrayDimension::setName(std::string name)
{
	// Start of user code for accessor setName:
	this->name = name;
	return true;
	// End of user code
}


std::string ArrayDimension::getInterpretation() const
{
	// Start of user code for accessor getInterpretation:
	
	return interpretation;
	// End of user code
}

bool ArrayDimension::setInterpretation(std::string interpretation)
{
	// Start of user code for accessor setInterpretation:
	this->interpretation = interpretation;
	return true;
	// End of user code
}


long ArrayDimension::getSize() const
{
	// Start of user code for accessor getSize:
	
	return size;
	// End of user code
}

bool ArrayDimension::setSize(long size)
{
	// Start of user code for accessor setSize:
	this->size = size;
	return true;
	// End of user code
}



// Class Methods


std::string ArrayDimension::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const ArrayDimension& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
// End of user code

} // namespace fields
} // namespace model
} // namespace openjaus

