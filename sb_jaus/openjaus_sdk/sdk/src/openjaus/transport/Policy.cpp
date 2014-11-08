/**
\file Policy.h

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

#include "openjaus/transport/Policy.h"
#include <sstream>
// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace transport
{

// Start of user code for default constructor:
Policy::Policy() :
	requestCount(0),
	zlibCompression(false),
	confirmed(false),
	TCPSupported(false),
	OpenJAUSSupported(false),
	preferredTCPUse(APPLICATION_REQUIRED_ONLY),
	timeout()
{
}
// End of user code

// Start of user code for default destructor:
Policy::~Policy()
{
}
// End of user code

int Policy::getRequestCount() const
{
	// Start of user code for accessor getRequestCount:
	
	return requestCount;
	// End of user code
}

bool Policy::setRequestCount(int requestCount)
{
	// Start of user code for accessor setRequestCount:
	this->requestCount = requestCount;
	return true;
	// End of user code
}


bool Policy::isZlibCompression() const
{
	// Start of user code for accessor getZlibCompression:
	
	return zlibCompression;
	// End of user code
}

bool Policy::setZlibCompression(bool zlibCompression)
{
	// Start of user code for accessor setZlibCompression:
	this->zlibCompression = zlibCompression;
	return true;
	// End of user code
}


bool Policy::isConfirmed() const
{
	// Start of user code for accessor getConfirmed:
	
	return confirmed;
	// End of user code
}

bool Policy::setConfirmed(bool confirmed)
{
	// Start of user code for accessor setConfirmed:
	this->confirmed = confirmed;
	return true;
	// End of user code
}


bool Policy::isTCPSupported() const
{
	// Start of user code for accessor getTCPSupported:
	
	return TCPSupported;
	// End of user code
}

bool Policy::setTCPSupported(bool TCPSupported)
{
	// Start of user code for accessor setTCPSupported:
	this->TCPSupported = TCPSupported;
	return true;
	// End of user code
}


bool Policy::isOpenJAUSSupported() const
{
	// Start of user code for accessor getOpenJAUSSupported:
	
	return OpenJAUSSupported;
	// End of user code
}

bool Policy::setOpenJAUSSupported(bool OpenJAUSSupported)
{
	// Start of user code for accessor setOpenJAUSSupported:
	this->OpenJAUSSupported = OpenJAUSSupported;
	return true;
	// End of user code
}


TCPPreference Policy::getPreferredTCPUse() const
{
	// Start of user code for accessor getPreferredTCPUse:
	
	return preferredTCPUse;
	// End of user code
}

bool Policy::setPreferredTCPUse(TCPPreference preferredTCPUse)
{
	// Start of user code for accessor setPreferredTCPUse:
	this->preferredTCPUse = preferredTCPUse;
	return true;
	// End of user code
}


const system::Time& Policy::getTimeout() const
{
	// Start of user code for accessor getTimeout:
	
	return timeout;
	// End of user code
}

bool Policy::setTimeout(const system::Time& timeout)
{
	// Start of user code for accessor setTimeout:
	this->timeout = timeout;
	return true;
	// End of user code
}



// Class Methods
bool Policy::setPolicy(bool zlib)
{
	// Start of user code for method setPolicy:
	bool result = 0;

	return result;
	// End of user code
}


void Policy::incrementRequestCount()
{
	// Start of user code for method incrementRequestCount:
	requestCount++;
	// End of user code
}


void Policy::resetTimeout()
{
	// Start of user code for method resetTimeout:
	timeout = system::Time::getTime();
	timeout.setSeconds(timeout.getSeconds() + 1);
	// End of user code
}




std::string Policy::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const Policy& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
// End of user code

} // namespace transport
} // namespace openjaus

