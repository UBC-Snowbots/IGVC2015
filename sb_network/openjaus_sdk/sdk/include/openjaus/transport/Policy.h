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
#ifndef TRANSPORT_POLICY_H
#define TRANSPORT_POLICY_H

#include "openjaus/system/Time.h"
#include "openjaus/transport/TCPPreference.h"
#include <string>
#include "openjaus/types.h"
#include <ostream>

// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace transport
{

/// \class Policy Policy.h

class OPENJAUS_EXPORT Policy 
{
public:
	Policy(); 
	virtual ~Policy();
	// Start of user code for additional constructors
	// End of user code
	/// Accessor to get the value of requestCount.
	int getRequestCount() const;

	/// Accessor to set value of requestCount.
	/// \param requestCount The value of the new requestCount.
	bool setRequestCount(int requestCount);

	/// Accessor to get the value of zlibCompression.
	bool isZlibCompression() const;

	/// Accessor to set value of zlibCompression.
	/// \param zlibCompression The value of the new zlibCompression.
	bool setZlibCompression(bool zlibCompression);

	/// Accessor to get the value of confirmed.
	bool isConfirmed() const;

	/// Accessor to set value of confirmed.
	/// \param confirmed The value of the new confirmed.
	bool setConfirmed(bool confirmed);

	/// Accessor to get the value of TCPSupported.
	bool isTCPSupported() const;

	/// Accessor to set value of TCPSupported.
	/// \param TCPSupported The value of the new TCPSupported.
	bool setTCPSupported(bool TCPSupported);

	/// Accessor to get the value of OpenJAUSSupported.
	bool isOpenJAUSSupported() const;

	/// Accessor to set value of OpenJAUSSupported.
	/// \param OpenJAUSSupported The value of the new OpenJAUSSupported.
	bool setOpenJAUSSupported(bool OpenJAUSSupported);

	/// Accessor to get the value of preferredTCPUse.
	TCPPreference getPreferredTCPUse() const;

	/// Accessor to set value of preferredTCPUse.
	/// \param preferredTCPUse The value of the new preferredTCPUse.
	bool setPreferredTCPUse(TCPPreference preferredTCPUse);

	/// Accessor to get the value of timeout.
	const system::Time& getTimeout() const;

	/// Accessor to set value of timeout.
	/// \param timeout The value of the new timeout.
	bool setTimeout(const system::Time& timeout);

	/// Operation setPolicy.
	/// \param zlib 
	 bool setPolicy(bool zlib);

	/// Operation incrementRequestCount.
	 void incrementRequestCount();

	/// Operation resetTimeout.
	 void resetTimeout();

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const Policy& object);

protected:
	// Member attributes & references
	int requestCount;
	bool zlibCompression;
	bool confirmed;
	bool TCPSupported;
	bool OpenJAUSSupported;
	TCPPreference preferredTCPUse;
	system::Time timeout;

// Start of user code for additional member data
// End of user code

}; // class Policy

// Start of user code for inline functions
// End of user code



} // namespace transport
} // namespace openjaus

#endif // TRANSPORT_POLICY_H

