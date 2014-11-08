/**
\file JtcpPacket.h

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
#ifndef AS5669_JTCPPACKET_H
#define AS5669_JTCPPACKET_H

#include "openjaus/transport/WrapperType.h"
#include "openjaus/transport/Wrapper.h"
#include "openjaus/system/Packet.h"
#include <vector>
#include <string>
#include "openjaus/types.h"
#include <ostream>

// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace transport
{
class Wrapper;
namespace AS5669
{

/// \class JtcpPacket JtcpPacket.h
/// \brief This is a brief description.
/// Detailed description.
/// \author Name (name@email.com)
class OPENJAUS_EXPORT JtcpPacket : public system::Packet
{
public:
	JtcpPacket(); 
	virtual ~JtcpPacket();
	// Start of user code for additional constructors
	// End of user code
	/// Accessor to get the value of version.
	unsigned char getVersion() const;


	/// Accessor to get the value of wrappers.
	const std::vector< transport::Wrapper* >& getWrappers() const;

	/// Accessor to set value of wrappers.
	/// \param wrappers The value of the new wrappers.
	bool setWrappers(const transport::Wrapper& wrappers);

	/// \brief Serializes object to internal transport buffer.
	 transport::WrapperType nextWrapperType();


	 transport::Wrapper* popWrapper();

	/// Operation pushWrapper.
	/// \param newWrapper 
	 bool pushWrapper(transport::Wrapper *newWrapper);

	/// Operation reset.
	 unsigned char * reset();

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const JtcpPacket& object);

protected:
	// Member attributes & references
	unsigned char version;
	std::vector< transport::Wrapper* > wrappers;

// Start of user code for additional member data
	static const int DEFAULT_SIZE = 4096;
	static const unsigned char STANDARD_VERSION = 2;
// End of user code

}; // class JtcpPacket

// Start of user code for inline functions
// End of user code



} // namespace AS5669
} // namespace transport
} // namespace openjaus

#endif // AS5669_JTCPPACKET_H

