/**
\file JudpLargeMessageBuffer.h

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
#ifndef AS5669_JUDPLARGEMESSAGEBUFFER_H
#define AS5669_JUDPLARGEMESSAGEBUFFER_H

#include "openjaus/transport/Wrapper.h"
#include <map>
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

/// \class JudpLargeMessageBuffer JudpLargeMessageBuffer.h

class OPENJAUS_EXPORT JudpLargeMessageBuffer 
{
public:
	JudpLargeMessageBuffer(); 
	virtual ~JudpLargeMessageBuffer();
	// Start of user code for additional constructors
	// End of user code
	/// Accessor to get the value of buffer.
	const std::map< uint16_t, transport::Wrapper * >& getBuffer() const;


	/// Accessor to get the value of lastSeqNum.
	uint16_t getLastSeqNum() const;


	/// Accessor to get the value of first.
	transport::Wrapper* getFirst() const;


	/// \brief Adds wrapper to the internal buffer. Returns true if wrapper passed checks and false if wrapper was discarded.
	/// \param wrapper 
	 bool addWrapper(transport::Wrapper *wrapper);

	/// \brief Reassembles the buffer into one JudpWrapper. Returns NULL on failure.
	/// \param last 
	 transport::Wrapper* assembleWrapper(transport::Wrapper *last);

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const JudpLargeMessageBuffer& object);

protected:
	// Member attributes & references
	std::map< uint16_t, transport::Wrapper * > buffer;
	uint16_t lastSeqNum;
	transport::Wrapper *first;

// Start of user code for additional member data
protected:
	void reset();
// End of user code

}; // class JudpLargeMessageBuffer

// Start of user code for inline functions
// End of user code



} // namespace AS5669
} // namespace transport
} // namespace openjaus

#endif // AS5669_JUDPLARGEMESSAGEBUFFER_H

