/**
\file JausWrapper.h

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
#ifndef AS5669_JAUSWRAPPER_H
#define AS5669_JAUSWRAPPER_H

#include "openjaus/system/Buffer.h"
#include "openjaus/transport/Wrapper.h"
#include <string>
#include "openjaus/types.h"
#include <ostream>

// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace transport
{
namespace AS5669
{

/// \class JausWrapper JausWrapper.h
/// \brief This is a brief description.
/// Detailed description.
/// \author Name (name@email.com)
class OPENJAUS_EXPORT JausWrapper : public transport::Wrapper
{
public:
	JausWrapper(); 
	virtual ~JausWrapper();
	// Start of user code for additional constructors
	JausWrapper(const transport::Wrapper& wrapper);
	// End of user code
	/// Accessor to get the value of headerNumber.
	unsigned char getHeaderNumber() const;

	/// Accessor to set value of headerNumber.
	/// \param headerNumber The value of the new headerNumber.
	bool setHeaderNumber(unsigned char headerNumber);

	/// Accessor to get the value of headerLength.
	unsigned char getHeaderLength() const;

	/// Accessor to set value of headerLength.
	/// \param headerLength The value of the new headerLength.
	bool setHeaderLength(unsigned char headerLength);

	/// Accessor to get the value of headerFlags.
	unsigned char getHeaderFlags() const;

	/// Accessor to set value of headerFlags.
	/// \param headerFlags The value of the new headerFlags.
	bool setHeaderFlags(unsigned char headerFlags);

	/// Operation to.
	/// \param dst 
	 int to(system::Buffer *dst);

	/// Operation from.
	/// \param src 
	 int from(system::Buffer *src);

	/// \brief Serializes object to internal transport buffer.
	virtual int length();

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const JausWrapper& object);

protected:
	// Member attributes & references
	unsigned char headerNumber;
	unsigned char headerLength;
	unsigned char headerFlags;

// Start of user code for additional member data
// End of user code

}; // class JausWrapper

// Start of user code for inline functions
// End of user code



} // namespace AS5669
} // namespace transport
} // namespace openjaus

#endif // AS5669_JAUSWRAPPER_H

