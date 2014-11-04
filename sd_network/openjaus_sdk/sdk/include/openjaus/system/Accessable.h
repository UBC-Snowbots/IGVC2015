/**
\file Accessable.h

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
#ifndef SYSTEM_ACCESSABLE_H
#define SYSTEM_ACCESSABLE_H

#include <vector>
#include <string>
#include "openjaus/types.h"
#include <ostream>

// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace system
{

/// \class Accessable Accessable.h

class OPENJAUS_EXPORT Accessable 
{
public:
	Accessable(); 
	virtual ~Accessable();
	// Start of user code for additional constructors
	// End of user code
	/// Accessor to get the value of globalRead.
	bool isGlobalRead() const;

	/// Accessor to set value of globalRead.
	/// \param globalRead The value of the new globalRead.
	bool setGlobalRead(bool globalRead);

	/// Accessor to get the value of controllerRead.
	bool isControllerRead() const;

	/// Accessor to set value of controllerRead.
	/// \param controllerRead The value of the new controllerRead.
	bool setControllerRead(bool controllerRead);

	/// Accessor to get the value of controllerWrite.
	bool isControllerWrite() const;

	/// Accessor to set value of controllerWrite.
	/// \param controllerWrite The value of the new controllerWrite.
	bool setControllerWrite(bool controllerWrite);

	/// Accessor to get the value of ownerUri.
	std::vector< std::string > getOwnerUri() const;

	/// Accessor to set value of ownerUri.
	/// \param ownerUri The value of the new ownerUri.
	bool setOwnerUri(std::string ownerUri);

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const Accessable& object);

protected:
	// Member attributes & references
	bool globalRead;
	bool controllerRead;
	bool controllerWrite;
	std::vector< std::string > ownerUri;

// Start of user code for additional member data
// End of user code

}; // class Accessable

// Start of user code for inline functions
// End of user code



} // namespace system
} // namespace openjaus

#endif // SYSTEM_ACCESSABLE_H

