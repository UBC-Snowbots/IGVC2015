/**
\file ActionSet.h

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
#ifndef MODEL_ACTIONSET_H
#define MODEL_ACTIONSET_H

#include "openjaus/model/Action.h"
#include "openjaus/model/Guard.h"
#include "openjaus/model/Trigger.h"
#include <vector>
#include <string>
#include "openjaus/types.h"
#include <ostream>

// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace model
{
class Action;
class Guard;
class Trigger;

/// \class ActionSet ActionSet.h
/// \brief This is a brief description.
/// Detailed description.
/// \author Name (name@email.com)
class OPENJAUS_EXPORT ActionSet 
{
public:
	ActionSet(); 
	virtual ~ActionSet();
	// Start of user code for additional constructors
	// End of user code
	/// Accessor to get the value of actions.
	std::vector< Action* >* getActions() const;

	/// Accessor to set value of actions.
	/// \param actions The value of the new actions.
	bool setActions(Action* actions);

	/// Accessor to get the value of guard.
	const Guard& getGuard() const;

	/// Accessor to set value of guard.
	/// \param guard The value of the new guard.
	bool setGuard(const Guard& guard);

	/// Accessor to get the value of trigger.
	Trigger* getTrigger() const;

	/// Accessor to set value of trigger.
	/// \param trigger The value of the new trigger.
	bool setTrigger(Trigger* trigger);

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const ActionSet& object);

protected:
	// Member attributes & references
	std::vector< Action* > *actions;
	Guard guard;
	Trigger *trigger;

// Start of user code for additional member data
// End of user code

}; // class ActionSet

// Start of user code for inline functions
// End of user code



} // namespace model
} // namespace openjaus

#endif // MODEL_ACTIONSET_H

