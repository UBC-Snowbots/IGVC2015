/**
\file Transition.h

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
#ifndef MODEL_TRANSITION_H
#define MODEL_TRANSITION_H

#include "openjaus/model/Trigger.h"
#include "openjaus/model/Message.h"
#include "openjaus/model/TransitionType.h"
#include "openjaus/model/ActionSet.h"
#include "openjaus/model/State.h"
#include "openjaus/model/StateMachine.h"
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
class Trigger;
class Message;
class ActionSet;
class State;
class StateMachine;

/// \class Transition Transition.h
/// \brief This is a brief description.
/// Detailed description.
/// \author Name (name@email.com)
class OPENJAUS_EXPORT Transition 
{
public:
	Transition(); 
	virtual ~Transition();
	// Start of user code for additional constructors
	// End of user code
	/// Accessor to get the value of name.
	std::string getName() const;

	/// Accessor to set value of name.
	/// \param name The value of the new name.
	bool setName(std::string name);

	/// Accessor to get the value of type.
	TransitionType getType() const;

	/// Accessor to set value of type.
	/// \param type The value of the new type.
	bool setType(TransitionType type);

	/// Accessor to get the value of actionSets.
	const std::vector< ActionSet* >& getActionSets() const;

	/// Accessor to set value of actionSets.
	/// \param actionSets The value of the new actionSets.
	bool setActionSets(const ActionSet& actionSets);

	/// Accessor to get the value of triggers.
	std::vector< Trigger* >* getTriggers() const;

	/// Accessor to set value of triggers.
	/// \param triggers The value of the new triggers.
	bool setTriggers(Trigger* triggers);

	/// Accessor to get the value of endState.
	State* getEndState() const;

	/// Accessor to set value of endState.
	/// \param endState The value of the new endState.
	bool setEndState(State* endState);

	/// Accessor to get the value of startState.
	State* getStartState() const;

	/// Accessor to set value of startState.
	/// \param startState The value of the new startState.
	bool setStartState(State* startState);

	/// Accessor to get the value of parentState.
	State* getParentState() const;

	/// Accessor to set value of parentState.
	/// \param parentState The value of the new parentState.
	bool setParentState(State* parentState);

	/// Accessor to get the value of parentStateMachine.
	StateMachine* getParentStateMachine() const;

	/// Accessor to set value of parentStateMachine.
	/// \param parentStateMachine The value of the new parentStateMachine.
	bool setParentStateMachine(StateMachine* parentStateMachine);


	/// \param trigger 
	virtual bool processTrigger(Trigger *trigger);


	/// \param trigger 
	virtual Message* getResponse(Trigger *trigger);

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const Transition& object);

protected:
	// Member attributes & references
	std::string name;
	TransitionType type;
	std::vector< ActionSet* > actionSets;
	std::vector< Trigger* > *triggers;
	State *endState;
	State *startState;
	State *parentState;
	StateMachine *parentStateMachine;

// Start of user code for additional member data
// End of user code

}; // class Transition

// Start of user code for inline functions
// End of user code



} // namespace model
} // namespace openjaus

#endif // MODEL_TRANSITION_H

