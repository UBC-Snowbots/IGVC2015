/**
\file StateMachine.h

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
#ifndef MODEL_STATEMACHINE_H
#define MODEL_STATEMACHINE_H

#include "openjaus/model/Trigger.h"
#include "openjaus/model/State.h"
#include "openjaus/model/Transition.h"
#include "openjaus/model/Message.h"
#include <list>
#include <vector>
#include <string>
#include "openjaus/types.h"
#include <ostream>

// Start of user code for additional includes
#include "openjaus/model/MessageCallback.h"
#include "openjaus/model/MessageCallbackImpl.h"
#include "openjaus/model/TriggerCallbackImpl.h"
#include "openjaus/model/StaticCallbackImpl.h"
// End of user code

namespace openjaus
{
namespace model
{
class Trigger;
class State;
class Transition;
class Message;

/// \class StateMachine StateMachine.h
/// \brief This is a brief description.
/// Detailed description.
/// \author Name (name@email.com)
class OPENJAUS_EXPORT StateMachine 
{
public:
	StateMachine(); 
	virtual ~StateMachine();
	// Start of user code for additional constructors
	// End of user code
	/// Accessor to get the value of name.
	std::string getName() const;

	/// Accessor to set value of name.
	/// \param name The value of the new name.
	bool setName(std::string name);

	/// Accessor to get the value of stateStack.
	const std::list< State * >& getStateStack() const;

	/// Accessor to set value of stateStack.
	/// \param stateStack The value of the new stateStack.
	bool setStateStack(std::list< State * > stateStack);

	/// Accessor to get the value of states.
	const std::vector< State* >& getStates() const;

	/// Accessor to set value of states.
	/// \param states The value of the new states.
	bool setStates(const State& states);

	/// Accessor to get the value of parentState.
	State* getParentState() const;

	/// Accessor to set value of parentState.
	/// \param parentState The value of the new parentState.
	bool setParentState(State* parentState);

	/// Accessor to get the value of currentState.
	State* getCurrentState() const;


	/// Accessor to get the value of startingState.
	State* getStartingState() const;

	/// Accessor to set value of startingState.
	/// \param startingState The value of the new startingState.
	bool setStartingState(State* startingState);

	/// Accessor to get the value of defaultStateTransitions.
	const std::vector< Transition* >& getDefaultStateTransitions() const;

	/// Accessor to set value of defaultStateTransitions.
	/// \param defaultStateTransitions The value of the new defaultStateTransitions.
	bool setDefaultStateTransitions(const Transition& defaultStateTransitions);

	/// Operation processTrigger.
	/// \param trigger 
	 bool processTrigger(Trigger *trigger);

	/// Operation getState.
	/// \param name 
	 State getState(std::string name);

	/// Operation addState.
	/// \param state 
	 int addState(State &state);

	/// Operation addDefaultStateTransition.
	/// \param transition 
	 int addDefaultStateTransition(Transition &transition);


	/// \param trigger 
	 Message* getResponse(Trigger *trigger);

	/// Operation setCurrentState.
	/// \param state 
	 bool setCurrentState(State *state);

	/// Operation executeTransition.
	/// \param transition 
	 bool executeTransition(Transition *transition);

	/// Operation entry.
	 void entry();

	/// Operation exit.
	 void exit();

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const StateMachine& object);

protected:
	// Member attributes & references
	std::string name;
	std::list< State * > stateStack;
	std::vector< State* > states;
	State *parentState;
	State *currentState;
	State *startingState;
	std::vector< Transition* > defaultStateTransitions;

// Start of user code for additional member data
	std::vector< MessageCallback* > callbacks;

public:
	template <class MessageType, class CallbackClass>
	void addMessageCallback(bool(CallbackClass::*callback)(MessageType &messageRef), CallbackClass* object)
	{
		MessageCallbackImpl<MessageType, CallbackClass> *cbImpl = new MessageCallbackImpl<MessageType, CallbackClass>(callback, object);
		callbacks.push_back(cbImpl);
	}

	template <class CallbackClass>
	void addMessageCallback(bool(CallbackClass::*callback)(Trigger *trigger), CallbackClass* object)
	{
		TriggerCallbackImpl<CallbackClass> *cbImpl = new TriggerCallbackImpl<CallbackClass>(callback, object);
		callbacks.push_back(cbImpl);
	}

	template <class MessageType>
	void addMessageCallback(bool(*callback)(MessageType &messageRef))
	{
		StaticCallbackImpl<MessageType> *cbImpl = new StaticCallbackImpl<MessageType>(callback);
		callbacks.push_back(cbImpl);
	}

// End of user code

}; // class StateMachine

// Start of user code for inline functions
// End of user code



} // namespace model
} // namespace openjaus

#endif // MODEL_STATEMACHINE_H

