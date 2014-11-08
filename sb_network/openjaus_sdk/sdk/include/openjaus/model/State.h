/**
\file State.h

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
#ifndef MODEL_STATE_H
#define MODEL_STATE_H

#include "openjaus/model/Transition.h"
#include "openjaus/model/Trigger.h"
#include "openjaus/model/StateMachine.h"
#include "openjaus/model/Message.h"
#include "openjaus/model/Action.h"
#include <vector>
#include <string>
#include "openjaus/types.h"
#include <ostream>

// Start of user code for additional includes
#include "openjaus/model/ActionImpl.h"
// End of user code

namespace openjaus
{
namespace model
{
class Transition;
class Trigger;
class StateMachine;
class Message;
class Action;

/// \class State State.h
/// \brief This is a brief description.
/// Detailed description.
/// \author Name (name@email.com)
class OPENJAUS_EXPORT State 
{
public:
	State(); 
	virtual ~State();
	// Start of user code for additional constructors
	// End of user code
	/// Accessor to get the value of name.
	std::string getName() const;

	/// Accessor to set value of name.
	/// \param name The value of the new name.
	bool setName(std::string name);

	/// Accessor to get the value of startingState.
	bool isStartingState() const;

	/// Accessor to set value of startingState.
	/// \param startingState The value of the new startingState.
	bool setStartingState(bool startingState);

	/// Accessor to get the value of stateMachines.
	const std::vector< StateMachine* >& getStateMachines() const;

	/// Accessor to set value of stateMachines.
	/// \param stateMachines The value of the new stateMachines.
	bool setStateMachines(const StateMachine& stateMachines);

	/// Accessor to get the value of transitions.
	const std::vector< Transition* >& getTransitions() const;

	/// Accessor to set value of transitions.
	/// \param transitions The value of the new transitions.
	bool setTransitions(const Transition& transitions);

	/// Accessor to get the value of entryActions.
	std::vector< Action* >* getEntryActions() const;

	/// Accessor to set value of entryActions.
	/// \param entryActions The value of the new entryActions.
	bool setEntryActions(Action* entryActions);

	/// Accessor to get the value of exitActions.
	std::vector< Action* >* getExitActions() const;

	/// Accessor to set value of exitActions.
	/// \param exitActions The value of the new exitActions.
	bool setExitActions(Action* exitActions);

	/// Accessor to get the value of parentStateMachine.
	StateMachine* getParentStateMachine() const;

	/// Accessor to set value of parentStateMachine.
	/// \param parentStateMachine The value of the new parentStateMachine.
	bool setParentStateMachine(StateMachine* parentStateMachine);

	/// Accessor to get the value of loop.
	Transition* getLoop() const;

	/// Accessor to set value of loop.
	/// \param loop The value of the new loop.
	bool setLoop(Transition* loop);


	/// \param trigger 
	virtual Transition* processTrigger(Trigger *trigger);

	/// Operation getStateMachine.
	/// \param stateMachineName 
	 StateMachine getStateMachine(std::string stateMachineName);

	/// Operation addStateMachine.
	/// \param stateMachine 
	 int addStateMachine(StateMachine &stateMachine);

	/// Operation addTransition.
	/// \param transition 
	 int addTransition(Transition &transition);


	/// \param trigger 
	 Message* getResponse(Trigger *trigger);

	/// Operation entry.
	 void entry();

	/// Operation exit.
	 void exit();

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const State& object);

protected:
	// Member attributes & references
	std::string name;
	bool startingState;
	std::vector< StateMachine* > stateMachines;
	std::vector< Transition* > transitions;
	std::vector< Action* > *entryActions;
	std::vector< Action* > *exitActions;
	StateMachine *parentStateMachine;
	Transition *loop;

// Start of user code for additional member data
public:
	template <class C>
	void addEntryAction(void(C::*fp)(), C* object)
	{
		ActionImpl<C> *action = new ActionImpl<C>(fp, object);
		entryActions->push_back(action);
	}

	template <class C>
	void addExitAction(void(C::*fp)(), C* object)
	{
		ActionImpl<C> *action = new ActionImpl<C>(fp, object);
		exitActions->push_back(action);
	}

// End of user code

}; // class State

// Start of user code for inline functions
// End of user code



} // namespace model
} // namespace openjaus

#endif // MODEL_STATE_H

