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

#include "openjaus/model/StateMachine.h"
#include <sstream>
// Start of user code for additional includes
#include "openjaus/system/Exception.h"
// End of user code

namespace openjaus
{
namespace model
{

// Start of user code for default constructor:
StateMachine::StateMachine() :
		name(),
		stateStack(),
		states(),
		parentState(NULL),
		currentState(NULL),
		startingState(NULL),
		defaultStateTransitions(),
		callbacks()
{
}
// End of user code

// Start of user code for default destructor:
StateMachine::~StateMachine()
{
	for(size_t i = 0; i < callbacks.size(); i++)
	{
		delete callbacks[i];
	}

}
// End of user code

std::string StateMachine::getName() const
{
	// Start of user code for accessor getName:
	
	return name;
	// End of user code
}

bool StateMachine::setName(std::string name)
{
	// Start of user code for accessor setName:
	this->name = name;
	return true;
	// End of user code
}


const std::list< State * >& StateMachine::getStateStack() const
{
	// Start of user code for accessor getStateStack:
	
	return stateStack;
	// End of user code
}

bool StateMachine::setStateStack(std::list< State * > stateStack)
{
	// Start of user code for accessor setStateStack:
	this->stateStack = stateStack;
	return true;
	// End of user code
}


const std::vector< State* >& StateMachine::getStates() const
{
	// Start of user code for accessor getStates:
	
	return states;
	// End of user code
}

bool StateMachine::setStates(const State& states)
{
	// Start of user code for accessor setStates:
	return true;
	// End of user code
}


State* StateMachine::getParentState() const
{
	// Start of user code for accessor getParentState:
	
	return parentState;
	// End of user code
}

bool StateMachine::setParentState(State* parentState)
{
	// Start of user code for accessor setParentState:
	this->parentState = parentState;
	return true;
	// End of user code
}


State* StateMachine::getCurrentState() const
{
	// Start of user code for accessor getCurrentState:
	if(stateStack.empty())
	{
		return NULL;
	}
	return stateStack.front();
	// End of user code
}


State* StateMachine::getStartingState() const
{
	// Start of user code for accessor getStartingState:
	
	return startingState;
	// End of user code
}

bool StateMachine::setStartingState(State* startingState)
{
	// Start of user code for accessor setStartingState:
	if(!stateStack.empty())
	{
		LOG_DEBUG("State Machine: " << toString() << " attempted to set start state while state stack is not empty");
		return false;
	}

	// Set current state if it exists in the vector
	for(unsigned int i = 0; i < states.size(); i++)
	{
		if(states[i] == startingState)
		{
			this->startingState = startingState;
			LOG_DEBUG("State Machine: " << toString() << ", setting start state to: " << startingState->toString());
			return true;
		}
	}
	
	return false;
	// End of user code
}


const std::vector< Transition* >& StateMachine::getDefaultStateTransitions() const
{
	// Start of user code for accessor getDefaultStateTransitions:
	
	return defaultStateTransitions;
	// End of user code
}

bool StateMachine::setDefaultStateTransitions(const Transition& defaultStateTransitions)
{
	// Start of user code for accessor setDefaultStateTransitions:
	return true;
	// End of user code
}



// Class Methods
bool StateMachine::processTrigger(Trigger *trigger)
{
	// Start of user code for method processTrigger:
	if(stateStack.empty())
	{
		THROW_EXCEPTION("StateMachine: \"" << name <<
						"\" attempted to process trigger: " << trigger->toString() <<
						", while currentState is undefined");
	}

	for(size_t i = 0; i < callbacks.size(); i++)
	{
		if(callbacks[i]->processTrigger(trigger))
		{
			return true;
		}
	}

	Transition *transition = stateStack.front()->processTrigger(trigger);
	if(transition)
	{
		return executeTransition(transition);
	}

	for(size_t i = 0; i < defaultStateTransitions.size(); i++)
	{
		if(defaultStateTransitions[i]->processTrigger(trigger))
		{
			return executeTransition(defaultStateTransitions[i]);
		}
	}

	return false;
	// End of user code
}


State StateMachine::getState(std::string name)
{
	// Start of user code for method getState:
	for(unsigned int i=0; i < states.size(); i++)
	{
		if(states[i]->getName().compare(name) == 0)
		{
			return *states[i];
		}
	}

	THROW_EXCEPTION( "State: " << name << " Not Found in: " << toString() );

	// End of user code
}


int StateMachine::addState(State &state)
{
	// Start of user code for method addState:
	states.push_back(&state);

	return true;
	// End of user code
}


int StateMachine::addDefaultStateTransition(Transition &transition)
{
	// Start of user code for method addDefaultStateTransition:
	defaultStateTransitions.push_back(&transition);

	return defaultStateTransitions.size();
	// End of user code
}


Message* StateMachine::getResponse(Trigger *trigger)
{
	// Start of user code for method getResponse:
	if(stateStack.empty())
	{
		THROW_EXCEPTION("StateMachine: \"" << name <<
						"\" attempted to get response for trigger: " << trigger->toString() <<
						", while currentState is undefined");
	}

	Message* response = stateStack.front()->getResponse(trigger);
	if(response)
	{
		return response;
	}

	for(size_t i = 0; i < defaultStateTransitions.size(); i++)
	{
		response = defaultStateTransitions[i]->getResponse(trigger);
		if(response)
		{
			return response;
		}
	}

	return NULL;
	// End of user code
}


bool StateMachine::setCurrentState(State *state)
{
	// Start of user code for method setCurrentState:
	if(!state)
	{
		return false;
	}

	if(state == stateStack.front())
	{
		return true;
	}

	// Set current state if it exists in the vector
	for(unsigned int i = 0; i < states.size(); i++)
	{
		if(states[i] == state)
		{
			LOG_DEBUG("State Machine: " << name << ", switching to state: " << state->toString());
			stateStack.front()->exit();
			state->entry();
			stateStack.pop_front();
			stateStack.push_front(state);
			return true;
		}
	}

	THROW_EXCEPTION("State: " <<
					stateStack.front()->toString() <<
					" attempted to transition to invalid state: " <<
					state->toString() );
	// End of user code
}


bool StateMachine::executeTransition(Transition *transition)
{
	// Start of user code for method executeTransition:
	switch(transition->getType())
	{
		case LOOPBACK:
			return true;

		case SIMPLE:
			return setCurrentState(transition->getEndState());

		case PUSH:
		{
			State *state = transition->getEndState();
			// Set current state if it exists in the vector
			for(unsigned int i = 0; i < states.size(); i++)
			{
				if(states[i] == state)
				{
					LOG_DEBUG("State Machine: " << name << ", pushing to state: " << state->toString());
					state->entry();
					stateStack.push_front(state);
					return true;
				}
			}

			THROW_EXCEPTION("State: " <<
							stateStack.front()->toString() <<
							" attempted push to invalid state: " <<
							state->toString() );
			break;
		}

		case POP:
		{
			if(stateStack.size() < 2)
			{
				THROW_EXCEPTION("State: " <<
								stateStack.front()->toString() <<
								" attempted to pop with only one state remaining in stack");
			}

			LOG_DEBUG("State Machine: " << name << ", popping from state: " << stateStack.front()->toString());
			stateStack.front()->exit();
			stateStack.pop_front();
			LOG_DEBUG("State Machine: " << name << ", popped to state: " << stateStack.front()->toString());
			return true;
		}
	}

	return false;
	// End of user code
}


void StateMachine::entry()
{
	// Start of user code for method entry:
	stateStack.push_front(startingState);
	stateStack.front()->entry();
	// End of user code
}


void StateMachine::exit()
{
	// Start of user code for method exit:
	stateStack.front()->exit();
	stateStack.pop_front();
	// End of user code
}




std::string StateMachine::toString() const
{	
	// Start of user code for toString
	return name;
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const StateMachine& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
// End of user code

} // namespace model
} // namespace openjaus

