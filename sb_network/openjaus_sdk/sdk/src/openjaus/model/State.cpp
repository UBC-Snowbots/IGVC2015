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

#include "openjaus/model/State.h"
#include <sstream>
// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace model
{

// Start of user code for default constructor:
State::State() :
	name(),
	stateMachines(),
	transitions(),
	loop(NULL)
{
	loop = new Transition();
	loop->setEndState(this);
	loop->setType(LOOPBACK);

	entryActions = new std::vector< Action * >;
	exitActions = new std::vector< Action * >;
}
// End of user code

// Start of user code for default destructor:
State::~State()
{
	for(size_t i = 0; i < exitActions->size(); i++)
	{
		delete exitActions->at(i);
	}

	for(size_t i = 0; i < entryActions->size(); i++)
	{
		delete entryActions->at(i);
	}

	delete entryActions;
	delete exitActions;
	delete loop;
}
// End of user code

std::string State::getName() const
{
	// Start of user code for accessor getName:
	
	return name;
	// End of user code
}

bool State::setName(std::string name)
{
	// Start of user code for accessor setName:
	this->name = name;
	return true;
	// End of user code
}


bool State::isStartingState() const
{
	// Start of user code for accessor getStartingState:
	
	return startingState;
	// End of user code
}

bool State::setStartingState(bool startingState)
{
	// Start of user code for accessor setStartingState:
	this->startingState = startingState;
	return true;
	// End of user code
}


const std::vector< StateMachine* >& State::getStateMachines() const
{
	// Start of user code for accessor getStateMachines:
	
	return stateMachines;
	// End of user code
}

bool State::setStateMachines(const StateMachine& stateMachines)
{
	// Start of user code for accessor setStateMachines:
	return true;
	// End of user code
}


const std::vector< Transition* >& State::getTransitions() const
{
	// Start of user code for accessor getTransitions:
	
	return transitions;
	// End of user code
}

bool State::setTransitions(const Transition& transitions)
{
	// Start of user code for accessor setTransitions:
	return true;
	// End of user code
}


std::vector< Action* >* State::getEntryActions() const
{
	// Start of user code for accessor getEntryActions:
	
	return entryActions;
	// End of user code
}

bool State::setEntryActions(Action* entryActions)
{
	// Start of user code for accessor setEntryActions:
	return true;
	// End of user code
}


std::vector< Action* >* State::getExitActions() const
{
	// Start of user code for accessor getExitActions:
	
	return exitActions;
	// End of user code
}

bool State::setExitActions(Action* exitActions)
{
	// Start of user code for accessor setExitActions:
	return true;
	// End of user code
}


StateMachine* State::getParentStateMachine() const
{
	// Start of user code for accessor getParentStateMachine:
	
	return parentStateMachine;
	// End of user code
}

bool State::setParentStateMachine(StateMachine* parentStateMachine)
{
	// Start of user code for accessor setParentStateMachine:
	this->parentStateMachine = parentStateMachine;
	return true;
	// End of user code
}


Transition* State::getLoop() const
{
	// Start of user code for accessor getLoop:
	
	return loop;
	// End of user code
}

bool State::setLoop(Transition* loop)
{
	// Start of user code for accessor setLoop:
	this->loop = loop;
	return true;
	// End of user code
}



// Class Methods
Transition* State::processTrigger(Trigger *trigger)
{
	// Start of user code for method processTrigger:

	// Attempt to process trigger in nested state machines
	for(unsigned int i = 0; i < stateMachines.size(); i++)
	{
		bool triggerProcessed = stateMachines[i]->processTrigger(trigger);
		if(triggerProcessed)
		{
			return loop;
		}
	}

	// Attempt to process trigger in transitions
	for(unsigned int i = 0; i < transitions.size(); i++)
	{
		if(transitions[i]->processTrigger(trigger))
		{
			return transitions[i];
		}
	}

	return NULL;
	// End of user code
}


StateMachine State::getStateMachine(std::string stateMachineName)
{
	// Start of user code for method getStateMachine:
	StateMachine result;

	return result;
	// End of user code
}


int State::addStateMachine(StateMachine &stateMachine)
{
	// Start of user code for method addStateMachine:
	stateMachines.push_back(&stateMachine);

	return true;
	// End of user code
}


int State::addTransition(Transition &transition)
{
	// Start of user code for method addTransition:
	transitions.push_back(&transition);

	return true;
	// End of user code
}


Message* State::getResponse(Trigger *trigger)
{
	// Start of user code for method getResponse:

	// Attempt to get response from nested state machines
	for(unsigned int i = 0; i < stateMachines.size(); i++)
	{
		model::Message* response = stateMachines[i]->getResponse(trigger);
		if(response)
		{
			return response;
		}
	}

	// Attempt to get response from transitions
	for(unsigned int i = 0; i < transitions.size(); i++)
	{
		model::Message* response = transitions[i]->getResponse(trigger);
		if(response)
		{
			return response;
		}
	}

	return NULL;
	// End of user code
}


void State::entry()
{
	// Start of user code for method entry:
	for(unsigned int i = 0; i < stateMachines.size(); i++)
	{
		stateMachines[i]->entry();
	}

	for(size_t i = 0; i < entryActions->size(); ++i)
	{
		entryActions->at(i)->execute();
	}
	// End of user code
}


void State::exit()
{
	// Start of user code for method exit:
	for(unsigned int i = 0; i < stateMachines.size(); i++)
	{
		stateMachines[i]->exit();
	}

	for(size_t i = 0; i < exitActions->size(); ++i)
	{
		exitActions->at(i)->execute();
	}
	// End of user code
}




std::string State::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << name;
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const State& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
// End of user code

} // namespace model
} // namespace openjaus

