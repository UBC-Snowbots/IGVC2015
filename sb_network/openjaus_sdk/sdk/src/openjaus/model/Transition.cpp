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

#include "openjaus/model/Transition.h"
#include <sstream>
// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace model
{

// Start of user code for default constructor:
Transition::Transition() :
	name(),
	type(LOOPBACK),
	actionSets(),
	triggers(NULL),
	endState(NULL),
	startState(NULL),
	parentState(NULL),
	parentStateMachine(NULL)
{
}
// End of user code

// Start of user code for default destructor:
Transition::~Transition()
{
}
// End of user code

std::string Transition::getName() const
{
	// Start of user code for accessor getName:
	
	return name;
	// End of user code
}

bool Transition::setName(std::string name)
{
	// Start of user code for accessor setName:
	this->name = name;
	return true;
	// End of user code
}


TransitionType Transition::getType() const
{
	// Start of user code for accessor getType:
	
	return type;
	// End of user code
}

bool Transition::setType(TransitionType type)
{
	// Start of user code for accessor setType:
	this->type = type;
	return true;
	// End of user code
}


const std::vector< ActionSet* >& Transition::getActionSets() const
{
	// Start of user code for accessor getActionSets:
	
	return actionSets;
	// End of user code
}

bool Transition::setActionSets(const ActionSet& actionSets)
{
	// Start of user code for accessor setActionSets:
	return true;
	// End of user code
}


std::vector< Trigger* >* Transition::getTriggers() const
{
	// Start of user code for accessor getTriggers:
	
	return triggers;
	// End of user code
}

bool Transition::setTriggers(Trigger* triggers)
{
	// Start of user code for accessor setTriggers:
	return true;
	// End of user code
}


State* Transition::getEndState() const
{
	// Start of user code for accessor getEndState:
	
	return endState;
	// End of user code
}

bool Transition::setEndState(State* endState)
{
	// Start of user code for accessor setEndState:
	this->endState = endState;
	return true;
	// End of user code
}


State* Transition::getStartState() const
{
	// Start of user code for accessor getStartState:
	
	return startState;
	// End of user code
}

bool Transition::setStartState(State* startState)
{
	// Start of user code for accessor setStartState:
	this->startState = startState;
	return true;
	// End of user code
}


State* Transition::getParentState() const
{
	// Start of user code for accessor getParentState:
	
	return parentState;
	// End of user code
}

bool Transition::setParentState(State* parentState)
{
	// Start of user code for accessor setParentState:
	this->parentState = parentState;
	return true;
	// End of user code
}


StateMachine* Transition::getParentStateMachine() const
{
	// Start of user code for accessor getParentStateMachine:
	
	return parentStateMachine;
	// End of user code
}

bool Transition::setParentStateMachine(StateMachine* parentStateMachine)
{
	// Start of user code for accessor setParentStateMachine:
	this->parentStateMachine = parentStateMachine;
	return true;
	// End of user code
}



// Class Methods
bool Transition::processTrigger(Trigger *trigger)
{
	// Start of user code for method processTrigger:
	return NULL;
	// End of user code
}


Message* Transition::getResponse(Trigger *trigger)
{
	// Start of user code for method getResponse:
	model::Message* result = 0;

	return result;
	// End of user code
}




std::string Transition::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const Transition& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
// End of user code

} // namespace model
} // namespace openjaus

