/**
\file TestState.h

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
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestState : public CxxTest::TestSuite
{
public:
	// Start of user code TestState:
	openjaus::model::State *state;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		state = new openjaus::model::State();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete state;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(state);
		// End of user code
	}
	
	void testNameAccessors()
	{
		// Start of user code testNameAccessors:
		//std::string name;		
		//state->setName(name);
		state->getName();
		// End of user code
	}
	
	void testStartingStateAccessors()
	{
		// Start of user code testStartingStateAccessors:
		//bool startingState;		
		//state->setStartingState(startingState);
		//state->getStartingState();
		// End of user code
	}
	
	void testStateMachinesAccessors()
	{
		// Start of user code testStateMachinesAccessors:
		//openjaus::model::StateMachine stateMachines;		
		//state->setStateMachines(stateMachines);
		state->getStateMachines();
		// End of user code
	}
	
	void testTransitionsAccessors()
	{
		// Start of user code testTransitionsAccessors:
		//openjaus::model::Transition transitions;		
		//state->setTransitions(transitions);
		state->getTransitions();
		// End of user code
	}
	
	void testEntryActionsAccessors()
	{
		// Start of user code testEntryActionsAccessors:
		//openjaus::model::Action entryActions;		
		//state->setEntryActions(entryActions);
		//state->getEntryActions();
		// End of user code
	}
	
	void testExitActionsAccessors()
	{
		// Start of user code testExitActionsAccessors:
		//openjaus::model::Action exitActions;		
		//state->setExitActions(exitActions);
		//state->getExitActions();
		// End of user code
	}
	
	void testParentStateMachineAccessors()
	{
		// Start of user code testParentStateMachineAccessors:
		//openjaus::model::StateMachine parentStateMachine;		
		//state->setParentStateMachine(parentStateMachine);
		//state->getParentStateMachine();
		// End of user code
	}
	
	void testLoopAccessors()
	{
		// Start of user code testLoopAccessors:
		//openjaus::model::Transition loop;		
		//state->setLoop(loop);
		//state->getLoop();
		// End of user code
	}
	
	void testProcessTrigger()
	{
		// Start of user code testProcessTrigger:
		// End of user code
	}
	
	void testGetStateMachine()
	{
		// Start of user code testGetStateMachine:
		// End of user code
	}
	
	void testAddStateMachine()
	{
		// Start of user code testAddStateMachine:
		// End of user code
	}
	
	void testAddTransition()
	{
		// Start of user code testAddTransition:
		// End of user code
	}
	
	void testGetResponse()
	{
		// Start of user code testGetResponse:
		// End of user code
	}
	
	void testEntry()
	{
		// Start of user code testEntry:
		// End of user code
	}
	
	void testExit()
	{
		// Start of user code testExit:
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

