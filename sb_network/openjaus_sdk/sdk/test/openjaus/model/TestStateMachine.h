/**
\file TestStateMachine.h

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
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestStateMachine : public CxxTest::TestSuite
{
public:
	// Start of user code TestStateMachine:
	openjaus::model::StateMachine *stateMachine;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		stateMachine = new openjaus::model::StateMachine();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete stateMachine;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(stateMachine);
		// End of user code
	}
	
	void testNameAccessors()
	{
		// Start of user code testNameAccessors:
		//std::string name;		
		//stateMachine->setName(name);
		stateMachine->getName();
		// End of user code
	}
	
	void testStateStackAccessors()
	{
		// Start of user code testStateStackAccessors:
		//std::list< openjaus::model::State * > stateStack;		
		//stateMachine->setStateStack(stateStack);
		//stateMachine->getStateStack();
		// End of user code
	}
	
	void testStatesAccessors()
	{
		// Start of user code testStatesAccessors:
		//openjaus::model::State states;		
		//stateMachine->setStates(states);
		stateMachine->getStates();
		// End of user code
	}
	
	void testParentStateAccessors()
	{
		// Start of user code testParentStateAccessors:
		//openjaus::model::State parentState;		
		//stateMachine->setParentState(parentState);
		stateMachine->getParentState();
		// End of user code
	}
	
	void testCurrentStateAccessors()
	{
		// Start of user code testCurrentStateAccessors:
		//openjaus::model::State currentState;		
		//stateMachine->setCurrentState(currentState);
		stateMachine->getCurrentState();
		// End of user code
	}
	
	void testStartingStateAccessors()
	{
		// Start of user code testStartingStateAccessors:
		//openjaus::model::State startingState;		
		//stateMachine->setStartingState(startingState);
		stateMachine->getStartingState();
		// End of user code
	}
	
	void testDefaultStateTransitionsAccessors()
	{
		// Start of user code testDefaultStateTransitionsAccessors:
		//openjaus::model::Transition defaultStateTransitions;		
		//stateMachine->setDefaultStateTransitions(defaultStateTransitions);
		//stateMachine->getDefaultStateTransitions();
		// End of user code
	}
	
	void testProcessTrigger()
	{
		// Start of user code testProcessTrigger:
		// End of user code
	}
	
	void testGetState()
	{
		// Start of user code testGetState:
		// End of user code
	}
	
	void testAddState()
	{
		// Start of user code testAddState:
		// End of user code
	}
	
	void testAddDefaultStateTransition()
	{
		// Start of user code testAddDefaultStateTransition:
		// End of user code
	}
	
	void testGetResponse()
	{
		// Start of user code testGetResponse:
		// End of user code
	}
	
	void testSetCurrentState()
	{
		// Start of user code testSetCurrentState:
		// End of user code
	}
	
	void testExecuteTransition()
	{
		// Start of user code testExecuteTransition:
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

