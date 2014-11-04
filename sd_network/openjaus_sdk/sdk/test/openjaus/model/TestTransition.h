/**
\file TestTransition.h

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
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestTransition : public CxxTest::TestSuite
{
public:
	// Start of user code TestTransition:
	openjaus::model::Transition *transition;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		transition = new openjaus::model::Transition();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete transition;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(transition);
		// End of user code
	}
	
	void testNameAccessors()
	{
		// Start of user code testNameAccessors:
		//std::string name;		
		//transition->setName(name);
		transition->getName();
		// End of user code
	}
	
	void testTypeAccessors()
	{
		// Start of user code testTypeAccessors:
		//openjaus::model::TransitionType type;		
		//transition->setType(type);
		//transition->getType();
		// End of user code
	}
	
	void testActionSetsAccessors()
	{
		// Start of user code testActionSetsAccessors:
		//openjaus::model::ActionSet actionSets;		
		//transition->setActionSets(actionSets);
		transition->getActionSets();
		// End of user code
	}
	
	void testTriggersAccessors()
	{
		// Start of user code testTriggersAccessors:
		//openjaus::model::Trigger triggers;		
		//transition->setTriggers(triggers);
		transition->getTriggers();
		// End of user code
	}
	
	void testEndStateAccessors()
	{
		// Start of user code testEndStateAccessors:
		//openjaus::model::State endState;		
		//transition->setEndState(endState);
		transition->getEndState();
		// End of user code
	}
	
	void testStartStateAccessors()
	{
		// Start of user code testStartStateAccessors:
		//openjaus::model::State startState;		
		//transition->setStartState(startState);
		transition->getStartState();
		// End of user code
	}
	
	void testParentStateAccessors()
	{
		// Start of user code testParentStateAccessors:
		//openjaus::model::State parentState;		
		//transition->setParentState(parentState);
		//transition->getParentState();
		// End of user code
	}
	
	void testParentStateMachineAccessors()
	{
		// Start of user code testParentStateMachineAccessors:
		//openjaus::model::StateMachine parentStateMachine;		
		//transition->setParentStateMachine(parentStateMachine);
		//transition->getParentStateMachine();
		// End of user code
	}
	
	void testProcessTrigger()
	{
		// Start of user code testProcessTrigger:
		// End of user code
	}
	
	void testGetResponse()
	{
		// Start of user code testGetResponse:
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

