// File Header Here

#include "openjaus/model/StateExtension.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestStateExtension : public CxxTest::TestSuite
{
public:
	// Start of user code TestStateExtension:
	openjaus::model::StateExtension *stateExtension;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		stateExtension = new openjaus::model::StateExtension();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete stateExtension;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(stateExtension);
		// End of user code
	}
	
	void testStartingStateAccessors()
	{
		// Start of user code testStartingStateAccessors:
		//bool startingState;		
		//stateExtension->setStartingState(startingState);
		//stateExtension->getStartingState();
		// End of user code
	}
	
	void testParentStateMachineAccessors()
	{
		// Start of user code testParentStateMachineAccessors:
		//openjaus::model::StateMachine parentStateMachine;		
		//stateExtension->setParentStateMachine(parentStateMachine);
		//stateExtension->getParentStateMachine();
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

