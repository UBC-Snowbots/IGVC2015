// File Header Here

#include "openjaus/model/TransitionExtension.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestTransitionExtension : public CxxTest::TestSuite
{
public:
	// Start of user code TestTransitionExtension:
	openjaus::model::TransitionExtension *transitionExtension;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		transitionExtension = new openjaus::model::TransitionExtension();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete transitionExtension;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(transitionExtension);
		// End of user code
	}
	
	void testParentStateAccessors()
	{
		// Start of user code testParentStateAccessors:
		//openjaus::model::State parentState;		
		//transitionExtension->setParentState(parentState);
		//transitionExtension->getParentState();
		// End of user code
	}
	
	void testParentStateMachineAccessors()
	{
		// Start of user code testParentStateMachineAccessors:
		//openjaus::model::StateMachine parentStateMachine;		
		//transitionExtension->setParentStateMachine(parentStateMachine);
		//transitionExtension->getParentStateMachine();
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

