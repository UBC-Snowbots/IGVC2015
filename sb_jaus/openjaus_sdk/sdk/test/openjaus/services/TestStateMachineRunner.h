// File Header Here

#include "openjaus/services/StateMachineRunner.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestStateMachineRunner : public CxxTest::TestSuite
{
public:
	// Start of user code TestStateMachineRunner:
	openjaus::services::StateMachineRunner *stateMachineRunner;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		stateMachineRunner = new openjaus::services::StateMachineRunner();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete stateMachineRunner;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(stateMachineRunner);
		// End of user code
	}
	
	void testNameAccessors()
	{
		// Start of user code testNameAccessors:
		//std::string name;		
		//stateMachineRunner->setName(name);
		stateMachineRunner->getName();
		// End of user code
	}
	
	void testQueueAccessors()
	{
		// Start of user code testQueueAccessors:
		//openjaus::system::PriorityQueue queue;		
		//stateMachineRunner->setQueue(queue);
		stateMachineRunner->getQueue();
		// End of user code
	}
	
	void testStateMachineAccessors()
	{
		// Start of user code testStateMachineAccessors:
		//openjaus::model::StateMachine stateMachine;		
		//stateMachineRunner->setStateMachine(stateMachine);
		stateMachineRunner->getStateMachine();
		// End of user code
	}
	
	void testPush()
	{
		// Start of user code testPush:
		// End of user code
	}
	
	void testStop()
	{
		// Start of user code testStop:
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

