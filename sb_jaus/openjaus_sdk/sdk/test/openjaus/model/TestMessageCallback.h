// File Header Here

#include "openjaus/model/MessageCallback.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestMessageCallback : public CxxTest::TestSuite
{
public:
	// Start of user code TestMessageCallback:
	openjaus::model::MessageCallback<void *void *> *messageCallback;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		messageCallback = new openjaus::model::MessageCallback<void *void *>();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete messageCallback;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(messageCallback);
		// End of user code
	}
	
	void testIdAccessors()
	{
		// Start of user code testIdAccessors:
		//uint16_t id;		
		//messageCallback->setId(id);
		//messageCallback->getId();
		// End of user code
	}
	
	void testProcessTrigger()
	{
		// Start of user code testProcessTrigger:
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

