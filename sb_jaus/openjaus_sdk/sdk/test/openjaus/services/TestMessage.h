// File Header Here

#include "openjaus/services/Message.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestMessage : public CxxTest::TestSuite
{
public:
	// Start of user code TestMessage:
	openjaus::services::Message *message;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		message = new openjaus::services::Message();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete message;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(message);
		// End of user code
	}
	
	void testInterpretationAccessors()
	{
		// Start of user code testInterpretationAccessors:
		//std::string Description;		
		//message->setDescription(Description);
		//message->getDescription();
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

