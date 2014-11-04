// File Header Here

#include "openjaus/model/fields/Message.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestMessage : public CxxTest::TestSuite
{
public:
	// Start of user code TestMessage:
	openjaus::model::fields::Message *message;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		message = new openjaus::model::fields::Message();	
	
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
	
	void testMessageAccessors()
	{
		// Start of user code testMessageAccessors:
		//openjaus::services::Message message;		
		//message->setMessage(message);
		//message->getMessage();
		// End of user code
	}
	
	void testBufferAccessors()
	{
		// Start of user code testBufferAccessors:
		//openjaus::system::Buffer buffer;		
		//message->setBuffer(buffer);
		//message->getBuffer();
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

