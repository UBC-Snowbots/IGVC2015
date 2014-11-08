// File Header Here

#include "openjaus/services/Event.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestEvent : public CxxTest::TestSuite
{
public:
	// Start of user code TestEvent:
	openjaus::services::Event *event;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		event = new openjaus::services::Event();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete event;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(event);
		// End of user code
	}
	
	void testRate_HzAccessors()
	{
		// Start of user code testRate_HzAccessors:
		//double rate_Hz;		
		//event->setRate_Hz(rate_Hz);
		//event->getRate_Hz();
		// End of user code
	}
	
	void testRequestIdAccessors()
	{
		// Start of user code testRequestIdAccessors:
		//unsigned char requestId;		
		//event->setRequestId(requestId);
		//event->getRequestId();
		// End of user code
	}
	
	void testMessageAccessors()
	{
		// Start of user code testMessageAccessors:
		//openjaus::services::Message message;		
		//event->setMessage(message);
		//event->getMessage();
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

