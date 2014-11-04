// File Header Here

#include "openjaus/services/Connection.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestConnection : public CxxTest::TestSuite
{
public:
	// Start of user code TestConnection:
	openjaus::services::Connection *connection;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		connection = new openjaus::services::Connection();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete connection;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(connection);
		// End of user code
	}
	
	void testEventIdAccessors()
	{
		// Start of user code testEventIdAccessors:
		//uint8_t eventId;		
		//connection->setEventId(eventId);
		//connection->getEventId();
		// End of user code
	}
	
	void testLocalIdAccessors()
	{
		// Start of user code testLocalIdAccessors:
		//uint32_t localId;		
		//connection->setLocalId(localId);
		//connection->getLocalId();
		// End of user code
	}
	
	void testRequestIdAccessors()
	{
		// Start of user code testRequestIdAccessors:
		//uint8_t requestId;		
		//connection->setRequestId(requestId);
		//connection->getRequestId();
		// End of user code
	}
	
	void testRate_HzAccessors()
	{
		// Start of user code testRate_HzAccessors:
		//double rate_Hz;		
		//connection->setRate_Hz(rate_Hz);
		//connection->getRate_Hz();
		// End of user code
	}
	
	void testTypeAccessors()
	{
		// Start of user code testTypeAccessors:
		//openjaus::services::ConnectionType type;		
		//connection->setType(type);
		//connection->getType();
		// End of user code
	}
	
	void testSequenceNumberAccessors()
	{
		// Start of user code testSequenceNumberAccessors:
		//uint8_t sequenceNumber;		
		//connection->setSequenceNumber(sequenceNumber);
		//connection->getSequenceNumber();
		// End of user code
	}
	
	void testActiveAccessors()
	{
		// Start of user code testActiveAccessors:
		//bool active;		
		//connection->setActive(active);
		//connection->getActive();
		// End of user code
	}
	
	void testResponseIdAccessors()
	{
		// Start of user code testResponseIdAccessors:
		//uint16_t responseId;		
		//connection->setResponseId(responseId);
		//connection->getResponseId();
		// End of user code
	}
	
	void testTimeoutAccessors()
	{
		// Start of user code testTimeoutAccessors:
		//openjaus::system::Time timeout;		
		//connection->setTimeout(timeout);
		//connection->getTimeout();
		// End of user code
	}
	
	void testAddressAccessors()
	{
		// Start of user code testAddressAccessors:
		//openjaus::transport::Address address;		
		//connection->setAddress(address);
		//connection->getAddress();
		// End of user code
	}
	
	void testQueryAccessors()
	{
		// Start of user code testQueryAccessors:
		//openjaus::services::Message query;		
		//connection->setQuery(query);
		//connection->getQuery();
		// End of user code
	}
	
	void testTimerAccessors()
	{
		// Start of user code testTimerAccessors:
		//openjaus::system::Timer timer;		
		//connection->setTimer(timer);
		//connection->getTimer();
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

