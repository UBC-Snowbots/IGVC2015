// File Header Here

#include "openjaus/transport/AS5669A/jtcp/Packet.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestPacket : public CxxTest::TestSuite
{
public:
	// Start of user code TestPacket:
	openjaus::transport::AS5669A::jtcp::Packet *packet;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		packet = new openjaus::transport::AS5669A::jtcp::Packet();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete packet;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(packet);
		// End of user code
	}
	
	void testVersionAccessors()
	{
		// Start of user code testVersionAccessors:
		//unsigned char version;		
		//packet->setVersion(version);
		packet->getVersion();
		// End of user code
	}
	
	void testMessagesAccessors()
	{
		// Start of user code testMessagesAccessors:
		//openjaus::transport::AS5669A::Header messages;		
		//packet->setMessages(messages);
		packet->getMessages();
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

