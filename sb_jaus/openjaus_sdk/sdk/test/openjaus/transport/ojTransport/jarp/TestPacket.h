// File Header Here

#include "openjaus/transport/ojTransport/jarp/Packet.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestPacket : public CxxTest::TestSuite
{
public:
	// Start of user code TestPacket:
	openjaus::transport::ojTransport::jarp::Packet *packet;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		packet = new openjaus::transport::ojTransport::jarp::Packet();	
	
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
	
	void testTransportTypeAccessors()
	{
		// Start of user code testTransportTypeAccessors:
		//short transportType;		
		//packet->setTransportType(transportType);
		packet->getTransportType();
		// End of user code
	}
	
	void testApplicationTypeAccessors()
	{
		// Start of user code testApplicationTypeAccessors:
		//short applicationType;		
		//packet->setApplicationType(applicationType);
		packet->getApplicationType();
		// End of user code
	}
	
	void testTransportLengthAccessors()
	{
		// Start of user code testTransportLengthAccessors:
		//unsigned char transportLength;		
		//packet->setTransportLength(transportLength);
		packet->getTransportLength();
		// End of user code
	}
	
	void testApplicationLengthAccessors()
	{
		// Start of user code testApplicationLengthAccessors:
		//unsigned char applicationLength;		
		//packet->setApplicationLength(applicationLength);
		packet->getApplicationLength();
		// End of user code
	}
	
	void testOperationCodeAccessors()
	{
		// Start of user code testOperationCodeAccessors:
		//short operationCode;		
		//packet->setOperationCode(operationCode);
		packet->getOperationCode();
		// End of user code
	}
	
	void testSenderTransportAddressAccessors()
	{
		// Start of user code testSenderTransportAddressAccessors:
		//unsigned char * senderTransportAddress;		
		//packet->setSenderTransportAddress(senderTransportAddress);
		packet->getSenderTransportAddress();
		// End of user code
	}
	
	void testSenderApplicationAddressAccessors()
	{
		// Start of user code testSenderApplicationAddressAccessors:
		//unsigned char * senderApplicationAddress;		
		//packet->setSenderApplicationAddress(senderApplicationAddress);
		packet->getSenderApplicationAddress();
		// End of user code
	}
	
	void testDestinationTransportAddressAccessors()
	{
		// Start of user code testDestinationTransportAddressAccessors:
		//unsigned char * destinationTransportAddress;		
		//packet->setDestinationTransportAddress(destinationTransportAddress);
		packet->getDestinationTransportAddress();
		// End of user code
	}
	
	void testDestinationApplicationAddressAccessors()
	{
		// Start of user code testDestinationApplicationAddressAccessors:
		//unsigned char * destinationApplicationAddress;		
		//packet->setDestinationApplicationAddress(destinationApplicationAddress);
		packet->getDestinationApplicationAddress();
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

