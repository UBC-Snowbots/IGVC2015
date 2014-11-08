// File Header Here

#include "openjaus/transport/AS5669A/Header.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestHeader : public CxxTest::TestSuite
{
public:
	// Start of user code TestHeader:
	openjaus::transport::AS5669A::Header *header;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		header = new openjaus::transport::AS5669A::Header();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete header;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(header);
		// End of user code
	}
	
	void testMsgTypeAccessors()
	{
		// Start of user code testMsgTypeAccessors:
		//unsigned char msgType;		
		//header->setMsgType(msgType);
		header->getMsgType();
		// End of user code
	}
	
	void testHcFlagsAccessors()
	{
		// Start of user code testHcFlagsAccessors:
		//unsigned char hcFlags;		
		//header->setHcFlags(hcFlags);
		header->getHcFlags();
		// End of user code
	}
	
	void testDataSizeAccessors()
	{
		// Start of user code testDataSizeAccessors:
		//short dataSize;		
		//header->setDataSize(dataSize);
		header->getDataSize();
		// End of user code
	}
	
	void testHcNumberAccessors()
	{
		// Start of user code testHcNumberAccessors:
		//unsigned char hcNumber;		
		//header->setHcNumber(hcNumber);
		header->getHcNumber();
		// End of user code
	}
	
	void testHcLengthAccessors()
	{
		// Start of user code testHcLengthAccessors:
		//unsigned char hcLength;		
		//header->setHcLength(hcLength);
		header->getHcLength();
		// End of user code
	}
	
	void testBroadcastAccessors()
	{
		// Start of user code testBroadcastAccessors:
		//unsigned char broadcast;		
		//header->setBroadcast(broadcast);
		header->getBroadcast();
		// End of user code
	}
	
	void testDataFlagsAccessors()
	{
		// Start of user code testDataFlagsAccessors:
		//unsigned char dataFlags;		
		//header->setDataFlags(dataFlags);
		header->getDataFlags();
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

