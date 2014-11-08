/**
\file TestConnection.h

\par Copyright
Copyright (c) 2012, OpenJAUS, LLC
All rights reserved.

This file is part of the OpenJAUS Software Development Kit (SDK). This 
software is distributed under one of two licenses, the OpenJAUS SDK 
Commercial End User License Agreement or the OpenJAUS SDK Non-Commercial 
End User License Agreement. The appropriate licensing details were included 
in with your developer credentials and software download. See the LICENSE 
file included with this software for full details.
 
THIS SOFTWARE IS PROVIDED BY THE LICENSOR (OPENJAUS LCC) "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE LICENSOR BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THE SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE. THE LICENSOR DOES NOT WARRANT THAT THE LICENSED SOFTWARE WILL MEET
LICENSEE'S REQUIREMENTS OR THAT THE OPERATION OF THE LICENSED SOFTWARE
WILL BE UNINTERRUPTED OR ERROR-FREE, OR THAT ERRORS IN THE LICENSED
SOFTWARE WILL BE CORRECTED.

\ Software History
- [2011-08-23] - Added AS6057: Manipulators
- [2011-08-01] - Added AS6060: Environment Sensing
- [2011-06-16] - First Release 

*/

#include "openjaus/model/Connection.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestConnection : public CxxTest::TestSuite
{
public:
	// Start of user code TestConnection:
	openjaus::model::Connection *connection;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		connection = new openjaus::model::Connection();	
	
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
		//openjaus::model::ConnectionType type;		
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
	
	void testLastRequestTypeAccessors()
	{
		// Start of user code testLastRequestTypeAccessors:
		//openjaus::model::ConnectionRequestType lastRequestType;		
		//connection->setLastRequestType(lastRequestType);
		//connection->getLastRequestType();
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
		//openjaus::model::Message query;		
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

