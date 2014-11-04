/**
\file TestJudpInterface.h

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

#include "openjaus/transport/AS5669/JudpInterface.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestJudpInterface : public CxxTest::TestSuite
{
public:
	// Start of user code TestJudpInterface:
	openjaus::transport::AS5669::JudpInterface *judpInterface;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		judpInterface = new openjaus::transport::AS5669::JudpInterface();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete judpInterface;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(judpInterface);
		// End of user code
	}
	
	void testHeaderMapAccessors()
	{
		// Start of user code testHeaderMapAccessors:
		//std::map<uint32_t, std::list<openjaus::transport::CompressedHeader *>> headerMap;		
		//judpInterface->setHeaderMap(headerMap);
		//judpInterface->getHeaderMap();
		// End of user code
	}
	
	void testDatagramSocketAccessors()
	{
		// Start of user code testDatagramSocketAccessors:
		//openjaus::system::MulticastSocket multicastSocket;		
		//judpInterface->setMulticastSocket(multicastSocket);
		//judpInterface->getMulticastSocket();
		// End of user code
	}
	
	void testUdpAddressAccessors()
	{
		// Start of user code testUdpAddressAccessors:
		//openjaus::transport::AS5669::JudpAddress udpAddress;		
		//judpInterface->setUdpAddress(udpAddress);
		//judpInterface->getUdpAddress();
		// End of user code
	}
	
	void testDiscoveryThreadAccessors()
	{
		// Start of user code testDiscoveryThreadAccessors:
		//openjaus::system::Thread discoveryThread;		
		//judpInterface->setDiscoveryThread(discoveryThread);
		//judpInterface->getDiscoveryThread();
		// End of user code
	}
	
	void testDiscoverySocketAccessors()
	{
		// Start of user code testDiscoverySocketAccessors:
		//openjaus::system::DatagramSocket discoverySocket;		
		//judpInterface->setDiscoverySocket(discoverySocket);
		//judpInterface->getDiscoverySocket();
		// End of user code
	}
	
	void testRun()
	{
		// Start of user code testRun:
		// End of user code
	}
	
	void testStop()
	{
		// Start of user code testStop:
		// End of user code
	}
	
	void testGetDestinationAddresses()
	{
		// Start of user code testGetDestinationAddresses:
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

