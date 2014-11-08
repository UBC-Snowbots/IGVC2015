/**
\file TestDatagramSocket.h

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

#include "openjaus/system/DatagramSocket.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
#include <cstdio>
// End of user code

class TestDatagramSocket : public CxxTest::TestSuite
{
public:
	// Start of user code TestDatagramSocket:
	openjaus::system::DatagramSocket *datagramSocket;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		datagramSocket = new openjaus::system::DatagramSocket();
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete datagramSocket;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(datagramSocket);
		// End of user code
	}
	
	void testOpen()
	{
		// Start of user code testOpen:
		openjaus::system::DatagramSocket loopbackUdp(openjaus::system::InetAddress::getLocalHost(), 0);
		std::cout 	<< "\nRunning testDatagramSocket\nSocket = "
					<< loopbackUdp.toString() << "\n";

		TS_ASSERT_EQUALS("127.0.0.1", loopbackUdp.getIpAddress().toString());
		TS_ASSERT_DIFFERS(0, loopbackUdp.getPort());
		// End of user code
	}
	
	void testSend()
	{
		// Start of user code testSend:
		std::cout << "Running testSend" << "\n";
		openjaus::system::DatagramSocket loopbackUdp(openjaus::system::InetAddress::getLocalHost(), 0);
		openjaus::system::Packet txPacket(32);
		txPacket.setAddress(loopbackUdp.getIpAddress());
		txPacket.setPort(loopbackUdp.getPort());
		std::sprintf(reinterpret_cast<char *>(txPacket.getBuffer()), "Send Test");

		int sentBytes = loopbackUdp.send(txPacket);
		TS_ASSERT_EQUALS(sentBytes, txPacket.getBufferSize())
		// End of user code
	}
	
	void testReceive()
	{
		// Start of user code testReceive:
		std::cout << "Running testReceive" << "\n";
		openjaus::system::DatagramSocket loopbackUdp(openjaus::system::InetAddress::getLocalHost(), 0);
		openjaus::system::Packet txPacket(32);
		txPacket.setAddress(loopbackUdp.getIpAddress());
		txPacket.setPort(loopbackUdp.getPort());
		std::sprintf(reinterpret_cast<char *>(txPacket.getBuffer()), "Send Test");
		loopbackUdp.send(txPacket);

		openjaus::system::Packet rxPacket(32);
		openjaus::system::Time timeout;
		timeout.setSeconds(-1);
		timeout.setMicroseconds(-1);
		loopbackUdp.setTimeout(timeout);
		loopbackUdp.receive(rxPacket);
		TS_ASSERT_EQUALS(std::string(reinterpret_cast<const char *>(rxPacket.getBuffer())), "Send Test");
		// End of user code
	}
	
	void testReuseAddress()
	{
		// Start of user code testReuseAddress:
		// End of user code
	}
	
	void testEnableBroadcast()
	{
		// Start of user code testEnableBroadcast:
		// End of user code
	}
	
	void testSetBufferSize()
	{
		// Start of user code testSetBufferSize:
		// End of user code
	}
	
	// Start of user code custom tests:
	void testTimeout()
	{
		// Start of user code testReceive:
		std::cout << "Running testTimeout" << "\n";
		openjaus::system::DatagramSocket loopbackUdp(openjaus::system::InetAddress::getLocalHost(), 0);
		openjaus::system::Time timeout;
		timeout.setSeconds(1);
		timeout.setMicroseconds(500000);
		loopbackUdp.setTimeout(timeout);

		openjaus::system::Packet rxPacket(32);
		int bytesReceived = loopbackUdp.receive(rxPacket);
		TS_ASSERT_EQUALS(bytesReceived, 0);
	}
	// End of user code
};

