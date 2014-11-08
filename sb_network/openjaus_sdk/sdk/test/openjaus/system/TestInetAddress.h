/**
\file TestInetAddress.h

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

#include "openjaus/system/InetAddress.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestInetAddress : public CxxTest::TestSuite
{
public:
	// Start of user code TestInetAddress:
	openjaus::system::InetAddress *inetAddress;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		inetAddress = new openjaus::system::InetAddress();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete inetAddress;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(inetAddress);
		// End of user code
	}
	
	void testValueAccessors()
	{
		// Start of user code testValueAccessors:
		//int value;		
		//inetAddress->setValue(value);
		inetAddress->getValue();
		// End of user code
	}
	
	void testGetByName()
	{
		// Start of user code testGetByName:
		openjaus::system::InetAddress localHost = openjaus::system::InetAddress::getByName("127.0.0.1");
		std::cout << "\nRunning testGetByName\nLocal Host = " << localHost.toString() << "\n";
		TS_ASSERT_EQUALS("127.0.0.1", localHost.toString());
		// End of user code
	}
	
	void testGetLocalHost()
	{
		// Start of user code testGetLocalHost:
		openjaus::system::InetAddress localHost = openjaus::system::InetAddress::getLocalHost();
		std::cout << "\nRunning testGetLocalHost\nLocal Host = " << localHost.toString() << "\n";
		TS_ASSERT_EQUALS("127.0.0.1", localHost.toString());
		// End of user code
	}
	
	void testAnyAddress()
	{
		// Start of user code testAnyAddress:
		// End of user code
	}
	
	void testHash()
	{
		// Start of user code testHash:
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

