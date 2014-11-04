/**
\file TestPolicy.h

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

#include "openjaus/transport/Policy.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestPolicy : public CxxTest::TestSuite
{
public:
	// Start of user code TestPolicy:
	openjaus::transport::Policy *policy;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		policy = new openjaus::transport::Policy();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete policy;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(policy);
		// End of user code
	}
	
	void testRequestCountAccessors()
	{
		// Start of user code testRequestCountAccessors:
		//int requestCount;		
		//policy->setRequestCount(requestCount);
		//policy->getRequestCount();
		// End of user code
	}
	
	void testZlibCompressionAccessors()
	{
		// Start of user code testZlibCompressionAccessors:
		//bool zlibCompression;		
		//policy->setZlibCompression(zlibCompression);
		//policy->getZlibCompression();
		// End of user code
	}
	
	void testConfirmedAccessors()
	{
		// Start of user code testConfirmedAccessors:
		//bool confirmed;		
		//policy->setConfirmed(confirmed);
		//policy->getConfirmed();
		// End of user code
	}
	
	void testTCPSupportedAccessors()
	{
		// Start of user code testTCPSupportedAccessors:
		//bool TCPSupported;		
		//policy->setTCPSupported(TCPSupported);
		//policy->getTCPSupported();
		// End of user code
	}
	
	void testOpenJAUSSupportedAccessors()
	{
		// Start of user code testOpenJAUSSupportedAccessors:
		//bool OpenJAUSSupported;		
		//policy->setOpenJAUSSupported(OpenJAUSSupported);
		//policy->getOpenJAUSSupported();
		// End of user code
	}
	
	void testPreferredTCPUseAccessors()
	{
		// Start of user code testPreferredTCPUseAccessors:
		//openjaus::transport::TCPPreference TCPPreference;		
		//policy->setTCPPreference(TCPPreference);
		//policy->getTCPPreference();
		// End of user code
	}
	
	void testTimeoutAccessors()
	{
		// Start of user code testTimeoutAccessors:
		//openjaus::system::Time timeout;		
		//policy->setTimeout(timeout);
		//policy->getTimeout();
		// End of user code
	}
	
	void testSetPolicy()
	{
		// Start of user code testSetPolicy:
		// End of user code
	}
	
	void testIncrementRequestCount()
	{
		// Start of user code testIncrementRequestCount:
		// End of user code
	}
	
	void testResetTimeout()
	{
		// Start of user code testResetTimeout:
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

