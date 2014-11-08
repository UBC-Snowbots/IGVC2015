/**
\file TestApplication.h

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

#include "openjaus/system/Application.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestApplication : public CxxTest::TestSuite
{
public:
	// Start of user code TestApplication:
	openjaus::system::Application *application;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		application = new openjaus::system::Application();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete application;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(application);
		// End of user code
	}
	
	void testSettings()
	{
		// Start of user code testSettings:
		// End of user code
	}
	
	void testProcessId()
	{
		// Start of user code testProcessId:
		// End of user code
	}
	
	void testGetName()
	{
		// Start of user code testGetName:
		// End of user code
	}
	
	void testGetArg()
	{
		// Start of user code testGetArg:
		// End of user code
	}
	
	void testGetArg()
	{
		// Start of user code testGetArg:
		// End of user code
	}
	
	void testIsLogSpaceOn()
	{
		// Start of user code testIsLogSpaceOn:
		// End of user code
	}
	
	void testInstance()
	{
		// Start of user code testInstance:
		// End of user code
	}
	
	void testSetTerminalMode()
	{
		// Start of user code testSetTerminalMode:
		// End of user code
	}
	
	void testGetChar()
	{
		// Start of user code testGetChar:
		// End of user code
	}
	
	void testGetSetting()
	{
		// Start of user code testGetSetting:
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

