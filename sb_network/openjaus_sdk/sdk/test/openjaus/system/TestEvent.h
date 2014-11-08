/**
\file TestEvent.h

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

#include "openjaus/system/Event.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestEvent : public CxxTest::TestSuite
{
public:
	// Start of user code TestEvent:
	openjaus::system::Event *event;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		event = new openjaus::system::Event();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete event;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(event);
		// End of user code
	}
	
	void testDescriptionAccessors()
	{
		// Start of user code testDescriptionAccessors:
		//std::string description;		
		//event->setDescription(description);
		event->getDescription();
		// End of user code
	}
	
	void testMaxLoggingHzAccessors()
	{
		// Start of user code testMaxLoggingHzAccessors:
		//float maxLoggingHz;		
		//event->setMaxLoggingHz(maxLoggingHz);
		event->getMaxLoggingHz();
		// End of user code
	}
	
	void testSaveAccessors()
	{
		// Start of user code testSaveAccessors:
		//bool save;		
		//event->setSave(save);
		event->isSave();
		// End of user code
	}
	
	void testNameSpaceAccessors()
	{
		// Start of user code testNameSpaceAccessors:
		//openjaus::system::RunLevel level;		
		//event->setLevel(level);
		event->getLevel();
		// End of user code
	}
	
	void testTimeAccessors()
	{
		// Start of user code testTimeAccessors:
		//openjaus::system::Time time;		
		//event->setTime(time);
		event->getTime();
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

