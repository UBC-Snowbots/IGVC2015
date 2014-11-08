/**
\file TestService.h

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

#include "openjaus/model/Service.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestService : public CxxTest::TestSuite
{
public:
	// Start of user code TestService:
	openjaus::model::Service *service;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		service = new openjaus::model::Service();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete service;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(service);
		// End of user code
	}
	
	void testNameAccessors()
	{
		// Start of user code testNameAccessors:
		//std::string name;		
		//service->setName(name);
		//service->getName();
		// End of user code
	}
	
	void testUriAccessors()
	{
		// Start of user code testUriAccessors:
		//std::string uri;		
		//service->setUri(uri);
		//service->getUri();
		// End of user code
	}
	
	void testDescriptionAccessors()
	{
		// Start of user code testDescriptionAccessors:
		//std::string description;		
		//service->setDescription(description);
		//service->getDescription();
		// End of user code
	}
	
	void testVersionMajorAccessors()
	{
		// Start of user code testVersionMajorAccessors:
		//int versionMajor;		
		//service->setVersionMajor(versionMajor);
		//service->getVersionMajor();
		// End of user code
	}
	
	void testVersionMinorAccessors()
	{
		// Start of user code testVersionMinorAccessors:
		//int versionMinor;		
		//service->setVersionMinor(versionMinor);
		//service->getVersionMinor();
		// End of user code
	}
	
	void testInheritsFromAccessors()
	{
		// Start of user code testInheritsFromAccessors:
		//openjaus::model::Service inheritsFrom;		
		//service->setInheritsFrom(inheritsFrom);
		//service->getInheritsFrom();
		// End of user code
	}
	
	void testTriggersAccessors()
	{
		// Start of user code testTriggersAccessors:
		//openjaus::model::Trigger triggers;		
		//service->setTriggers(triggers);
		//service->getTriggers();
		// End of user code
	}
	
	void testStateMachinesAccessors()
	{
		// Start of user code testStateMachinesAccessors:
		//openjaus::model::StateMachine stateMachines;		
		//service->setStateMachines(stateMachines);
		//service->getStateMachines();
		// End of user code
	}
	
	void testActionsAccessors()
	{
		// Start of user code testActionsAccessors:
		//openjaus::model::Action actions;		
		//service->setActions(actions);
		//service->getActions();
		// End of user code
	}
	
	void testConditionsAccessors()
	{
		// Start of user code testConditionsAccessors:
		//openjaus::model::Condition conditions;		
		//service->setConditions(conditions);
		//service->getConditions();
		// End of user code
	}
	
	void testTransitionsAccessors()
	{
		// Start of user code testTransitionsAccessors:
		//openjaus::model::TransitionExtension transitions;		
		//service->setTransitions(transitions);
		//service->getTransitions();
		// End of user code
	}
	
	void testStatesAccessors()
	{
		// Start of user code testStatesAccessors:
		//openjaus::model::StateExtension states;		
		//service->setStates(states);
		//service->getStates();
		// End of user code
	}
	
	void testGetStateMachine()
	{
		// Start of user code testGetStateMachine:
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

