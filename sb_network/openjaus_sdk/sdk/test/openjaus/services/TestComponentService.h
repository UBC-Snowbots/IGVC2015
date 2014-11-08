// File Header Here

#include "openjaus/services/ComponentService.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestComponentService : public CxxTest::TestSuite
{
public:
	// Start of user code TestComponentService:
	openjaus::services::ComponentService *componentService;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		componentService = new openjaus::services::ComponentService();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete componentService;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(componentService);
		// End of user code
	}
	
	void testRunnersAccessors()
	{
		// Start of user code testRunnersAccessors:
		//openjaus::services::StateMachineRunner runners;		
		//componentService->setRunners(runners);
		componentService->getRunners();
		// End of user code
	}
	
	void testInterfacesAccessors()
	{
		// Start of user code testInterfacesAccessors:
		//openjaus::transport::Interface interfaces;		
		//componentService->setInterfaces(interfaces);
		componentService->getInterfaces();
		// End of user code
	}
	
	void testNodeAccessors()
	{
		// Start of user code testNodeAccessors:
		//openjaus::services::Node node;		
		//componentService->setNode(node);
		//componentService->getNode();
		// End of user code
	}
	
	void testSystemTreeAccessors()
	{
		// Start of user code testSystemTreeAccessors:
		//openjaus::services::SystemTree systemTree;		
		//componentService->setSystemTree(systemTree);
		//componentService->getSystemTree();
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
	
	// Start of user code custom tests:
	// End of user code
};

