// File Header Here

#include "openjaus/services/Subsystem.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestSubsystem : public CxxTest::TestSuite
{
public:
	// Start of user code TestSubsystem:
	openjaus::services::Subsystem *subsystem;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		subsystem = new openjaus::services::Subsystem();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete subsystem;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(subsystem);
		// End of user code
	}
	
	void testNameAccessors()
	{
		// Start of user code testNameAccessors:
		//std::string name;		
		//subsystem->setName(name);
		//subsystem->getName();
		// End of user code
	}
	
	void testIdAccessors()
	{
		// Start of user code testIdAccessors:
		//int id;		
		//subsystem->setId(id);
		//subsystem->getId();
		// End of user code
	}
	
	void testNodesAccessors()
	{
		// Start of user code testNodesAccessors:
		//openjaus::services::Node nodes;		
		//subsystem->setNodes(nodes);
		//subsystem->getNodes();
		// End of user code
	}
	
	void testAddNode()
	{
		// Start of user code testAddNode:
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

