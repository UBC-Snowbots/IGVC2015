// File Header Here

#include "openjaus/services/Node.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestNode : public CxxTest::TestSuite
{
public:
	// Start of user code TestNode:
	openjaus::services::Node *node;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		node = new openjaus::services::Node();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete node;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(node);
		// End of user code
	}
	
	void testNameAccessors()
	{
		// Start of user code testNameAccessors:
		//std::string name;		
		//node->setName(name);
		//node->getName();
		// End of user code
	}
	
	void testIdAccessors()
	{
		// Start of user code testIdAccessors:
		//int id;		
		//node->setId(id);
		//node->getId();
		// End of user code
	}
	
	void testComponentsAccessors()
	{
		// Start of user code testComponentsAccessors:
		//openjaus::services::ComponentService components;		
		//node->setComponents(components);
		//node->getComponents();
		// End of user code
	}
	
	void testSubsystemAccessors()
	{
		// Start of user code testSubsystemAccessors:
		//openjaus::services::Subsystem subsystem;		
		//node->setSubsystem(subsystem);
		//node->getSubsystem();
		// End of user code
	}
	
	void testAddComponent()
	{
		// Start of user code testAddComponent:
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

