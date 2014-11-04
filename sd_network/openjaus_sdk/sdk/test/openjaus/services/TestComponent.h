// File Header Here

#include "openjaus/services/Component.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestComponent : public CxxTest::TestSuite
{
public:
	// Start of user code TestComponent:
	openjaus::services::Component *component;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		component = new openjaus::services::Component();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete component;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(component);
		// End of user code
	}
	
	void testNameAccessors()
	{
		// Start of user code testNameAccessors:
		//std::string name;		
		//component->setName(name);
		//component->getName();
		// End of user code
	}
	
	void testIdAccessors()
	{
		// Start of user code testIdAccessors:
		//int id;		
		//component->setId(id);
		//component->getId();
		// End of user code
	}
	
	void testServicesAccessors()
	{
		// Start of user code testServicesAccessors:
		//std::map< std::string, openjaus::model::Service * > services;		
		//component->setServices(services);
		//component->getServices();
		// End of user code
	}
	
	void testNodeAccessors()
	{
		// Start of user code testNodeAccessors:
		//openjaus::services::Node node;		
		//component->setNode(node);
		//component->getNode();
		// End of user code
	}
	
	void testAddressAccessors()
	{
		// Start of user code testAddressAccessors:
		//openjaus::transport::Address address;		
		//component->setAddress(address);
		//component->getAddress();
		// End of user code
	}
	
	void testAddService()
	{
		// Start of user code testAddService:
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

