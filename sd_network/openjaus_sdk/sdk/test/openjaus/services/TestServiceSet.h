// File Header Here

#include "openjaus/services/ServiceSet.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestServiceSet : public CxxTest::TestSuite
{
public:
	// Start of user code TestServiceSet:
	openjaus::services::ServiceSet *serviceSet;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		serviceSet = new openjaus::services::ServiceSet();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete serviceSet;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(serviceSet);
		// End of user code
	}
	
	void testNameAccessors()
	{
		// Start of user code testNameAccessors:
		//std::string name;		
		//serviceSet->setName(name);
		serviceSet->getName();
		// End of user code
	}
	
	void testServicesAccessors()
	{
		// Start of user code testServicesAccessors:
		//openjaus::services::ComponentService services;		
		//serviceSet->setServices(services);
		serviceSet->getServices();
		// End of user code
	}
	
	void testServiceSetsAccessors()
	{
		// Start of user code testServiceSetsAccessors:
		//openjaus::services::ServiceSet serviceSets;		
		//serviceSet->setServiceSets(serviceSets);
		serviceSet->getServiceSets();
		// End of user code
	}
	
	void testParentSetAccessors()
	{
		// Start of user code testParentSetAccessors:
		//openjaus::services::ServiceSet parentSet;		
		//serviceSet->setParentSet(parentSet);
		serviceSet->getParentSet();
		// End of user code
	}
	
	void testDeclaredFieldsAccessors()
	{
		// Start of user code testDeclaredFieldsAccessors:
		//openjaus::services::DeclaredFields declaredFields;		
		//serviceSet->setDeclaredFields(declaredFields);
		serviceSet->getDeclaredFields();
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

