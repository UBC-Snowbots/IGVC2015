// File Header Here

#include "openjaus/services/DeclaredFields.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestDeclaredFields : public CxxTest::TestSuite
{
public:
	// Start of user code TestDeclaredFields:
	openjaus::services::DeclaredFields *declaredFields;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		declaredFields = new openjaus::services::DeclaredFields();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete declaredFields;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(declaredFields);
		// End of user code
	}
	
	void testFieldsAccessors()
	{
		// Start of user code testFieldsAccessors:
		//openjaus::model::Field fields;		
		//declaredFields->setFields(fields);
		declaredFields->getFields();
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

