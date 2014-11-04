// File Header Here

#include "openjaus/system/Property.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestProperty : public CxxTest::TestSuite
{
public:
	// Start of user code TestProperty:
	openjaus::system::Property *property;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		property = new openjaus::system::Property();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete property;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(property);
		// End of user code
	}
	
	void testKeyAccessors()
	{
		// Start of user code testKeyAccessors:
		//std::string key;		
		//property->setKey(key);
		property->getKey();
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

