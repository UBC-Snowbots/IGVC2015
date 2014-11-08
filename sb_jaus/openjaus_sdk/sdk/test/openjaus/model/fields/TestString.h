// File Header Here

#include "openjaus/model/fields/String.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestString : public CxxTest::TestSuite
{
public:
	// Start of user code TestString:
	openjaus::model::fields::String *string;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		string = new openjaus::model::fields::String();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete string;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(string);
		// End of user code
	}
	
	void testValueAccessors()
	{
		// Start of user code testValueAccessors:
		//std::string value;		
		//string->setValue(value);
		string->getValue();
		// End of user code
	}
	
	void testDefaultValueAccessors()
	{
		// Start of user code testDefaultValueAccessors:
		//std::string defaultValue;		
		//string->setDefaultValue(defaultValue);
		string->getDefaultValue();
		// End of user code
	}
	
	void testTypeAccessors()
	{
		// Start of user code testTypeAccessors:
		//long maxLength;		
		//string->setMaxLength(maxLength);
		string->getMaxLength();
		// End of user code
	}
	
	void testSerialize()
	{
		// Start of user code testSerialize:
		// End of user code
	}
	
	void testDeserialize()
	{
		// Start of user code testDeserialize:
		// End of user code
	}
	
	void testLength()
	{
		// Start of user code testLength:
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

