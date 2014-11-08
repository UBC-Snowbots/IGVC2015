// File Header Here

#include "openjaus/model/fields/BitFieldItem.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestBitFieldItem : public CxxTest::TestSuite
{
public:
	// Start of user code TestBitFieldItem:
	openjaus::model::fields::BitFieldItem *bitFieldItem;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		bitFieldItem = new openjaus::model::fields::BitFieldItem();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete bitFieldItem;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(bitFieldItem);
		// End of user code
	}
	
	void testNameAccessors()
	{
		// Start of user code testNameAccessors:
		//std::string name;		
		//bitFieldItem->setName(name);
		bitFieldItem->getName();
		// End of user code
	}
	
	void testInterpretationAccessors()
	{
		// Start of user code testInterpretationAccessors:
		//std::string interpretation;		
		//bitFieldItem->setInterpretation(interpretation);
		bitFieldItem->getInterpretation();
		// End of user code
	}
	
	void testStartIndexAccessors()
	{
		// Start of user code testStartIndexAccessors:
		//long startIndex;		
		//bitFieldItem->setStartIndex(startIndex);
		bitFieldItem->getStartIndex();
		// End of user code
	}
	
	void testEndIndexAccessors()
	{
		// Start of user code testEndIndexAccessors:
		//long endIndex;		
		//bitFieldItem->setEndIndex(endIndex);
		bitFieldItem->getEndIndex();
		// End of user code
	}
	
	void testValuesAccessors()
	{
		// Start of user code testValuesAccessors:
		//openjaus::model::fields::EnumerationItem values;		
		//bitFieldItem->setValues(values);
		bitFieldItem->getValues();
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

