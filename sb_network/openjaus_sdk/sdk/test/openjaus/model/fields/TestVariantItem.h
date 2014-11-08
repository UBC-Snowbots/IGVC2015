// File Header Here

#include "openjaus/model/fields/VariantItem.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestVariantItem : public CxxTest::TestSuite
{
public:
	// Start of user code TestVariantItem:
	openjaus::model::fields::VariantItem *variantItem;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		variantItem = new openjaus::model::fields::VariantItem();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete variantItem;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(variantItem);
		// End of user code
	}
	
	void testIdAccessors()
	{
		// Start of user code testIdAccessors:
		//long index;		
		//variantItem->setIndex(index);
		variantItem->getId();
		// End of user code
	}
	
	void testTypeAccessors()
	{
		// Start of user code testTypeAccessors:
		//openjaus::model::Field type;		
		//variantItem->setType(type);
		variantItem->getType();
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

