// File Header Here

#include "openjaus/model/fields/ScaledIntegerUpperLimit.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestScaledIntegerUpperLimit : public CxxTest::TestSuite
{
public:
	// Start of user code TestScaledIntegerUpperLimit:
	openjaus::model::fields::ScaledIntegerUpperLimit *scaledIntegerUpperLimit;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		scaledIntegerUpperLimit = new openjaus::model::fields::ScaledIntegerUpperLimit();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete scaledIntegerUpperLimit;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(scaledIntegerUpperLimit);
		// End of user code
	}
	
	void testValueAccessors()
	{
		// Start of user code testValueAccessors:
		//double value;		
		//scaledIntegerUpperLimit->setValue(value);
		//scaledIntegerUpperLimit->getValue();
		// End of user code
	}
	
	void testLimitAccessors()
	{
		// Start of user code testLimitAccessors:
		//openjaus::model::fields::VariablePoint limit;		
		//scaledIntegerUpperLimit->setLimit(limit);
		scaledIntegerUpperLimit->getLimit();
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

