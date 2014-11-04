// File Header Here

#include "openjaus/model/fields/ScaledIntegerLowerLimit.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestScaledIntegerLowerLimit : public CxxTest::TestSuite
{
public:
	// Start of user code TestScaledIntegerLowerLimit:
	openjaus::model::fields::ScaledIntegerLowerLimit *scaledIntegerLowerLimit;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		scaledIntegerLowerLimit = new openjaus::model::fields::ScaledIntegerLowerLimit();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete scaledIntegerLowerLimit;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(scaledIntegerLowerLimit);
		// End of user code
	}
	
	void testValueAccessors()
	{
		// Start of user code testValueAccessors:
		//double value;		
		//scaledIntegerLowerLimit->setValue(value);
		//scaledIntegerLowerLimit->getValue();
		// End of user code
	}
	
	void testLimitAccessors()
	{
		// Start of user code testLimitAccessors:
		//openjaus::model::fields::VariablePoint limit;		
		//scaledIntegerLowerLimit->setLimit(limit);
		scaledIntegerLowerLimit->getLimit();
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

