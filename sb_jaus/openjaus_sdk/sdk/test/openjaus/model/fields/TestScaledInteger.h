/**
\file TestScaledInteger.h

\par Copyright
Copyright (c) 2012, OpenJAUS, LLC
All rights reserved.

This file is part of the OpenJAUS Software Development Kit (SDK). This 
software is distributed under one of two licenses, the OpenJAUS SDK 
Commercial End User License Agreement or the OpenJAUS SDK Non-Commercial 
End User License Agreement. The appropriate licensing details were included 
in with your developer credentials and software download. See the LICENSE 
file included with this software for full details.
 
THIS SOFTWARE IS PROVIDED BY THE LICENSOR (OPENJAUS LCC) "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE LICENSOR BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THE SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE. THE LICENSOR DOES NOT WARRANT THAT THE LICENSED SOFTWARE WILL MEET
LICENSEE'S REQUIREMENTS OR THAT THE OPERATION OF THE LICENSED SOFTWARE
WILL BE UNINTERRUPTED OR ERROR-FREE, OR THAT ERRORS IN THE LICENSED
SOFTWARE WILL BE CORRECTED.

\ Software History
- [2011-08-23] - Added AS6057: Manipulators
- [2011-08-01] - Added AS6060: Environment Sensing
- [2011-06-16] - First Release 

*/

#include "openjaus/model/fields/ScaledInteger.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestScaledInteger : public CxxTest::TestSuite
{
public:
	// Start of user code TestScaledInteger:
	openjaus::model::fields::ScaledInteger *scaledInteger;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		scaledInteger = new openjaus::model::fields::ScaledInteger();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete scaledInteger;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(scaledInteger);
		// End of user code
	}
	
	void testValueAccessors()
	{
		// Start of user code testValueAccessors:
		//double value;		
		//scaledInteger->setValue(value);
		scaledInteger->getValue();
		// End of user code
	}
	
	void testIntegerTypeAccessors()
	{
		// Start of user code testIntegerTypeAccessors:
		//openjaus::model::fields::FieldTypesUnsigned integerType;		
		//scaledInteger->setIntegerType(integerType);
		scaledInteger->getIntegerType();
		// End of user code
	}
	
	void testRoundingTypeAccessors()
	{
		// Start of user code testRoundingTypeAccessors:
		//openjaus::model::fields::RoundingType roundingType;		
		//scaledInteger->setRoundingType(roundingType);
		scaledInteger->getRoundingType();
		// End of user code
	}
	
	void testDefaultValueAccessors()
	{
		// Start of user code testDefaultValueAccessors:
		//double defaultValue;		
		//scaledInteger->setDefaultValue(defaultValue);
		scaledInteger->getDefaultValue();
		// End of user code
	}
	
	void testUpperLimitAccessors()
	{
		// Start of user code testUpperLimitAccessors:
		//double upperLimit;		
		//scaledInteger->setUpperLimit(upperLimit);
		scaledInteger->getUpperLimit();
		// End of user code
	}
	
	void testLowerLimitAccessors()
	{
		// Start of user code testLowerLimitAccessors:
		//double lowerLimit;		
		//scaledInteger->setLowerLimit(lowerLimit);
		scaledInteger->getLowerLimit();
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

