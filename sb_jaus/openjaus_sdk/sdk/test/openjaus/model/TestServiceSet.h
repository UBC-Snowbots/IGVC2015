/**
\file TestServiceSet.h

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

#include "openjaus/model/ServiceSet.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestServiceSet : public CxxTest::TestSuite
{
public:
	// Start of user code TestServiceSet:
	openjaus::model::ServiceSet *serviceSet;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		serviceSet = new openjaus::model::ServiceSet();	
	
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
		//serviceSet->getName();
		// End of user code
	}
	
	void testServicesAccessors()
	{
		// Start of user code testServicesAccessors:
		//openjaus::model::Service services;		
		//serviceSet->setServices(services);
		//serviceSet->getServices();
		// End of user code
	}
	
	void testServiceSetsAccessors()
	{
		// Start of user code testServiceSetsAccessors:
		//openjaus::model::ServiceSet serviceSets;		
		//serviceSet->setServiceSets(serviceSets);
		//serviceSet->getServiceSets();
		// End of user code
	}
	
	void testParentSetAccessors()
	{
		// Start of user code testParentSetAccessors:
		//openjaus::model::ServiceSet parentSet;		
		//serviceSet->setParentSet(parentSet);
		//serviceSet->getParentSet();
		// End of user code
	}
	
	void testDeclaredFieldsAccessors()
	{
		// Start of user code testDeclaredFieldsAccessors:
		//openjaus::model::DeclaredFields declaredFields;		
		//serviceSet->setDeclaredFields(declaredFields);
		//serviceSet->getDeclaredFields();
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

