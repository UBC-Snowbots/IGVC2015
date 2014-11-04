/**
\file TestComponent.h

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

#include "openjaus/model/Component.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestComponent : public CxxTest::TestSuite
{
public:
	// Start of user code TestComponent:
	openjaus::model::Component *component;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		component = new openjaus::model::Component();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete component;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(component);
		// End of user code
	}
	
	void testNameAccessors()
	{
		// Start of user code testNameAccessors:
		//std::string name;		
		//component->setName(name);
		//component->getName();
		// End of user code
	}
	
	void testIdAccessors()
	{
		// Start of user code testIdAccessors:
		//int id;		
		//component->setId(id);
		//component->getId();
		// End of user code
	}
	
	void testServicesAccessors()
	{
		// Start of user code testServicesAccessors:
		//std::map< std::string, openjaus::model::Service * > services;		
		//component->setServices(services);
		//component->getServices();
		// End of user code
	}
	
	void testAuthorityAccessors()
	{
		// Start of user code testAuthorityAccessors:
		//uint8_t authority;		
		//component->setAuthority(authority);
		//component->getAuthority();
		// End of user code
	}
	
	void testAddressAccessors()
	{
		// Start of user code testAddressAccessors:
		//openjaus::transport::Address address;		
		//component->setAddress(address);
		//component->getAddress();
		// End of user code
	}
	
	void testRunnersAccessors()
	{
		// Start of user code testRunnersAccessors:
		//openjaus::model::StateMachineRunner runners;		
		//component->setRunners(runners);
		//component->getRunners();
		// End of user code
	}
	
	void testInterfacesAccessors()
	{
		// Start of user code testInterfacesAccessors:
		//openjaus::transport::Interface interfaces;		
		//component->setInterfaces(interfaces);
		//component->getInterfaces();
		// End of user code
	}
	
	void testNodeAccessors()
	{
		// Start of user code testNodeAccessors:
		//openjaus::model::Node node;		
		//component->setNode(node);
		//component->getNode();
		// End of user code
	}
	
	void testSystemTreeAccessors()
	{
		// Start of user code testSystemTreeAccessors:
		//openjaus::model::SystemTree systemTree;		
		//component->setSystemTree(systemTree);
		//component->getSystemTree();
		// End of user code
	}
	
	void testImplementsAccessors()
	{
		// Start of user code testImplementsAccessors:
		//openjaus::model::Service implements;		
		//component->setImplements(implements);
		//component->getImplements();
		// End of user code
	}
	
	void testInheritsFromAccessors()
	{
		// Start of user code testInheritsFromAccessors:
		//openjaus::model::Component inheritsFrom;		
		//component->setInheritsFrom(inheritsFrom);
		//component->getInheritsFrom();
		// End of user code
	}
	
	void testAddService()
	{
		// Start of user code testAddService:
		// End of user code
	}
	
	void testRun()
	{
		// Start of user code testRun:
		// End of user code
	}
	
	void testStop()
	{
		// Start of user code testStop:
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

