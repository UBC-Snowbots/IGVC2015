/**
\file TestSystemTree.h

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

#include "openjaus/model/SystemTree.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestSystemTree : public CxxTest::TestSuite
{
public:
	// Start of user code TestSystemTree:
	openjaus::model::SystemTree *systemTree;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		systemTree = new openjaus::model::SystemTree();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete systemTree;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(systemTree);
		// End of user code
	}
	
	void testSubsystemsAccessors()
	{
		// Start of user code testSubsystemsAccessors:
		//std::map< short, openjaus::model::Subsystem * > subsystems;		
		//systemTree->setSubsystems(subsystems);
		//systemTree->getSubsystems();
		// End of user code
	}
	
	void testThisSubsytemAccessors()
	{
		// Start of user code testThisSubsytemAccessors:
		//uint16_t thisSubsytem;		
		//systemTree->setThisSubsytem(thisSubsytem);
		//systemTree->getThisSubsytem();
		// End of user code
	}
	
	void testThisNodeAccessors()
	{
		// Start of user code testThisNodeAccessors:
		//uint8_t thisNode;		
		//systemTree->setThisNode(thisNode);
		//systemTree->getThisNode();
		// End of user code
	}
	
	void testMutexAccessors()
	{
		// Start of user code testMutexAccessors:
		//openjaus::system::Mutex mutex;		
		//systemTree->setMutex(mutex);
		//systemTree->getMutex();
		// End of user code
	}
	
	void testHasComponent()
	{
		// Start of user code testHasComponent:
		// End of user code
	}
	
	void testHasComponent()
	{
		// Start of user code testHasComponent:
		// End of user code
	}
	
	void testHasNode()
	{
		// Start of user code testHasNode:
		// End of user code
	}
	
	void testHasNode()
	{
		// Start of user code testHasNode:
		// End of user code
	}
	
	void testHasSubsystem()
	{
		// Start of user code testHasSubsystem:
		// End of user code
	}
	
	void testHasSubsystem()
	{
		// Start of user code testHasSubsystem:
		// End of user code
	}
	
	void testHasComponentIdentification()
	{
		// Start of user code testHasComponentIdentification:
		// End of user code
	}
	
	void testHasComponentIdentification()
	{
		// Start of user code testHasComponentIdentification:
		// End of user code
	}
	
	void testHasComponentIdentification()
	{
		// Start of user code testHasComponentIdentification:
		// End of user code
	}
	
	void testHasNodeIdentification()
	{
		// Start of user code testHasNodeIdentification:
		// End of user code
	}
	
	void testHasNodeIdentification()
	{
		// Start of user code testHasNodeIdentification:
		// End of user code
	}
	
	void testHasNodeIdentification()
	{
		// Start of user code testHasNodeIdentification:
		// End of user code
	}
	
	void testHasSubsystemIdentification()
	{
		// Start of user code testHasSubsystemIdentification:
		// End of user code
	}
	
	void testHasSubsystemIdentification()
	{
		// Start of user code testHasSubsystemIdentification:
		// End of user code
	}
	
	void testHasSubsystemIdentification()
	{
		// Start of user code testHasSubsystemIdentification:
		// End of user code
	}
	
	void testHasComponentConfiguration()
	{
		// Start of user code testHasComponentConfiguration:
		// End of user code
	}
	
	void testHasComponentConfiguration()
	{
		// Start of user code testHasComponentConfiguration:
		// End of user code
	}
	
	void testHasComponentConfiguration()
	{
		// Start of user code testHasComponentConfiguration:
		// End of user code
	}
	
	void testHasNodeConfiguration()
	{
		// Start of user code testHasNodeConfiguration:
		// End of user code
	}
	
	void testHasNodeConfiguration()
	{
		// Start of user code testHasNodeConfiguration:
		// End of user code
	}
	
	void testHasNodeConfiguration()
	{
		// Start of user code testHasNodeConfiguration:
		// End of user code
	}
	
	void testHasSubsystemConfiguration()
	{
		// Start of user code testHasSubsystemConfiguration:
		// End of user code
	}
	
	void testHasSubsystemConfiguration()
	{
		// Start of user code testHasSubsystemConfiguration:
		// End of user code
	}
	
	void testHasSubsystemConfiguration()
	{
		// Start of user code testHasSubsystemConfiguration:
		// End of user code
	}
	
	void testGetComponent()
	{
		// Start of user code testGetComponent:
		// End of user code
	}
	
	void testGetComponent()
	{
		// Start of user code testGetComponent:
		// End of user code
	}
	
	void testGetComponent()
	{
		// Start of user code testGetComponent:
		// End of user code
	}
	
	void testGetNode()
	{
		// Start of user code testGetNode:
		// End of user code
	}
	
	void testGetNode()
	{
		// Start of user code testGetNode:
		// End of user code
	}
	
	void testGetNode()
	{
		// Start of user code testGetNode:
		// End of user code
	}
	
	void testGetSubsystem()
	{
		// Start of user code testGetSubsystem:
		// End of user code
	}
	
	void testGetSubsystem()
	{
		// Start of user code testGetSubsystem:
		// End of user code
	}
	
	void testGetSubsystem()
	{
		// Start of user code testGetSubsystem:
		// End of user code
	}
	
	void testAddService()
	{
		// Start of user code testAddService:
		// End of user code
	}
	
	void testAddComponent()
	{
		// Start of user code testAddComponent:
		// End of user code
	}
	
	void testAddComponent()
	{
		// Start of user code testAddComponent:
		// End of user code
	}
	
	void testAddComponent()
	{
		// Start of user code testAddComponent:
		// End of user code
	}
	
	void testAddNode()
	{
		// Start of user code testAddNode:
		// End of user code
	}
	
	void testAddNode()
	{
		// Start of user code testAddNode:
		// End of user code
	}
	
	void testAddNode()
	{
		// Start of user code testAddNode:
		// End of user code
	}
	
	void testAddSubsystem()
	{
		// Start of user code testAddSubsystem:
		// End of user code
	}
	
	void testAddSubsystem()
	{
		// Start of user code testAddSubsystem:
		// End of user code
	}
	
	void testAddSubsystem()
	{
		// Start of user code testAddSubsystem:
		// End of user code
	}
	
	void testReplaceComponent()
	{
		// Start of user code testReplaceComponent:
		// End of user code
	}
	
	void testRemoveComponent()
	{
		// Start of user code testRemoveComponent:
		// End of user code
	}
	
	void testRemoveComponent()
	{
		// Start of user code testRemoveComponent:
		// End of user code
	}
	
	void testRemoveComponent()
	{
		// Start of user code testRemoveComponent:
		// End of user code
	}
	
	void testRemoveNode()
	{
		// Start of user code testRemoveNode:
		// End of user code
	}
	
	void testRemoveNode()
	{
		// Start of user code testRemoveNode:
		// End of user code
	}
	
	void testRemoveNode()
	{
		// Start of user code testRemoveNode:
		// End of user code
	}
	
	void testRemoveSubsystem()
	{
		// Start of user code testRemoveSubsystem:
		// End of user code
	}
	
	void testRemoveSubsystem()
	{
		// Start of user code testRemoveSubsystem:
		// End of user code
	}
	
	void testRemoveSubsystem()
	{
		// Start of user code testRemoveSubsystem:
		// End of user code
	}
	
	void testLock()
	{
		// Start of user code testLock:
		// End of user code
	}
	
	void testUnlock()
	{
		// Start of user code testUnlock:
		// End of user code
	}
	
	void testGetAvailableComponentId()
	{
		// Start of user code testGetAvailableComponentId:
		// End of user code
	}
	
	void testGetAvailableNodeId()
	{
		// Start of user code testGetAvailableNodeId:
		// End of user code
	}
	
	void testGetAvailableSubsystemId()
	{
		// Start of user code testGetAvailableSubsystemId:
		// End of user code
	}
	
	void testSetComponentIdentification()
	{
		// Start of user code testSetComponentIdentification:
		// End of user code
	}
	
	void testSetNodeIdentification()
	{
		// Start of user code testSetNodeIdentification:
		// End of user code
	}
	
	void testSetSubsystemIdentification()
	{
		// Start of user code testSetSubsystemIdentification:
		// End of user code
	}
	
	void testInstance()
	{
		// Start of user code testInstance:
		// End of user code
	}
	
	void testAddLocalComponent()
	{
		// Start of user code testAddLocalComponent:
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

