// File Header Here

#include "openjaus/services/SystemTree.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestSystemTree : public CxxTest::TestSuite
{
public:
	// Start of user code TestSystemTree:
	openjaus::services::SystemTree *systemTree;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		systemTree = new openjaus::services::SystemTree();	
	
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
		//openjaus::services::Subsystem subsystems;		
		//systemTree->setSubsystems(subsystems);
		//systemTree->getSubsystems();
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
	
	void testThisAddressAccessors()
	{
		// Start of user code testThisAddressAccessors:
		//openjaus::transport::Address thisAddress;		
		//systemTree->setThisAddress(thisAddress);
		//systemTree->getThisAddress();
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
	
	// Start of user code custom tests:
	// End of user code
};

