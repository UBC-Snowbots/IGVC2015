/**
\file TestBuffer.h

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

#include "openjaus/system/Buffer.h"
#include <cxxtest/TestSuite.h>
#include <stdio.h>
// Start of user code header files:
// End of user code

class TestBuffer : public CxxTest::TestSuite
{
public:
	// Start of user code TestBuffer:
	openjaus::system::Buffer *buffer;
	// End of user code
	
	void setUp()
	{
		// Start of user code test setup:
		buffer = new openjaus::system::Buffer();	
	
		// End of user code
	}

	void tearDown()
	{
		// Start of user code test tear down:
		
		delete buffer;
		// End of user code
	}

	void testConstruction()
	{
		// Start of user code testConstruction:
		TS_ASSERT(buffer);
		// End of user code
	}
	
	void testPointerAccessors()
	{
		// Start of user code testPointerAccessors:
		//unsigned char * pointer;		
		//buffer->setPointer(pointer);
		buffer->getPointer();
		// End of user code
	}
	
	void testMaxSizeAccessors()
	{
		// Start of user code testMaxSizeAccessors:
		//int maxSize;		
		//buffer->setMaxSize(maxSize);
		buffer->getMaxSize();
		// End of user code
	}
	
	void testBufferAccessors()
	{
		// Start of user code testBufferAccessors:
		//unsigned char * buffer;		
		//buffer->setBuffer(buffer);
		//buffer->getBuffer();
		// End of user code
	}
	
	void testAppend()
	{
		// Start of user code testAppend:
		// End of user code
	}
	
	void testFree()
	{
		// Start of user code testFree:
		// End of user code
	}
	
	void testIncrement()
	{
		// Start of user code testIncrement:
		// End of user code
	}
	
	void testRemainingBytes()
	{
		// Start of user code testRemainingBytes:
		// End of user code
	}
	
	void testContainedBytes()
	{
		// Start of user code testContainedBytes:
		// End of user code
	}
	
	void testClear()
	{
		// Start of user code testClear:
		// End of user code
	}
	
	void testReset()
	{
		// Start of user code testReset:
		// End of user code
	}
	
	void testTo()
	{
		// Start of user code testTo:
		// End of user code
	}
	
	void testFrom()
	{
		// Start of user code testFrom:
		// End of user code
	}
	
	void testSetAllTo()
	{
		// Start of user code testSetAllTo:
		// End of user code
	}
	
	void testSet()
	{
		// Start of user code testSet:
		// End of user code
	}
	
	// Start of user code custom tests:
	// End of user code
};

