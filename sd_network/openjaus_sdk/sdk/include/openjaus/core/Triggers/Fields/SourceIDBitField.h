/**
\file SourceID.h

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


#ifndef SOURCEID_BITFIELD_H
#define SOURCEID_BITFIELD_H

#include <openjaus.h>

namespace openjaus
{
namespace core
{

class OPENJAUS_EXPORT SourceIDBitField : public model::fields::BitField
{
public:
	SourceIDBitField();
	~SourceIDBitField();



    bool setComponentID(long value);
    long getComponentID(void) const;
    bool setNodeID(long value);
    long getNodeID(void) const;
    bool setSubsystemID(long value);
    long getSubsystemID(void) const;

	virtual int to(system::Buffer *dst);	
	virtual int from(system::Buffer *src);
	virtual int length();
	void copy(SourceIDBitField& source);
	std::string toXml(unsigned char level=0) const;
	
protected:
    long componentID;
    long nodeID;
    long subsystemID;

    static const long COMPONENTID_START_BIT = 0;
    static const long COMPONENTID_BIT_MASK = 0xFF;
    
    static const long NODEID_START_BIT = 8;
    static const long NODEID_BIT_MASK = 0xFF;
    
    static const long SUBSYSTEMID_START_BIT = 16;
    static const long SUBSYSTEMID_BIT_MASK = 0xFFFF;
    
    static const long COMPONENTID_MIN_VALUE = 0;
    static const long COMPONENTID_MAX_VALUE = 0;
    
    static const long NODEID_MIN_VALUE = 0;
    static const long NODEID_MAX_VALUE = 0;
    
    static const long SUBSYSTEMID_MIN_VALUE = 0;
    static const long SUBSYSTEMID_MAX_VALUE = 0;
    
};

} // namespace core
} // namespace openjaus

#endif // SOURCEID_BITFIELD_H

