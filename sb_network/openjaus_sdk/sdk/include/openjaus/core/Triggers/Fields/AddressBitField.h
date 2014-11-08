/**
\file address.h

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


#ifndef ADDRESS_BITFIELD_H
#define ADDRESS_BITFIELD_H

#include <openjaus.h>

namespace openjaus
{
namespace core
{

class OPENJAUS_EXPORT AddressBitField : public model::fields::BitField
{
public:
	AddressBitField();
	~AddressBitField();



    bool setSubsystem(long value);
    long getSubsystem(void) const;
    bool setNode(long value);
    long getNode(void) const;
    bool setComponent(long value);
    long getComponent(void) const;

	virtual int to(system::Buffer *dst);	
	virtual int from(system::Buffer *src);
	virtual int length();
	void copy(AddressBitField& source);
	std::string toXml(unsigned char level=0) const;
	
protected:
    long subsystem;
    long node;
    long component;

    static const long SUBSYSTEM_START_BIT = 16;
    static const long SUBSYSTEM_BIT_MASK = 0xFFFF;
    
    static const long NODE_START_BIT = 8;
    static const long NODE_BIT_MASK = 0xFF;
    
    static const long COMPONENT_START_BIT = 0;
    static const long COMPONENT_BIT_MASK = 0xFF;
    
    static const long SUBSYSTEM_MIN_VALUE = 0;
    static const long SUBSYSTEM_MAX_VALUE = 65535;
    
    static const long NODE_MIN_VALUE = 0;
    static const long NODE_MAX_VALUE = 255;
    
    static const long COMPONENT_MIN_VALUE = 0;
    static const long COMPONENT_MAX_VALUE = 255;
    
};

} // namespace core
} // namespace openjaus

#endif // ADDRESS_BITFIELD_H

