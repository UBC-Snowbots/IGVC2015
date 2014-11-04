/**
\file LightSensitivityLevels.h

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


#ifndef LIGHTSENSITIVITYLEVELS_BITFIELD_H
#define LIGHTSENSITIVITYLEVELS_BITFIELD_H

#include <openjaus.h>

namespace openjaus
{
namespace environment
{

class OPENJAUS_EXPORT LightSensitivityLevelsBitField : public model::fields::BitField
{
public:
	LightSensitivityLevelsBitField();
	~LightSensitivityLevelsBitField();


    bool setAutoISO(bool value);
    bool getAutoISO(void) const;
    bool setISO100(bool value);
    bool getISO100(void) const;
    bool setISO200(bool value);
    bool getISO200(void) const;
    bool setISO400(bool value);
    bool getISO400(void) const;
    bool setISO800(bool value);
    bool getISO800(void) const;
    bool setISO1600(bool value);
    bool getISO1600(void) const;
    bool setISO3200(bool value);
    bool getISO3200(void) const;


	virtual int to(system::Buffer *dst);	
	virtual int from(system::Buffer *src);
	virtual int length();
	void copy(LightSensitivityLevelsBitField& source);
	std::string toXml(unsigned char level=0) const;
	
protected:
	bool autoISO;
	bool iSO100;
	bool iSO200;
	bool iSO400;
	bool iSO800;
	bool iSO1600;
	bool iSO3200;

    static const long AUTOISO_START_BIT = 0;
    static const long AUTOISO_BIT_MASK = 0x1;
    
    static const long ISO100_START_BIT = 1;
    static const long ISO100_BIT_MASK = 0x1;
    
    static const long ISO200_START_BIT = 2;
    static const long ISO200_BIT_MASK = 0x1;
    
    static const long ISO400_START_BIT = 3;
    static const long ISO400_BIT_MASK = 0x1;
    
    static const long ISO800_START_BIT = 4;
    static const long ISO800_BIT_MASK = 0x1;
    
    static const long ISO1600_START_BIT = 5;
    static const long ISO1600_BIT_MASK = 0x1;
    
    static const long ISO3200_START_BIT = 6;
    static const long ISO3200_BIT_MASK = 0x1;
    
};

} // namespace environment
} // namespace openjaus

#endif // LIGHTSENSITIVITYLEVELS_BITFIELD_H

