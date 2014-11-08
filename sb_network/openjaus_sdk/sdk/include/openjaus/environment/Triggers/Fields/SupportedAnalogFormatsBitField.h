/**
\file SupportedAnalogFormats.h

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


#ifndef SUPPORTEDANALOGFORMATS_BITFIELD_H
#define SUPPORTEDANALOGFORMATS_BITFIELD_H

#include <openjaus.h>

namespace openjaus
{
namespace environment
{

class OPENJAUS_EXPORT SupportedAnalogFormatsBitField : public model::fields::BitField
{
public:
	SupportedAnalogFormatsBitField();
	~SupportedAnalogFormatsBitField();


    bool setNTSCM(bool value);
    bool getNTSCM(void) const;
    bool setNTSCJ(bool value);
    bool getNTSCJ(void) const;
    bool setPALN(bool value);
    bool getPALN(void) const;
    bool setPALM(bool value);
    bool getPALM(void) const;
    bool setSECAML(bool value);
    bool getSECAML(void) const;
    bool setSECAMBG(bool value);
    bool getSECAMBG(void) const;


	virtual int to(system::Buffer *dst);	
	virtual int from(system::Buffer *src);
	virtual int length();
	void copy(SupportedAnalogFormatsBitField& source);
	std::string toXml(unsigned char level=0) const;
	
protected:
	bool nTSCM;
	bool nTSCJ;
	bool pALN;
	bool pALM;
	bool sECAML;
	bool sECAMBG;

    static const long NTSCM_START_BIT = 0;
    static const long NTSCM_BIT_MASK = 0x1;
    
    static const long NTSCJ_START_BIT = 1;
    static const long NTSCJ_BIT_MASK = 0x1;
    
    static const long PALN_START_BIT = 2;
    static const long PALN_BIT_MASK = 0x1;
    
    static const long PALM_START_BIT = 3;
    static const long PALM_BIT_MASK = 0x1;
    
    static const long SECAML_START_BIT = 4;
    static const long SECAML_BIT_MASK = 0x1;
    
    static const long SECAMBG_START_BIT = 5;
    static const long SECAMBG_BIT_MASK = 0x1;
    
};

} // namespace environment
} // namespace openjaus

#endif // SUPPORTEDANALOGFORMATS_BITFIELD_H

