/**
\file SupportedImageFormats.h

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


#ifndef SUPPORTEDIMAGEFORMATS_BITFIELD_H
#define SUPPORTEDIMAGEFORMATS_BITFIELD_H

#include <openjaus.h>

namespace openjaus
{
namespace environment
{

class OPENJAUS_EXPORT SupportedImageFormatsBitField : public model::fields::BitField
{
public:
	SupportedImageFormatsBitField();
	~SupportedImageFormatsBitField();


    bool setJPEG(bool value);
    bool getJPEG(void) const;
    bool setGIF(bool value);
    bool getGIF(void) const;
    bool setPNG(bool value);
    bool getPNG(void) const;
    bool setBMP(bool value);
    bool getBMP(void) const;
    bool setTIFF(bool value);
    bool getTIFF(void) const;
    bool setPPM(bool value);
    bool getPPM(void) const;
    bool setPGM(bool value);
    bool getPGM(void) const;
    bool setPNM(bool value);
    bool getPNM(void) const;
    bool setNEF_Nikon_RAW(bool value);
    bool getNEF_Nikon_RAW(void) const;
    bool setCR2_Canon_RAW(bool value);
    bool getCR2_Canon_RAW(void) const;
    bool setDNG_Adobe_RAW(bool value);
    bool getDNG_Adobe_RAW(void) const;


	virtual int to(system::Buffer *dst);	
	virtual int from(system::Buffer *src);
	virtual int length();
	void copy(SupportedImageFormatsBitField& source);
	std::string toXml(unsigned char level=0) const;
	
protected:
	bool jPEG;
	bool gIF;
	bool pNG;
	bool bMP;
	bool tIFF;
	bool pPM;
	bool pGM;
	bool pNM;
	bool nEF_Nikon_RAW;
	bool cR2_Canon_RAW;
	bool dNG_Adobe_RAW;

    static const long JPEG_START_BIT = 0;
    static const long JPEG_BIT_MASK = 0x1;
    
    static const long GIF_START_BIT = 1;
    static const long GIF_BIT_MASK = 0x1;
    
    static const long PNG_START_BIT = 2;
    static const long PNG_BIT_MASK = 0x1;
    
    static const long BMP_START_BIT = 3;
    static const long BMP_BIT_MASK = 0x1;
    
    static const long TIFF_START_BIT = 4;
    static const long TIFF_BIT_MASK = 0x1;
    
    static const long PPM_START_BIT = 5;
    static const long PPM_BIT_MASK = 0x1;
    
    static const long PGM_START_BIT = 6;
    static const long PGM_BIT_MASK = 0x1;
    
    static const long PNM_START_BIT = 7;
    static const long PNM_BIT_MASK = 0x1;
    
    static const long NEF_NIKON_RAW_START_BIT = 8;
    static const long NEF_NIKON_RAW_BIT_MASK = 0x1;
    
    static const long CR2_CANON_RAW_START_BIT = 9;
    static const long CR2_CANON_RAW_BIT_MASK = 0x1;
    
    static const long DNG_ADOBE_RAW_START_BIT = 10;
    static const long DNG_ADOBE_RAW_BIT_MASK = 0x1;
    
};

} // namespace environment
} // namespace openjaus

#endif // SUPPORTEDIMAGEFORMATS_BITFIELD_H

