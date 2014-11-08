/**
\file SupportedFrameSizes.h

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


#ifndef SUPPORTEDFRAMESIZES_BITFIELD_H
#define SUPPORTEDFRAMESIZES_BITFIELD_H

#include <openjaus.h>

namespace openjaus
{
namespace environment
{

class OPENJAUS_EXPORT SupportedFrameSizesBitField : public model::fields::BitField
{
public:
	SupportedFrameSizesBitField();
	~SupportedFrameSizesBitField();


    bool setSqcif_128x96(bool value);
    bool getSqcif_128x96(void) const;
    bool setQcif_176x144(bool value);
    bool getQcif_176x144(void) const;
    bool setCif_352x288(bool value);
    bool getCif_352x288(void) const;
    bool setCif4_704x576(bool value);
    bool getCif4_704x576(void) const;
    bool setCif16_1408x1152(bool value);
    bool getCif16_1408x1152(void) const;
    bool setQqvga_160x120(bool value);
    bool getQqvga_160x120(void) const;
    bool setQvga_320x240(bool value);
    bool getQvga_320x240(void) const;
    bool setVga_640x480(bool value);
    bool getVga_640x480(void) const;
    bool setSvga_800x600(bool value);
    bool getSvga_800x600(void) const;
    bool setXga_1024x768(bool value);
    bool getXga_1024x768(void) const;
    bool setUxga_1600x1200(bool value);
    bool getUxga_1600x1200(void) const;
    bool setQxga_2048x1536(bool value);
    bool getQxga_2048x1536(void) const;
    bool setSxga_1280x1024(bool value);
    bool getSxga_1280x1024(void) const;
    bool setQsxga_2560x2048(bool value);
    bool getQsxga_2560x2048(void) const;
    bool setHsxga_5120x4096(bool value);
    bool getHsxga_5120x4096(void) const;
    bool setWvga_852x480(bool value);
    bool getWvga_852x480(void) const;
    bool setWxga_1366x768(bool value);
    bool getWxga_1366x768(void) const;
    bool setWsxga_1600x1024(bool value);
    bool getWsxga_1600x1024(void) const;
    bool setWuxga_1920x1200(bool value);
    bool getWuxga_1920x1200(void) const;
    bool setWoxga_2560x1600(bool value);
    bool getWoxga_2560x1600(void) const;
    bool setWqsxga_3200x2048(bool value);
    bool getWqsxga_3200x2048(void) const;
    bool setWquxga_3840x2400(bool value);
    bool getWquxga_3840x2400(void) const;
    bool setWhsxga_6400x4096(bool value);
    bool getWhsxga_6400x4096(void) const;
    bool setWhuxga_7680x4800(bool value);
    bool getWhuxga_7680x4800(void) const;
    bool setCga_320x200(bool value);
    bool getCga_320x200(void) const;
    bool setEga_640x350(bool value);
    bool getEga_640x350(void) const;
    bool setHd480_852x480(bool value);
    bool getHd480_852x480(void) const;
    bool setHd720_1280x720(bool value);
    bool getHd720_1280x720(void) const;
    bool setHd1080_1920x1080(bool value);
    bool getHd1080_1920x1080(void) const;


	virtual int to(system::Buffer *dst);	
	virtual int from(system::Buffer *src);
	virtual int length();
	void copy(SupportedFrameSizesBitField& source);
	std::string toXml(unsigned char level=0) const;
	
protected:
	bool sqcif_128x96;
	bool qcif_176x144;
	bool cif_352x288;
	bool cif4_704x576;
	bool cif16_1408x1152;
	bool qqvga_160x120;
	bool qvga_320x240;
	bool vga_640x480;
	bool svga_800x600;
	bool xga_1024x768;
	bool uxga_1600x1200;
	bool qxga_2048x1536;
	bool sxga_1280x1024;
	bool qsxga_2560x2048;
	bool hsxga_5120x4096;
	bool wvga_852x480;
	bool wxga_1366x768;
	bool wsxga_1600x1024;
	bool wuxga_1920x1200;
	bool woxga_2560x1600;
	bool wqsxga_3200x2048;
	bool wquxga_3840x2400;
	bool whsxga_6400x4096;
	bool whuxga_7680x4800;
	bool cga_320x200;
	bool ega_640x350;
	bool hd480_852x480;
	bool hd720_1280x720;
	bool hd1080_1920x1080;

    static const long SQCIF_128X96_START_BIT = 0;
    static const long SQCIF_128X96_BIT_MASK = 0x1;
    
    static const long QCIF_176X144_START_BIT = 1;
    static const long QCIF_176X144_BIT_MASK = 0x1;
    
    static const long CIF_352X288_START_BIT = 2;
    static const long CIF_352X288_BIT_MASK = 0x1;
    
    static const long CIF4_704X576_START_BIT = 3;
    static const long CIF4_704X576_BIT_MASK = 0x1;
    
    static const long CIF16_1408X1152_START_BIT = 4;
    static const long CIF16_1408X1152_BIT_MASK = 0x1;
    
    static const long QQVGA_160X120_START_BIT = 5;
    static const long QQVGA_160X120_BIT_MASK = 0x1;
    
    static const long QVGA_320X240_START_BIT = 6;
    static const long QVGA_320X240_BIT_MASK = 0x1;
    
    static const long VGA_640X480_START_BIT = 7;
    static const long VGA_640X480_BIT_MASK = 0x1;
    
    static const long SVGA_800X600_START_BIT = 8;
    static const long SVGA_800X600_BIT_MASK = 0x1;
    
    static const long XGA_1024X768_START_BIT = 9;
    static const long XGA_1024X768_BIT_MASK = 0x1;
    
    static const long UXGA_1600X1200_START_BIT = 10;
    static const long UXGA_1600X1200_BIT_MASK = 0x1;
    
    static const long QXGA_2048X1536_START_BIT = 11;
    static const long QXGA_2048X1536_BIT_MASK = 0x1;
    
    static const long SXGA_1280X1024_START_BIT = 12;
    static const long SXGA_1280X1024_BIT_MASK = 0x1;
    
    static const long QSXGA_2560X2048_START_BIT = 13;
    static const long QSXGA_2560X2048_BIT_MASK = 0x1;
    
    static const long HSXGA_5120X4096_START_BIT = 14;
    static const long HSXGA_5120X4096_BIT_MASK = 0x1;
    
    static const long WVGA_852X480_START_BIT = 15;
    static const long WVGA_852X480_BIT_MASK = 0x1;
    
    static const long WXGA_1366X768_START_BIT = 16;
    static const long WXGA_1366X768_BIT_MASK = 0x1;
    
    static const long WSXGA_1600X1024_START_BIT = 17;
    static const long WSXGA_1600X1024_BIT_MASK = 0x1;
    
    static const long WUXGA_1920X1200_START_BIT = 18;
    static const long WUXGA_1920X1200_BIT_MASK = 0x1;
    
    static const long WOXGA_2560X1600_START_BIT = 19;
    static const long WOXGA_2560X1600_BIT_MASK = 0x1;
    
    static const long WQSXGA_3200X2048_START_BIT = 20;
    static const long WQSXGA_3200X2048_BIT_MASK = 0x1;
    
    static const long WQUXGA_3840X2400_START_BIT = 21;
    static const long WQUXGA_3840X2400_BIT_MASK = 0x1;
    
    static const long WHSXGA_6400X4096_START_BIT = 22;
    static const long WHSXGA_6400X4096_BIT_MASK = 0x1;
    
    static const long WHUXGA_7680X4800_START_BIT = 23;
    static const long WHUXGA_7680X4800_BIT_MASK = 0x1;
    
    static const long CGA_320X200_START_BIT = 24;
    static const long CGA_320X200_BIT_MASK = 0x1;
    
    static const long EGA_640X350_START_BIT = 25;
    static const long EGA_640X350_BIT_MASK = 0x1;
    
    static const long HD480_852X480_START_BIT = 26;
    static const long HD480_852X480_BIT_MASK = 0x1;
    
    static const long HD720_1280X720_START_BIT = 27;
    static const long HD720_1280X720_BIT_MASK = 0x1;
    
    static const long HD1080_1920X1080_START_BIT = 28;
    static const long HD1080_1920X1080_BIT_MASK = 0x1;
    
};

} // namespace environment
} // namespace openjaus

#endif // SUPPORTEDFRAMESIZES_BITFIELD_H

