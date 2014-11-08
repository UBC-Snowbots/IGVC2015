/**
\file FrameSizeRef.h

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

#ifndef FRAMESIZEREFENUMERATION_H
#define FRAMESIZEREFENUMERATION_H

#include <openjaus.h>

namespace openjaus
{
namespace environment
{

class OPENJAUS_EXPORT FrameSizeRefEnumeration : public openjaus::model::fields::Enumeration
{
public:
	FrameSizeRefEnumeration();
	~FrameSizeRefEnumeration();

	enum FrameSizeRefEnum {SQCIF_128X96 = 0, QCIF_176X144 = 1, CIF_352X288 = 2, CIF4_704X576 = 3, CIF16_1408X1152 = 4, QQVGA_160X120 = 5, QVGA_320X240 = 6, VGA_640X480 = 7, SVGA_800X600 = 8, XGA_1024X768 = 9, UXGA_1600X1200 = 10, QXGA_2048X1536 = 11, SXGA_1280X1024 = 12, QSXGA_2560X2048 = 13, HSXGA_5120X4096 = 14, WVGA_852X480 = 15, WXGA_1366X768 = 16, WSXGA_1600X1024 = 17, WUXGA_1920X1200 = 18, WOXGA_2560X1600 = 19, WQSXGA_3200X2048 = 20, WQUXGA_3840X2400 = 21, WHSXGA_6400X4096 = 22, WHUXGA_7680X4800 = 23, CGA_320X200 = 24, EGA_640X350 = 25, HD480_852X480 = 26, HD720_1280X720 = 27, HD1080_1920X1080 = 28};
	
	virtual int to(system::Buffer *dst);
	virtual int from(system::Buffer *src);
	virtual int length(void);
	std::string toXml(unsigned char level=0) const;	
	std::string toString() const;
	
	FrameSizeRefEnum getValue(void) const;
	void setValue(FrameSizeRefEnum value);

protected:
	FrameSizeRefEnum value;
};

} // namespace environment
} // namespace openjaus

#endif // FRAMESIZEREFENUMERATION_H

