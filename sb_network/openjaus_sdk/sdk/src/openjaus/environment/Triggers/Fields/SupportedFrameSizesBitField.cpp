

/**
\file SupportedFrameSizesBitField.h

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

#include <openjaus.h>
#include "openjaus/environment/Triggers/Fields/SupportedFrameSizesBitField.h"

namespace openjaus
{
namespace environment
{

SupportedFrameSizesBitField::SupportedFrameSizesBitField() : 
	sqcif_128x96(),
	qcif_176x144(),
	cif_352x288(),
	cif4_704x576(),
	cif16_1408x1152(),
	qqvga_160x120(),
	qvga_320x240(),
	vga_640x480(),
	svga_800x600(),
	xga_1024x768(),
	uxga_1600x1200(),
	qxga_2048x1536(),
	sxga_1280x1024(),
	qsxga_2560x2048(),
	hsxga_5120x4096(),
	wvga_852x480(),
	wxga_1366x768(),
	wsxga_1600x1024(),
	wuxga_1920x1200(),
	woxga_2560x1600(),
	wqsxga_3200x2048(),
	wquxga_3840x2400(),
	whsxga_6400x4096(),
	whuxga_7680x4800(),
	cga_320x200(),
	ega_640x350(),
	hd480_852x480(),
	hd720_1280x720(),
	hd1080_1920x1080()
{

}

SupportedFrameSizesBitField::~SupportedFrameSizesBitField()
{

}

bool SupportedFrameSizesBitField::setSqcif_128x96(bool value)
{
	this->sqcif_128x96 = value;
	return true;
}

bool SupportedFrameSizesBitField::getSqcif_128x96(void) const
{
	return this->sqcif_128x96;
}


bool SupportedFrameSizesBitField::setQcif_176x144(bool value)
{
	this->qcif_176x144 = value;
	return true;
}

bool SupportedFrameSizesBitField::getQcif_176x144(void) const
{
	return this->qcif_176x144;
}


bool SupportedFrameSizesBitField::setCif_352x288(bool value)
{
	this->cif_352x288 = value;
	return true;
}

bool SupportedFrameSizesBitField::getCif_352x288(void) const
{
	return this->cif_352x288;
}


bool SupportedFrameSizesBitField::setCif4_704x576(bool value)
{
	this->cif4_704x576 = value;
	return true;
}

bool SupportedFrameSizesBitField::getCif4_704x576(void) const
{
	return this->cif4_704x576;
}


bool SupportedFrameSizesBitField::setCif16_1408x1152(bool value)
{
	this->cif16_1408x1152 = value;
	return true;
}

bool SupportedFrameSizesBitField::getCif16_1408x1152(void) const
{
	return this->cif16_1408x1152;
}


bool SupportedFrameSizesBitField::setQqvga_160x120(bool value)
{
	this->qqvga_160x120 = value;
	return true;
}

bool SupportedFrameSizesBitField::getQqvga_160x120(void) const
{
	return this->qqvga_160x120;
}


bool SupportedFrameSizesBitField::setQvga_320x240(bool value)
{
	this->qvga_320x240 = value;
	return true;
}

bool SupportedFrameSizesBitField::getQvga_320x240(void) const
{
	return this->qvga_320x240;
}


bool SupportedFrameSizesBitField::setVga_640x480(bool value)
{
	this->vga_640x480 = value;
	return true;
}

bool SupportedFrameSizesBitField::getVga_640x480(void) const
{
	return this->vga_640x480;
}


bool SupportedFrameSizesBitField::setSvga_800x600(bool value)
{
	this->svga_800x600 = value;
	return true;
}

bool SupportedFrameSizesBitField::getSvga_800x600(void) const
{
	return this->svga_800x600;
}


bool SupportedFrameSizesBitField::setXga_1024x768(bool value)
{
	this->xga_1024x768 = value;
	return true;
}

bool SupportedFrameSizesBitField::getXga_1024x768(void) const
{
	return this->xga_1024x768;
}


bool SupportedFrameSizesBitField::setUxga_1600x1200(bool value)
{
	this->uxga_1600x1200 = value;
	return true;
}

bool SupportedFrameSizesBitField::getUxga_1600x1200(void) const
{
	return this->uxga_1600x1200;
}


bool SupportedFrameSizesBitField::setQxga_2048x1536(bool value)
{
	this->qxga_2048x1536 = value;
	return true;
}

bool SupportedFrameSizesBitField::getQxga_2048x1536(void) const
{
	return this->qxga_2048x1536;
}


bool SupportedFrameSizesBitField::setSxga_1280x1024(bool value)
{
	this->sxga_1280x1024 = value;
	return true;
}

bool SupportedFrameSizesBitField::getSxga_1280x1024(void) const
{
	return this->sxga_1280x1024;
}


bool SupportedFrameSizesBitField::setQsxga_2560x2048(bool value)
{
	this->qsxga_2560x2048 = value;
	return true;
}

bool SupportedFrameSizesBitField::getQsxga_2560x2048(void) const
{
	return this->qsxga_2560x2048;
}


bool SupportedFrameSizesBitField::setHsxga_5120x4096(bool value)
{
	this->hsxga_5120x4096 = value;
	return true;
}

bool SupportedFrameSizesBitField::getHsxga_5120x4096(void) const
{
	return this->hsxga_5120x4096;
}


bool SupportedFrameSizesBitField::setWvga_852x480(bool value)
{
	this->wvga_852x480 = value;
	return true;
}

bool SupportedFrameSizesBitField::getWvga_852x480(void) const
{
	return this->wvga_852x480;
}


bool SupportedFrameSizesBitField::setWxga_1366x768(bool value)
{
	this->wxga_1366x768 = value;
	return true;
}

bool SupportedFrameSizesBitField::getWxga_1366x768(void) const
{
	return this->wxga_1366x768;
}


bool SupportedFrameSizesBitField::setWsxga_1600x1024(bool value)
{
	this->wsxga_1600x1024 = value;
	return true;
}

bool SupportedFrameSizesBitField::getWsxga_1600x1024(void) const
{
	return this->wsxga_1600x1024;
}


bool SupportedFrameSizesBitField::setWuxga_1920x1200(bool value)
{
	this->wuxga_1920x1200 = value;
	return true;
}

bool SupportedFrameSizesBitField::getWuxga_1920x1200(void) const
{
	return this->wuxga_1920x1200;
}


bool SupportedFrameSizesBitField::setWoxga_2560x1600(bool value)
{
	this->woxga_2560x1600 = value;
	return true;
}

bool SupportedFrameSizesBitField::getWoxga_2560x1600(void) const
{
	return this->woxga_2560x1600;
}


bool SupportedFrameSizesBitField::setWqsxga_3200x2048(bool value)
{
	this->wqsxga_3200x2048 = value;
	return true;
}

bool SupportedFrameSizesBitField::getWqsxga_3200x2048(void) const
{
	return this->wqsxga_3200x2048;
}


bool SupportedFrameSizesBitField::setWquxga_3840x2400(bool value)
{
	this->wquxga_3840x2400 = value;
	return true;
}

bool SupportedFrameSizesBitField::getWquxga_3840x2400(void) const
{
	return this->wquxga_3840x2400;
}


bool SupportedFrameSizesBitField::setWhsxga_6400x4096(bool value)
{
	this->whsxga_6400x4096 = value;
	return true;
}

bool SupportedFrameSizesBitField::getWhsxga_6400x4096(void) const
{
	return this->whsxga_6400x4096;
}


bool SupportedFrameSizesBitField::setWhuxga_7680x4800(bool value)
{
	this->whuxga_7680x4800 = value;
	return true;
}

bool SupportedFrameSizesBitField::getWhuxga_7680x4800(void) const
{
	return this->whuxga_7680x4800;
}


bool SupportedFrameSizesBitField::setCga_320x200(bool value)
{
	this->cga_320x200 = value;
	return true;
}

bool SupportedFrameSizesBitField::getCga_320x200(void) const
{
	return this->cga_320x200;
}


bool SupportedFrameSizesBitField::setEga_640x350(bool value)
{
	this->ega_640x350 = value;
	return true;
}

bool SupportedFrameSizesBitField::getEga_640x350(void) const
{
	return this->ega_640x350;
}


bool SupportedFrameSizesBitField::setHd480_852x480(bool value)
{
	this->hd480_852x480 = value;
	return true;
}

bool SupportedFrameSizesBitField::getHd480_852x480(void) const
{
	return this->hd480_852x480;
}


bool SupportedFrameSizesBitField::setHd720_1280x720(bool value)
{
	this->hd720_1280x720 = value;
	return true;
}

bool SupportedFrameSizesBitField::getHd720_1280x720(void) const
{
	return this->hd720_1280x720;
}


bool SupportedFrameSizesBitField::setHd1080_1920x1080(bool value)
{
	this->hd1080_1920x1080 = value;
	return true;
}

bool SupportedFrameSizesBitField::getHd1080_1920x1080(void) const
{
	return this->hd1080_1920x1080;
}



int SupportedFrameSizesBitField::to(system::Buffer *dst)
{
	uint32_t intValue = 0;

	intValue |= ((this->sqcif_128x96 & SupportedFrameSizesBitField::SQCIF_128X96_BIT_MASK) << SupportedFrameSizesBitField::SQCIF_128X96_START_BIT);
	intValue |= ((this->qcif_176x144 & SupportedFrameSizesBitField::QCIF_176X144_BIT_MASK) << SupportedFrameSizesBitField::QCIF_176X144_START_BIT);
	intValue |= ((this->cif_352x288 & SupportedFrameSizesBitField::CIF_352X288_BIT_MASK) << SupportedFrameSizesBitField::CIF_352X288_START_BIT);
	intValue |= ((this->cif4_704x576 & SupportedFrameSizesBitField::CIF4_704X576_BIT_MASK) << SupportedFrameSizesBitField::CIF4_704X576_START_BIT);
	intValue |= ((this->cif16_1408x1152 & SupportedFrameSizesBitField::CIF16_1408X1152_BIT_MASK) << SupportedFrameSizesBitField::CIF16_1408X1152_START_BIT);
	intValue |= ((this->qqvga_160x120 & SupportedFrameSizesBitField::QQVGA_160X120_BIT_MASK) << SupportedFrameSizesBitField::QQVGA_160X120_START_BIT);
	intValue |= ((this->qvga_320x240 & SupportedFrameSizesBitField::QVGA_320X240_BIT_MASK) << SupportedFrameSizesBitField::QVGA_320X240_START_BIT);
	intValue |= ((this->vga_640x480 & SupportedFrameSizesBitField::VGA_640X480_BIT_MASK) << SupportedFrameSizesBitField::VGA_640X480_START_BIT);
	intValue |= ((this->svga_800x600 & SupportedFrameSizesBitField::SVGA_800X600_BIT_MASK) << SupportedFrameSizesBitField::SVGA_800X600_START_BIT);
	intValue |= ((this->xga_1024x768 & SupportedFrameSizesBitField::XGA_1024X768_BIT_MASK) << SupportedFrameSizesBitField::XGA_1024X768_START_BIT);
	intValue |= ((this->uxga_1600x1200 & SupportedFrameSizesBitField::UXGA_1600X1200_BIT_MASK) << SupportedFrameSizesBitField::UXGA_1600X1200_START_BIT);
	intValue |= ((this->qxga_2048x1536 & SupportedFrameSizesBitField::QXGA_2048X1536_BIT_MASK) << SupportedFrameSizesBitField::QXGA_2048X1536_START_BIT);
	intValue |= ((this->sxga_1280x1024 & SupportedFrameSizesBitField::SXGA_1280X1024_BIT_MASK) << SupportedFrameSizesBitField::SXGA_1280X1024_START_BIT);
	intValue |= ((this->qsxga_2560x2048 & SupportedFrameSizesBitField::QSXGA_2560X2048_BIT_MASK) << SupportedFrameSizesBitField::QSXGA_2560X2048_START_BIT);
	intValue |= ((this->hsxga_5120x4096 & SupportedFrameSizesBitField::HSXGA_5120X4096_BIT_MASK) << SupportedFrameSizesBitField::HSXGA_5120X4096_START_BIT);
	intValue |= ((this->wvga_852x480 & SupportedFrameSizesBitField::WVGA_852X480_BIT_MASK) << SupportedFrameSizesBitField::WVGA_852X480_START_BIT);
	intValue |= ((this->wxga_1366x768 & SupportedFrameSizesBitField::WXGA_1366X768_BIT_MASK) << SupportedFrameSizesBitField::WXGA_1366X768_START_BIT);
	intValue |= ((this->wsxga_1600x1024 & SupportedFrameSizesBitField::WSXGA_1600X1024_BIT_MASK) << SupportedFrameSizesBitField::WSXGA_1600X1024_START_BIT);
	intValue |= ((this->wuxga_1920x1200 & SupportedFrameSizesBitField::WUXGA_1920X1200_BIT_MASK) << SupportedFrameSizesBitField::WUXGA_1920X1200_START_BIT);
	intValue |= ((this->woxga_2560x1600 & SupportedFrameSizesBitField::WOXGA_2560X1600_BIT_MASK) << SupportedFrameSizesBitField::WOXGA_2560X1600_START_BIT);
	intValue |= ((this->wqsxga_3200x2048 & SupportedFrameSizesBitField::WQSXGA_3200X2048_BIT_MASK) << SupportedFrameSizesBitField::WQSXGA_3200X2048_START_BIT);
	intValue |= ((this->wquxga_3840x2400 & SupportedFrameSizesBitField::WQUXGA_3840X2400_BIT_MASK) << SupportedFrameSizesBitField::WQUXGA_3840X2400_START_BIT);
	intValue |= ((this->whsxga_6400x4096 & SupportedFrameSizesBitField::WHSXGA_6400X4096_BIT_MASK) << SupportedFrameSizesBitField::WHSXGA_6400X4096_START_BIT);
	intValue |= ((this->whuxga_7680x4800 & SupportedFrameSizesBitField::WHUXGA_7680X4800_BIT_MASK) << SupportedFrameSizesBitField::WHUXGA_7680X4800_START_BIT);
	intValue |= ((this->cga_320x200 & SupportedFrameSizesBitField::CGA_320X200_BIT_MASK) << SupportedFrameSizesBitField::CGA_320X200_START_BIT);
	intValue |= ((this->ega_640x350 & SupportedFrameSizesBitField::EGA_640X350_BIT_MASK) << SupportedFrameSizesBitField::EGA_640X350_START_BIT);
	intValue |= ((this->hd480_852x480 & SupportedFrameSizesBitField::HD480_852X480_BIT_MASK) << SupportedFrameSizesBitField::HD480_852X480_START_BIT);
	intValue |= ((this->hd720_1280x720 & SupportedFrameSizesBitField::HD720_1280X720_BIT_MASK) << SupportedFrameSizesBitField::HD720_1280X720_START_BIT);
	intValue |= ((this->hd1080_1920x1080 & SupportedFrameSizesBitField::HD1080_1920X1080_BIT_MASK) << SupportedFrameSizesBitField::HD1080_1920X1080_START_BIT);
	return dst->pack(intValue);
}

int SupportedFrameSizesBitField::from(system::Buffer *src)
{
	int byteSize = 0;
	uint32_t intValue = 0;
	byteSize = src->unpack(intValue);

	this->sqcif_128x96 = (intValue >> (SupportedFrameSizesBitField::SQCIF_128X96_START_BIT)) & SupportedFrameSizesBitField::SQCIF_128X96_BIT_MASK;
	this->qcif_176x144 = (intValue >> (SupportedFrameSizesBitField::QCIF_176X144_START_BIT)) & SupportedFrameSizesBitField::QCIF_176X144_BIT_MASK;
	this->cif_352x288 = (intValue >> (SupportedFrameSizesBitField::CIF_352X288_START_BIT)) & SupportedFrameSizesBitField::CIF_352X288_BIT_MASK;
	this->cif4_704x576 = (intValue >> (SupportedFrameSizesBitField::CIF4_704X576_START_BIT)) & SupportedFrameSizesBitField::CIF4_704X576_BIT_MASK;
	this->cif16_1408x1152 = (intValue >> (SupportedFrameSizesBitField::CIF16_1408X1152_START_BIT)) & SupportedFrameSizesBitField::CIF16_1408X1152_BIT_MASK;
	this->qqvga_160x120 = (intValue >> (SupportedFrameSizesBitField::QQVGA_160X120_START_BIT)) & SupportedFrameSizesBitField::QQVGA_160X120_BIT_MASK;
	this->qvga_320x240 = (intValue >> (SupportedFrameSizesBitField::QVGA_320X240_START_BIT)) & SupportedFrameSizesBitField::QVGA_320X240_BIT_MASK;
	this->vga_640x480 = (intValue >> (SupportedFrameSizesBitField::VGA_640X480_START_BIT)) & SupportedFrameSizesBitField::VGA_640X480_BIT_MASK;
	this->svga_800x600 = (intValue >> (SupportedFrameSizesBitField::SVGA_800X600_START_BIT)) & SupportedFrameSizesBitField::SVGA_800X600_BIT_MASK;
	this->xga_1024x768 = (intValue >> (SupportedFrameSizesBitField::XGA_1024X768_START_BIT)) & SupportedFrameSizesBitField::XGA_1024X768_BIT_MASK;
	this->uxga_1600x1200 = (intValue >> (SupportedFrameSizesBitField::UXGA_1600X1200_START_BIT)) & SupportedFrameSizesBitField::UXGA_1600X1200_BIT_MASK;
	this->qxga_2048x1536 = (intValue >> (SupportedFrameSizesBitField::QXGA_2048X1536_START_BIT)) & SupportedFrameSizesBitField::QXGA_2048X1536_BIT_MASK;
	this->sxga_1280x1024 = (intValue >> (SupportedFrameSizesBitField::SXGA_1280X1024_START_BIT)) & SupportedFrameSizesBitField::SXGA_1280X1024_BIT_MASK;
	this->qsxga_2560x2048 = (intValue >> (SupportedFrameSizesBitField::QSXGA_2560X2048_START_BIT)) & SupportedFrameSizesBitField::QSXGA_2560X2048_BIT_MASK;
	this->hsxga_5120x4096 = (intValue >> (SupportedFrameSizesBitField::HSXGA_5120X4096_START_BIT)) & SupportedFrameSizesBitField::HSXGA_5120X4096_BIT_MASK;
	this->wvga_852x480 = (intValue >> (SupportedFrameSizesBitField::WVGA_852X480_START_BIT)) & SupportedFrameSizesBitField::WVGA_852X480_BIT_MASK;
	this->wxga_1366x768 = (intValue >> (SupportedFrameSizesBitField::WXGA_1366X768_START_BIT)) & SupportedFrameSizesBitField::WXGA_1366X768_BIT_MASK;
	this->wsxga_1600x1024 = (intValue >> (SupportedFrameSizesBitField::WSXGA_1600X1024_START_BIT)) & SupportedFrameSizesBitField::WSXGA_1600X1024_BIT_MASK;
	this->wuxga_1920x1200 = (intValue >> (SupportedFrameSizesBitField::WUXGA_1920X1200_START_BIT)) & SupportedFrameSizesBitField::WUXGA_1920X1200_BIT_MASK;
	this->woxga_2560x1600 = (intValue >> (SupportedFrameSizesBitField::WOXGA_2560X1600_START_BIT)) & SupportedFrameSizesBitField::WOXGA_2560X1600_BIT_MASK;
	this->wqsxga_3200x2048 = (intValue >> (SupportedFrameSizesBitField::WQSXGA_3200X2048_START_BIT)) & SupportedFrameSizesBitField::WQSXGA_3200X2048_BIT_MASK;
	this->wquxga_3840x2400 = (intValue >> (SupportedFrameSizesBitField::WQUXGA_3840X2400_START_BIT)) & SupportedFrameSizesBitField::WQUXGA_3840X2400_BIT_MASK;
	this->whsxga_6400x4096 = (intValue >> (SupportedFrameSizesBitField::WHSXGA_6400X4096_START_BIT)) & SupportedFrameSizesBitField::WHSXGA_6400X4096_BIT_MASK;
	this->whuxga_7680x4800 = (intValue >> (SupportedFrameSizesBitField::WHUXGA_7680X4800_START_BIT)) & SupportedFrameSizesBitField::WHUXGA_7680X4800_BIT_MASK;
	this->cga_320x200 = (intValue >> (SupportedFrameSizesBitField::CGA_320X200_START_BIT)) & SupportedFrameSizesBitField::CGA_320X200_BIT_MASK;
	this->ega_640x350 = (intValue >> (SupportedFrameSizesBitField::EGA_640X350_START_BIT)) & SupportedFrameSizesBitField::EGA_640X350_BIT_MASK;
	this->hd480_852x480 = (intValue >> (SupportedFrameSizesBitField::HD480_852X480_START_BIT)) & SupportedFrameSizesBitField::HD480_852X480_BIT_MASK;
	this->hd720_1280x720 = (intValue >> (SupportedFrameSizesBitField::HD720_1280X720_START_BIT)) & SupportedFrameSizesBitField::HD720_1280X720_BIT_MASK;
	this->hd1080_1920x1080 = (intValue >> (SupportedFrameSizesBitField::HD1080_1920X1080_START_BIT)) & SupportedFrameSizesBitField::HD1080_1920X1080_BIT_MASK;

	return byteSize;
}

int SupportedFrameSizesBitField::length(void)
{
	return sizeof(uint32_t);
}

void SupportedFrameSizesBitField::copy(SupportedFrameSizesBitField& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	setSqcif_128x96(source.getSqcif_128x96());
	setQcif_176x144(source.getQcif_176x144());
	setCif_352x288(source.getCif_352x288());
	setCif4_704x576(source.getCif4_704x576());
	setCif16_1408x1152(source.getCif16_1408x1152());
	setQqvga_160x120(source.getQqvga_160x120());
	setQvga_320x240(source.getQvga_320x240());
	setVga_640x480(source.getVga_640x480());
	setSvga_800x600(source.getSvga_800x600());
	setXga_1024x768(source.getXga_1024x768());
	setUxga_1600x1200(source.getUxga_1600x1200());
	setQxga_2048x1536(source.getQxga_2048x1536());
	setSxga_1280x1024(source.getSxga_1280x1024());
	setQsxga_2560x2048(source.getQsxga_2560x2048());
	setHsxga_5120x4096(source.getHsxga_5120x4096());
	setWvga_852x480(source.getWvga_852x480());
	setWxga_1366x768(source.getWxga_1366x768());
	setWsxga_1600x1024(source.getWsxga_1600x1024());
	setWuxga_1920x1200(source.getWuxga_1920x1200());
	setWoxga_2560x1600(source.getWoxga_2560x1600());
	setWqsxga_3200x2048(source.getWqsxga_3200x2048());
	setWquxga_3840x2400(source.getWquxga_3840x2400());
	setWhsxga_6400x4096(source.getWhsxga_6400x4096());
	setWhuxga_7680x4800(source.getWhuxga_7680x4800());
	setCga_320x200(source.getCga_320x200());
	setEga_640x350(source.getEga_640x350());
	setHd480_852x480(source.getHd480_852x480());
	setHd720_1280x720(source.getHd720_1280x720());
	setHd1080_1920x1080(source.getHd1080_1920x1080());

}

std::string SupportedFrameSizesBitField::toXml(unsigned char level) const
{
	uint32_t intValue = 0;

	intValue |= ((this->sqcif_128x96 & SupportedFrameSizesBitField::SQCIF_128X96_BIT_MASK) << SupportedFrameSizesBitField::SQCIF_128X96_START_BIT);
	intValue |= ((this->qcif_176x144 & SupportedFrameSizesBitField::QCIF_176X144_BIT_MASK) << SupportedFrameSizesBitField::QCIF_176X144_START_BIT);
	intValue |= ((this->cif_352x288 & SupportedFrameSizesBitField::CIF_352X288_BIT_MASK) << SupportedFrameSizesBitField::CIF_352X288_START_BIT);
	intValue |= ((this->cif4_704x576 & SupportedFrameSizesBitField::CIF4_704X576_BIT_MASK) << SupportedFrameSizesBitField::CIF4_704X576_START_BIT);
	intValue |= ((this->cif16_1408x1152 & SupportedFrameSizesBitField::CIF16_1408X1152_BIT_MASK) << SupportedFrameSizesBitField::CIF16_1408X1152_START_BIT);
	intValue |= ((this->qqvga_160x120 & SupportedFrameSizesBitField::QQVGA_160X120_BIT_MASK) << SupportedFrameSizesBitField::QQVGA_160X120_START_BIT);
	intValue |= ((this->qvga_320x240 & SupportedFrameSizesBitField::QVGA_320X240_BIT_MASK) << SupportedFrameSizesBitField::QVGA_320X240_START_BIT);
	intValue |= ((this->vga_640x480 & SupportedFrameSizesBitField::VGA_640X480_BIT_MASK) << SupportedFrameSizesBitField::VGA_640X480_START_BIT);
	intValue |= ((this->svga_800x600 & SupportedFrameSizesBitField::SVGA_800X600_BIT_MASK) << SupportedFrameSizesBitField::SVGA_800X600_START_BIT);
	intValue |= ((this->xga_1024x768 & SupportedFrameSizesBitField::XGA_1024X768_BIT_MASK) << SupportedFrameSizesBitField::XGA_1024X768_START_BIT);
	intValue |= ((this->uxga_1600x1200 & SupportedFrameSizesBitField::UXGA_1600X1200_BIT_MASK) << SupportedFrameSizesBitField::UXGA_1600X1200_START_BIT);
	intValue |= ((this->qxga_2048x1536 & SupportedFrameSizesBitField::QXGA_2048X1536_BIT_MASK) << SupportedFrameSizesBitField::QXGA_2048X1536_START_BIT);
	intValue |= ((this->sxga_1280x1024 & SupportedFrameSizesBitField::SXGA_1280X1024_BIT_MASK) << SupportedFrameSizesBitField::SXGA_1280X1024_START_BIT);
	intValue |= ((this->qsxga_2560x2048 & SupportedFrameSizesBitField::QSXGA_2560X2048_BIT_MASK) << SupportedFrameSizesBitField::QSXGA_2560X2048_START_BIT);
	intValue |= ((this->hsxga_5120x4096 & SupportedFrameSizesBitField::HSXGA_5120X4096_BIT_MASK) << SupportedFrameSizesBitField::HSXGA_5120X4096_START_BIT);
	intValue |= ((this->wvga_852x480 & SupportedFrameSizesBitField::WVGA_852X480_BIT_MASK) << SupportedFrameSizesBitField::WVGA_852X480_START_BIT);
	intValue |= ((this->wxga_1366x768 & SupportedFrameSizesBitField::WXGA_1366X768_BIT_MASK) << SupportedFrameSizesBitField::WXGA_1366X768_START_BIT);
	intValue |= ((this->wsxga_1600x1024 & SupportedFrameSizesBitField::WSXGA_1600X1024_BIT_MASK) << SupportedFrameSizesBitField::WSXGA_1600X1024_START_BIT);
	intValue |= ((this->wuxga_1920x1200 & SupportedFrameSizesBitField::WUXGA_1920X1200_BIT_MASK) << SupportedFrameSizesBitField::WUXGA_1920X1200_START_BIT);
	intValue |= ((this->woxga_2560x1600 & SupportedFrameSizesBitField::WOXGA_2560X1600_BIT_MASK) << SupportedFrameSizesBitField::WOXGA_2560X1600_START_BIT);
	intValue |= ((this->wqsxga_3200x2048 & SupportedFrameSizesBitField::WQSXGA_3200X2048_BIT_MASK) << SupportedFrameSizesBitField::WQSXGA_3200X2048_START_BIT);
	intValue |= ((this->wquxga_3840x2400 & SupportedFrameSizesBitField::WQUXGA_3840X2400_BIT_MASK) << SupportedFrameSizesBitField::WQUXGA_3840X2400_START_BIT);
	intValue |= ((this->whsxga_6400x4096 & SupportedFrameSizesBitField::WHSXGA_6400X4096_BIT_MASK) << SupportedFrameSizesBitField::WHSXGA_6400X4096_START_BIT);
	intValue |= ((this->whuxga_7680x4800 & SupportedFrameSizesBitField::WHUXGA_7680X4800_BIT_MASK) << SupportedFrameSizesBitField::WHUXGA_7680X4800_START_BIT);
	intValue |= ((this->cga_320x200 & SupportedFrameSizesBitField::CGA_320X200_BIT_MASK) << SupportedFrameSizesBitField::CGA_320X200_START_BIT);
	intValue |= ((this->ega_640x350 & SupportedFrameSizesBitField::EGA_640X350_BIT_MASK) << SupportedFrameSizesBitField::EGA_640X350_START_BIT);
	intValue |= ((this->hd480_852x480 & SupportedFrameSizesBitField::HD480_852X480_BIT_MASK) << SupportedFrameSizesBitField::HD480_852X480_START_BIT);
	intValue |= ((this->hd720_1280x720 & SupportedFrameSizesBitField::HD720_1280X720_BIT_MASK) << SupportedFrameSizesBitField::HD720_1280X720_START_BIT);
	intValue |= ((this->hd1080_1920x1080 & SupportedFrameSizesBitField::HD1080_1920X1080_BIT_MASK) << SupportedFrameSizesBitField::HD1080_1920X1080_START_BIT);

	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<BitField name=\"" << this->name << "\">\n";
	oss << prefix.str() << "\t" << "<intValue>" << intValue << "</intValue>\n";
	oss << prefix.str() << "\t" << "<fields>\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"sqcif_128x96\" value=\"" << getSqcif_128x96() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"qcif_176x144\" value=\"" << getQcif_176x144() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"cif_352x288\" value=\"" << getCif_352x288() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"cif4_704x576\" value=\"" << getCif4_704x576() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"cif16_1408x1152\" value=\"" << getCif16_1408x1152() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"qqvga_160x120\" value=\"" << getQqvga_160x120() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"qvga_320x240\" value=\"" << getQvga_320x240() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"vga_640x480\" value=\"" << getVga_640x480() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"svga_800x600\" value=\"" << getSvga_800x600() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"xga_1024x768\" value=\"" << getXga_1024x768() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"uxga_1600x1200\" value=\"" << getUxga_1600x1200() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"qxga_2048x1536\" value=\"" << getQxga_2048x1536() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"sxga_1280x1024\" value=\"" << getSxga_1280x1024() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"qsxga_2560x2048\" value=\"" << getQsxga_2560x2048() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"hsxga_5120x4096\" value=\"" << getHsxga_5120x4096() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"wvga_852x480\" value=\"" << getWvga_852x480() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"wxga_1366x768\" value=\"" << getWxga_1366x768() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"wsxga_1600x1024\" value=\"" << getWsxga_1600x1024() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"wuxga_1920x1200\" value=\"" << getWuxga_1920x1200() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"woxga_2560x1600\" value=\"" << getWoxga_2560x1600() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"wqsxga_3200x2048\" value=\"" << getWqsxga_3200x2048() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"wquxga_3840x2400\" value=\"" << getWquxga_3840x2400() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"whsxga_6400x4096\" value=\"" << getWhsxga_6400x4096() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"whuxga_7680x4800\" value=\"" << getWhuxga_7680x4800() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"cga_320x200\" value=\"" << getCga_320x200() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"ega_640x350\" value=\"" << getEga_640x350() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"hd480_852x480\" value=\"" << getHd480_852x480() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"hd720_1280x720\" value=\"" << getHd720_1280x720() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"hd1080_1920x1080\" value=\"" << getHd1080_1920x1080() << "\" />\n";
	oss << prefix.str() << "\t" << "</fields>\n";
	oss << prefix.str() << "</BitField>\n";
	return oss.str();
}

} // namespace environment
} // namespace openjaus

