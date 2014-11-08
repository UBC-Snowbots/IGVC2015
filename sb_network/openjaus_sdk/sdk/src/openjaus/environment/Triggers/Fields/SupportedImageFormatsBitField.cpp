

/**
\file SupportedImageFormatsBitField.h

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
#include "openjaus/environment/Triggers/Fields/SupportedImageFormatsBitField.h"

namespace openjaus
{
namespace environment
{

SupportedImageFormatsBitField::SupportedImageFormatsBitField() : 
	jPEG(),
	gIF(),
	pNG(),
	bMP(),
	tIFF(),
	pPM(),
	pGM(),
	pNM(),
	nEF_Nikon_RAW(),
	cR2_Canon_RAW(),
	dNG_Adobe_RAW()
{

}

SupportedImageFormatsBitField::~SupportedImageFormatsBitField()
{

}

bool SupportedImageFormatsBitField::setJPEG(bool value)
{
	this->jPEG = value;
	return true;
}

bool SupportedImageFormatsBitField::getJPEG(void) const
{
	return this->jPEG;
}


bool SupportedImageFormatsBitField::setGIF(bool value)
{
	this->gIF = value;
	return true;
}

bool SupportedImageFormatsBitField::getGIF(void) const
{
	return this->gIF;
}


bool SupportedImageFormatsBitField::setPNG(bool value)
{
	this->pNG = value;
	return true;
}

bool SupportedImageFormatsBitField::getPNG(void) const
{
	return this->pNG;
}


bool SupportedImageFormatsBitField::setBMP(bool value)
{
	this->bMP = value;
	return true;
}

bool SupportedImageFormatsBitField::getBMP(void) const
{
	return this->bMP;
}


bool SupportedImageFormatsBitField::setTIFF(bool value)
{
	this->tIFF = value;
	return true;
}

bool SupportedImageFormatsBitField::getTIFF(void) const
{
	return this->tIFF;
}


bool SupportedImageFormatsBitField::setPPM(bool value)
{
	this->pPM = value;
	return true;
}

bool SupportedImageFormatsBitField::getPPM(void) const
{
	return this->pPM;
}


bool SupportedImageFormatsBitField::setPGM(bool value)
{
	this->pGM = value;
	return true;
}

bool SupportedImageFormatsBitField::getPGM(void) const
{
	return this->pGM;
}


bool SupportedImageFormatsBitField::setPNM(bool value)
{
	this->pNM = value;
	return true;
}

bool SupportedImageFormatsBitField::getPNM(void) const
{
	return this->pNM;
}


bool SupportedImageFormatsBitField::setNEF_Nikon_RAW(bool value)
{
	this->nEF_Nikon_RAW = value;
	return true;
}

bool SupportedImageFormatsBitField::getNEF_Nikon_RAW(void) const
{
	return this->nEF_Nikon_RAW;
}


bool SupportedImageFormatsBitField::setCR2_Canon_RAW(bool value)
{
	this->cR2_Canon_RAW = value;
	return true;
}

bool SupportedImageFormatsBitField::getCR2_Canon_RAW(void) const
{
	return this->cR2_Canon_RAW;
}


bool SupportedImageFormatsBitField::setDNG_Adobe_RAW(bool value)
{
	this->dNG_Adobe_RAW = value;
	return true;
}

bool SupportedImageFormatsBitField::getDNG_Adobe_RAW(void) const
{
	return this->dNG_Adobe_RAW;
}



int SupportedImageFormatsBitField::to(system::Buffer *dst)
{
	uint16_t intValue = 0;

	intValue |= ((this->jPEG & SupportedImageFormatsBitField::JPEG_BIT_MASK) << SupportedImageFormatsBitField::JPEG_START_BIT);
	intValue |= ((this->gIF & SupportedImageFormatsBitField::GIF_BIT_MASK) << SupportedImageFormatsBitField::GIF_START_BIT);
	intValue |= ((this->pNG & SupportedImageFormatsBitField::PNG_BIT_MASK) << SupportedImageFormatsBitField::PNG_START_BIT);
	intValue |= ((this->bMP & SupportedImageFormatsBitField::BMP_BIT_MASK) << SupportedImageFormatsBitField::BMP_START_BIT);
	intValue |= ((this->tIFF & SupportedImageFormatsBitField::TIFF_BIT_MASK) << SupportedImageFormatsBitField::TIFF_START_BIT);
	intValue |= ((this->pPM & SupportedImageFormatsBitField::PPM_BIT_MASK) << SupportedImageFormatsBitField::PPM_START_BIT);
	intValue |= ((this->pGM & SupportedImageFormatsBitField::PGM_BIT_MASK) << SupportedImageFormatsBitField::PGM_START_BIT);
	intValue |= ((this->pNM & SupportedImageFormatsBitField::PNM_BIT_MASK) << SupportedImageFormatsBitField::PNM_START_BIT);
	intValue |= ((this->nEF_Nikon_RAW & SupportedImageFormatsBitField::NEF_NIKON_RAW_BIT_MASK) << SupportedImageFormatsBitField::NEF_NIKON_RAW_START_BIT);
	intValue |= ((this->cR2_Canon_RAW & SupportedImageFormatsBitField::CR2_CANON_RAW_BIT_MASK) << SupportedImageFormatsBitField::CR2_CANON_RAW_START_BIT);
	intValue |= ((this->dNG_Adobe_RAW & SupportedImageFormatsBitField::DNG_ADOBE_RAW_BIT_MASK) << SupportedImageFormatsBitField::DNG_ADOBE_RAW_START_BIT);
	return dst->pack(intValue);
}

int SupportedImageFormatsBitField::from(system::Buffer *src)
{
	int byteSize = 0;
	uint16_t intValue = 0;
	byteSize = src->unpack(intValue);

	this->jPEG = (intValue >> (SupportedImageFormatsBitField::JPEG_START_BIT)) & SupportedImageFormatsBitField::JPEG_BIT_MASK;
	this->gIF = (intValue >> (SupportedImageFormatsBitField::GIF_START_BIT)) & SupportedImageFormatsBitField::GIF_BIT_MASK;
	this->pNG = (intValue >> (SupportedImageFormatsBitField::PNG_START_BIT)) & SupportedImageFormatsBitField::PNG_BIT_MASK;
	this->bMP = (intValue >> (SupportedImageFormatsBitField::BMP_START_BIT)) & SupportedImageFormatsBitField::BMP_BIT_MASK;
	this->tIFF = (intValue >> (SupportedImageFormatsBitField::TIFF_START_BIT)) & SupportedImageFormatsBitField::TIFF_BIT_MASK;
	this->pPM = (intValue >> (SupportedImageFormatsBitField::PPM_START_BIT)) & SupportedImageFormatsBitField::PPM_BIT_MASK;
	this->pGM = (intValue >> (SupportedImageFormatsBitField::PGM_START_BIT)) & SupportedImageFormatsBitField::PGM_BIT_MASK;
	this->pNM = (intValue >> (SupportedImageFormatsBitField::PNM_START_BIT)) & SupportedImageFormatsBitField::PNM_BIT_MASK;
	this->nEF_Nikon_RAW = (intValue >> (SupportedImageFormatsBitField::NEF_NIKON_RAW_START_BIT)) & SupportedImageFormatsBitField::NEF_NIKON_RAW_BIT_MASK;
	this->cR2_Canon_RAW = (intValue >> (SupportedImageFormatsBitField::CR2_CANON_RAW_START_BIT)) & SupportedImageFormatsBitField::CR2_CANON_RAW_BIT_MASK;
	this->dNG_Adobe_RAW = (intValue >> (SupportedImageFormatsBitField::DNG_ADOBE_RAW_START_BIT)) & SupportedImageFormatsBitField::DNG_ADOBE_RAW_BIT_MASK;

	return byteSize;
}

int SupportedImageFormatsBitField::length(void)
{
	return sizeof(uint16_t);
}

void SupportedImageFormatsBitField::copy(SupportedImageFormatsBitField& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	setJPEG(source.getJPEG());
	setGIF(source.getGIF());
	setPNG(source.getPNG());
	setBMP(source.getBMP());
	setTIFF(source.getTIFF());
	setPPM(source.getPPM());
	setPGM(source.getPGM());
	setPNM(source.getPNM());
	setNEF_Nikon_RAW(source.getNEF_Nikon_RAW());
	setCR2_Canon_RAW(source.getCR2_Canon_RAW());
	setDNG_Adobe_RAW(source.getDNG_Adobe_RAW());

}

std::string SupportedImageFormatsBitField::toXml(unsigned char level) const
{
	uint16_t intValue = 0;

	intValue |= ((this->jPEG & SupportedImageFormatsBitField::JPEG_BIT_MASK) << SupportedImageFormatsBitField::JPEG_START_BIT);
	intValue |= ((this->gIF & SupportedImageFormatsBitField::GIF_BIT_MASK) << SupportedImageFormatsBitField::GIF_START_BIT);
	intValue |= ((this->pNG & SupportedImageFormatsBitField::PNG_BIT_MASK) << SupportedImageFormatsBitField::PNG_START_BIT);
	intValue |= ((this->bMP & SupportedImageFormatsBitField::BMP_BIT_MASK) << SupportedImageFormatsBitField::BMP_START_BIT);
	intValue |= ((this->tIFF & SupportedImageFormatsBitField::TIFF_BIT_MASK) << SupportedImageFormatsBitField::TIFF_START_BIT);
	intValue |= ((this->pPM & SupportedImageFormatsBitField::PPM_BIT_MASK) << SupportedImageFormatsBitField::PPM_START_BIT);
	intValue |= ((this->pGM & SupportedImageFormatsBitField::PGM_BIT_MASK) << SupportedImageFormatsBitField::PGM_START_BIT);
	intValue |= ((this->pNM & SupportedImageFormatsBitField::PNM_BIT_MASK) << SupportedImageFormatsBitField::PNM_START_BIT);
	intValue |= ((this->nEF_Nikon_RAW & SupportedImageFormatsBitField::NEF_NIKON_RAW_BIT_MASK) << SupportedImageFormatsBitField::NEF_NIKON_RAW_START_BIT);
	intValue |= ((this->cR2_Canon_RAW & SupportedImageFormatsBitField::CR2_CANON_RAW_BIT_MASK) << SupportedImageFormatsBitField::CR2_CANON_RAW_START_BIT);
	intValue |= ((this->dNG_Adobe_RAW & SupportedImageFormatsBitField::DNG_ADOBE_RAW_BIT_MASK) << SupportedImageFormatsBitField::DNG_ADOBE_RAW_START_BIT);

	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<BitField name=\"" << this->name << "\">\n";
	oss << prefix.str() << "\t" << "<intValue>" << intValue << "</intValue>\n";
	oss << prefix.str() << "\t" << "<fields>\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"JPEG\" value=\"" << getJPEG() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"GIF\" value=\"" << getGIF() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"PNG\" value=\"" << getPNG() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"BMP\" value=\"" << getBMP() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"TIFF\" value=\"" << getTIFF() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"PPM\" value=\"" << getPPM() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"PGM\" value=\"" << getPGM() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"PNM\" value=\"" << getPNM() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"NEF_Nikon_RAW\" value=\"" << getNEF_Nikon_RAW() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"CR2_Canon_RAW\" value=\"" << getCR2_Canon_RAW() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"DNG_Adobe_RAW\" value=\"" << getDNG_Adobe_RAW() << "\" />\n";
	oss << prefix.str() << "\t" << "</fields>\n";
	oss << prefix.str() << "</BitField>\n";
	return oss.str();
}

} // namespace environment
} // namespace openjaus

