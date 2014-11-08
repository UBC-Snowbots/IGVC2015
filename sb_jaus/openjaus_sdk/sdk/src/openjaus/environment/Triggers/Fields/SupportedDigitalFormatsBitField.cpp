

/**
\file SupportedDigitalFormatsBitField.h

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
#include "openjaus/environment/Triggers/Fields/SupportedDigitalFormatsBitField.h"

namespace openjaus
{
namespace environment
{

SupportedDigitalFormatsBitField::SupportedDigitalFormatsBitField() : 
	aVI(),
	mJPEG(),
	mPEG2(),
	h263(),
	h263plus(),
	mPEG4_Visual(),
	mPEG4_AVC_h264()
{

}

SupportedDigitalFormatsBitField::~SupportedDigitalFormatsBitField()
{

}

bool SupportedDigitalFormatsBitField::setAVI(bool value)
{
	this->aVI = value;
	return true;
}

bool SupportedDigitalFormatsBitField::getAVI(void) const
{
	return this->aVI;
}


bool SupportedDigitalFormatsBitField::setMJPEG(bool value)
{
	this->mJPEG = value;
	return true;
}

bool SupportedDigitalFormatsBitField::getMJPEG(void) const
{
	return this->mJPEG;
}


bool SupportedDigitalFormatsBitField::setMPEG2(bool value)
{
	this->mPEG2 = value;
	return true;
}

bool SupportedDigitalFormatsBitField::getMPEG2(void) const
{
	return this->mPEG2;
}


bool SupportedDigitalFormatsBitField::setH263(bool value)
{
	this->h263 = value;
	return true;
}

bool SupportedDigitalFormatsBitField::getH263(void) const
{
	return this->h263;
}


bool SupportedDigitalFormatsBitField::setH263plus(bool value)
{
	this->h263plus = value;
	return true;
}

bool SupportedDigitalFormatsBitField::getH263plus(void) const
{
	return this->h263plus;
}


bool SupportedDigitalFormatsBitField::setMPEG4_Visual(bool value)
{
	this->mPEG4_Visual = value;
	return true;
}

bool SupportedDigitalFormatsBitField::getMPEG4_Visual(void) const
{
	return this->mPEG4_Visual;
}


bool SupportedDigitalFormatsBitField::setMPEG4_AVC_h264(bool value)
{
	this->mPEG4_AVC_h264 = value;
	return true;
}

bool SupportedDigitalFormatsBitField::getMPEG4_AVC_h264(void) const
{
	return this->mPEG4_AVC_h264;
}



int SupportedDigitalFormatsBitField::to(system::Buffer *dst)
{
	uint8_t intValue = 0;

	intValue |= ((this->aVI & SupportedDigitalFormatsBitField::AVI_BIT_MASK) << SupportedDigitalFormatsBitField::AVI_START_BIT);
	intValue |= ((this->mJPEG & SupportedDigitalFormatsBitField::MJPEG_BIT_MASK) << SupportedDigitalFormatsBitField::MJPEG_START_BIT);
	intValue |= ((this->mPEG2 & SupportedDigitalFormatsBitField::MPEG2_BIT_MASK) << SupportedDigitalFormatsBitField::MPEG2_START_BIT);
	intValue |= ((this->h263 & SupportedDigitalFormatsBitField::H263_BIT_MASK) << SupportedDigitalFormatsBitField::H263_START_BIT);
	intValue |= ((this->h263plus & SupportedDigitalFormatsBitField::H263PLUS_BIT_MASK) << SupportedDigitalFormatsBitField::H263PLUS_START_BIT);
	intValue |= ((this->mPEG4_Visual & SupportedDigitalFormatsBitField::MPEG4_VISUAL_BIT_MASK) << SupportedDigitalFormatsBitField::MPEG4_VISUAL_START_BIT);
	intValue |= ((this->mPEG4_AVC_h264 & SupportedDigitalFormatsBitField::MPEG4_AVC_H264_BIT_MASK) << SupportedDigitalFormatsBitField::MPEG4_AVC_H264_START_BIT);
	return dst->pack(intValue);
}

int SupportedDigitalFormatsBitField::from(system::Buffer *src)
{
	int byteSize = 0;
	uint8_t intValue = 0;
	byteSize = src->unpack(intValue);

	this->aVI = (intValue >> (SupportedDigitalFormatsBitField::AVI_START_BIT)) & SupportedDigitalFormatsBitField::AVI_BIT_MASK;
	this->mJPEG = (intValue >> (SupportedDigitalFormatsBitField::MJPEG_START_BIT)) & SupportedDigitalFormatsBitField::MJPEG_BIT_MASK;
	this->mPEG2 = (intValue >> (SupportedDigitalFormatsBitField::MPEG2_START_BIT)) & SupportedDigitalFormatsBitField::MPEG2_BIT_MASK;
	this->h263 = (intValue >> (SupportedDigitalFormatsBitField::H263_START_BIT)) & SupportedDigitalFormatsBitField::H263_BIT_MASK;
	this->h263plus = (intValue >> (SupportedDigitalFormatsBitField::H263PLUS_START_BIT)) & SupportedDigitalFormatsBitField::H263PLUS_BIT_MASK;
	this->mPEG4_Visual = (intValue >> (SupportedDigitalFormatsBitField::MPEG4_VISUAL_START_BIT)) & SupportedDigitalFormatsBitField::MPEG4_VISUAL_BIT_MASK;
	this->mPEG4_AVC_h264 = (intValue >> (SupportedDigitalFormatsBitField::MPEG4_AVC_H264_START_BIT)) & SupportedDigitalFormatsBitField::MPEG4_AVC_H264_BIT_MASK;

	return byteSize;
}

int SupportedDigitalFormatsBitField::length(void)
{
	return sizeof(uint8_t);
}

void SupportedDigitalFormatsBitField::copy(SupportedDigitalFormatsBitField& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	setAVI(source.getAVI());
	setMJPEG(source.getMJPEG());
	setMPEG2(source.getMPEG2());
	setH263(source.getH263());
	setH263plus(source.getH263plus());
	setMPEG4_Visual(source.getMPEG4_Visual());
	setMPEG4_AVC_h264(source.getMPEG4_AVC_h264());

}

std::string SupportedDigitalFormatsBitField::toXml(unsigned char level) const
{
	uint8_t intValue = 0;

	intValue |= ((this->aVI & SupportedDigitalFormatsBitField::AVI_BIT_MASK) << SupportedDigitalFormatsBitField::AVI_START_BIT);
	intValue |= ((this->mJPEG & SupportedDigitalFormatsBitField::MJPEG_BIT_MASK) << SupportedDigitalFormatsBitField::MJPEG_START_BIT);
	intValue |= ((this->mPEG2 & SupportedDigitalFormatsBitField::MPEG2_BIT_MASK) << SupportedDigitalFormatsBitField::MPEG2_START_BIT);
	intValue |= ((this->h263 & SupportedDigitalFormatsBitField::H263_BIT_MASK) << SupportedDigitalFormatsBitField::H263_START_BIT);
	intValue |= ((this->h263plus & SupportedDigitalFormatsBitField::H263PLUS_BIT_MASK) << SupportedDigitalFormatsBitField::H263PLUS_START_BIT);
	intValue |= ((this->mPEG4_Visual & SupportedDigitalFormatsBitField::MPEG4_VISUAL_BIT_MASK) << SupportedDigitalFormatsBitField::MPEG4_VISUAL_START_BIT);
	intValue |= ((this->mPEG4_AVC_h264 & SupportedDigitalFormatsBitField::MPEG4_AVC_H264_BIT_MASK) << SupportedDigitalFormatsBitField::MPEG4_AVC_H264_START_BIT);

	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<BitField name=\"" << this->name << "\">\n";
	oss << prefix.str() << "\t" << "<intValue>" << intValue << "</intValue>\n";
	oss << prefix.str() << "\t" << "<fields>\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"AVI\" value=\"" << getAVI() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"MJPEG\" value=\"" << getMJPEG() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"MPEG2\" value=\"" << getMPEG2() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"h263\" value=\"" << getH263() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"h263plus\" value=\"" << getH263plus() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"MPEG4_Visual\" value=\"" << getMPEG4_Visual() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"MPEG4_AVC_h264\" value=\"" << getMPEG4_AVC_h264() << "\" />\n";
	oss << prefix.str() << "\t" << "</fields>\n";
	oss << prefix.str() << "</BitField>\n";
	return oss.str();
}

} // namespace environment
} // namespace openjaus

