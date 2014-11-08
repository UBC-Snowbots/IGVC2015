

/**
\file SupportedCompressionBitField.h

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
#include "openjaus/environment/Triggers/Fields/SupportedCompressionBitField.h"

namespace openjaus
{
namespace environment
{

SupportedCompressionBitField::SupportedCompressionBitField() : 
	noCompression(),
	dEFLATE(),
	bzip2(),
	lZMA()
{

}

SupportedCompressionBitField::~SupportedCompressionBitField()
{

}

bool SupportedCompressionBitField::setNoCompression(bool value)
{
	this->noCompression = value;
	return true;
}

bool SupportedCompressionBitField::getNoCompression(void) const
{
	return this->noCompression;
}


bool SupportedCompressionBitField::setDEFLATE(bool value)
{
	this->dEFLATE = value;
	return true;
}

bool SupportedCompressionBitField::getDEFLATE(void) const
{
	return this->dEFLATE;
}


bool SupportedCompressionBitField::setBzip2(bool value)
{
	this->bzip2 = value;
	return true;
}

bool SupportedCompressionBitField::getBzip2(void) const
{
	return this->bzip2;
}


bool SupportedCompressionBitField::setLZMA(bool value)
{
	this->lZMA = value;
	return true;
}

bool SupportedCompressionBitField::getLZMA(void) const
{
	return this->lZMA;
}



int SupportedCompressionBitField::to(system::Buffer *dst)
{
	uint8_t intValue = 0;

	intValue |= ((this->noCompression & SupportedCompressionBitField::NOCOMPRESSION_BIT_MASK) << SupportedCompressionBitField::NOCOMPRESSION_START_BIT);
	intValue |= ((this->dEFLATE & SupportedCompressionBitField::DEFLATE_BIT_MASK) << SupportedCompressionBitField::DEFLATE_START_BIT);
	intValue |= ((this->bzip2 & SupportedCompressionBitField::BZIP2_BIT_MASK) << SupportedCompressionBitField::BZIP2_START_BIT);
	intValue |= ((this->lZMA & SupportedCompressionBitField::LZMA_BIT_MASK) << SupportedCompressionBitField::LZMA_START_BIT);
	return dst->pack(intValue);
}

int SupportedCompressionBitField::from(system::Buffer *src)
{
	int byteSize = 0;
	uint8_t intValue = 0;
	byteSize = src->unpack(intValue);

	this->noCompression = (intValue >> (SupportedCompressionBitField::NOCOMPRESSION_START_BIT)) & SupportedCompressionBitField::NOCOMPRESSION_BIT_MASK;
	this->dEFLATE = (intValue >> (SupportedCompressionBitField::DEFLATE_START_BIT)) & SupportedCompressionBitField::DEFLATE_BIT_MASK;
	this->bzip2 = (intValue >> (SupportedCompressionBitField::BZIP2_START_BIT)) & SupportedCompressionBitField::BZIP2_BIT_MASK;
	this->lZMA = (intValue >> (SupportedCompressionBitField::LZMA_START_BIT)) & SupportedCompressionBitField::LZMA_BIT_MASK;

	return byteSize;
}

int SupportedCompressionBitField::length(void)
{
	return sizeof(uint8_t);
}

void SupportedCompressionBitField::copy(SupportedCompressionBitField& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	setNoCompression(source.getNoCompression());
	setDEFLATE(source.getDEFLATE());
	setBzip2(source.getBzip2());
	setLZMA(source.getLZMA());

}

std::string SupportedCompressionBitField::toXml(unsigned char level) const
{
	uint8_t intValue = 0;

	intValue |= ((this->noCompression & SupportedCompressionBitField::NOCOMPRESSION_BIT_MASK) << SupportedCompressionBitField::NOCOMPRESSION_START_BIT);
	intValue |= ((this->dEFLATE & SupportedCompressionBitField::DEFLATE_BIT_MASK) << SupportedCompressionBitField::DEFLATE_START_BIT);
	intValue |= ((this->bzip2 & SupportedCompressionBitField::BZIP2_BIT_MASK) << SupportedCompressionBitField::BZIP2_START_BIT);
	intValue |= ((this->lZMA & SupportedCompressionBitField::LZMA_BIT_MASK) << SupportedCompressionBitField::LZMA_START_BIT);

	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<BitField name=\"" << this->name << "\">\n";
	oss << prefix.str() << "\t" << "<intValue>" << intValue << "</intValue>\n";
	oss << prefix.str() << "\t" << "<fields>\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"NoCompression\" value=\"" << getNoCompression() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"DEFLATE\" value=\"" << getDEFLATE() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"bzip2\" value=\"" << getBzip2() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"LZMA\" value=\"" << getLZMA() << "\" />\n";
	oss << prefix.str() << "\t" << "</fields>\n";
	oss << prefix.str() << "</BitField>\n";
	return oss.str();
}

} // namespace environment
} // namespace openjaus

