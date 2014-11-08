

/**
\file SupportedAnalogFormatsBitField.h

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
#include "openjaus/environment/Triggers/Fields/SupportedAnalogFormatsBitField.h"

namespace openjaus
{
namespace environment
{

SupportedAnalogFormatsBitField::SupportedAnalogFormatsBitField() : 
	nTSCM(),
	nTSCJ(),
	pALN(),
	pALM(),
	sECAML(),
	sECAMBG()
{

}

SupportedAnalogFormatsBitField::~SupportedAnalogFormatsBitField()
{

}

bool SupportedAnalogFormatsBitField::setNTSCM(bool value)
{
	this->nTSCM = value;
	return true;
}

bool SupportedAnalogFormatsBitField::getNTSCM(void) const
{
	return this->nTSCM;
}


bool SupportedAnalogFormatsBitField::setNTSCJ(bool value)
{
	this->nTSCJ = value;
	return true;
}

bool SupportedAnalogFormatsBitField::getNTSCJ(void) const
{
	return this->nTSCJ;
}


bool SupportedAnalogFormatsBitField::setPALN(bool value)
{
	this->pALN = value;
	return true;
}

bool SupportedAnalogFormatsBitField::getPALN(void) const
{
	return this->pALN;
}


bool SupportedAnalogFormatsBitField::setPALM(bool value)
{
	this->pALM = value;
	return true;
}

bool SupportedAnalogFormatsBitField::getPALM(void) const
{
	return this->pALM;
}


bool SupportedAnalogFormatsBitField::setSECAML(bool value)
{
	this->sECAML = value;
	return true;
}

bool SupportedAnalogFormatsBitField::getSECAML(void) const
{
	return this->sECAML;
}


bool SupportedAnalogFormatsBitField::setSECAMBG(bool value)
{
	this->sECAMBG = value;
	return true;
}

bool SupportedAnalogFormatsBitField::getSECAMBG(void) const
{
	return this->sECAMBG;
}



int SupportedAnalogFormatsBitField::to(system::Buffer *dst)
{
	uint8_t intValue = 0;

	intValue |= ((this->nTSCM & SupportedAnalogFormatsBitField::NTSCM_BIT_MASK) << SupportedAnalogFormatsBitField::NTSCM_START_BIT);
	intValue |= ((this->nTSCJ & SupportedAnalogFormatsBitField::NTSCJ_BIT_MASK) << SupportedAnalogFormatsBitField::NTSCJ_START_BIT);
	intValue |= ((this->pALN & SupportedAnalogFormatsBitField::PALN_BIT_MASK) << SupportedAnalogFormatsBitField::PALN_START_BIT);
	intValue |= ((this->pALM & SupportedAnalogFormatsBitField::PALM_BIT_MASK) << SupportedAnalogFormatsBitField::PALM_START_BIT);
	intValue |= ((this->sECAML & SupportedAnalogFormatsBitField::SECAML_BIT_MASK) << SupportedAnalogFormatsBitField::SECAML_START_BIT);
	intValue |= ((this->sECAMBG & SupportedAnalogFormatsBitField::SECAMBG_BIT_MASK) << SupportedAnalogFormatsBitField::SECAMBG_START_BIT);
	return dst->pack(intValue);
}

int SupportedAnalogFormatsBitField::from(system::Buffer *src)
{
	int byteSize = 0;
	uint8_t intValue = 0;
	byteSize = src->unpack(intValue);

	this->nTSCM = (intValue >> (SupportedAnalogFormatsBitField::NTSCM_START_BIT)) & SupportedAnalogFormatsBitField::NTSCM_BIT_MASK;
	this->nTSCJ = (intValue >> (SupportedAnalogFormatsBitField::NTSCJ_START_BIT)) & SupportedAnalogFormatsBitField::NTSCJ_BIT_MASK;
	this->pALN = (intValue >> (SupportedAnalogFormatsBitField::PALN_START_BIT)) & SupportedAnalogFormatsBitField::PALN_BIT_MASK;
	this->pALM = (intValue >> (SupportedAnalogFormatsBitField::PALM_START_BIT)) & SupportedAnalogFormatsBitField::PALM_BIT_MASK;
	this->sECAML = (intValue >> (SupportedAnalogFormatsBitField::SECAML_START_BIT)) & SupportedAnalogFormatsBitField::SECAML_BIT_MASK;
	this->sECAMBG = (intValue >> (SupportedAnalogFormatsBitField::SECAMBG_START_BIT)) & SupportedAnalogFormatsBitField::SECAMBG_BIT_MASK;

	return byteSize;
}

int SupportedAnalogFormatsBitField::length(void)
{
	return sizeof(uint8_t);
}

void SupportedAnalogFormatsBitField::copy(SupportedAnalogFormatsBitField& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	setNTSCM(source.getNTSCM());
	setNTSCJ(source.getNTSCJ());
	setPALN(source.getPALN());
	setPALM(source.getPALM());
	setSECAML(source.getSECAML());
	setSECAMBG(source.getSECAMBG());

}

std::string SupportedAnalogFormatsBitField::toXml(unsigned char level) const
{
	uint8_t intValue = 0;

	intValue |= ((this->nTSCM & SupportedAnalogFormatsBitField::NTSCM_BIT_MASK) << SupportedAnalogFormatsBitField::NTSCM_START_BIT);
	intValue |= ((this->nTSCJ & SupportedAnalogFormatsBitField::NTSCJ_BIT_MASK) << SupportedAnalogFormatsBitField::NTSCJ_START_BIT);
	intValue |= ((this->pALN & SupportedAnalogFormatsBitField::PALN_BIT_MASK) << SupportedAnalogFormatsBitField::PALN_START_BIT);
	intValue |= ((this->pALM & SupportedAnalogFormatsBitField::PALM_BIT_MASK) << SupportedAnalogFormatsBitField::PALM_START_BIT);
	intValue |= ((this->sECAML & SupportedAnalogFormatsBitField::SECAML_BIT_MASK) << SupportedAnalogFormatsBitField::SECAML_START_BIT);
	intValue |= ((this->sECAMBG & SupportedAnalogFormatsBitField::SECAMBG_BIT_MASK) << SupportedAnalogFormatsBitField::SECAMBG_START_BIT);

	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<BitField name=\"" << this->name << "\">\n";
	oss << prefix.str() << "\t" << "<intValue>" << intValue << "</intValue>\n";
	oss << prefix.str() << "\t" << "<fields>\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"NTSCM\" value=\"" << getNTSCM() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"NTSCJ\" value=\"" << getNTSCJ() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"PALN\" value=\"" << getPALN() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"PALM\" value=\"" << getPALM() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"SECAML\" value=\"" << getSECAML() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"SECAMBG\" value=\"" << getSECAMBG() << "\" />\n";
	oss << prefix.str() << "\t" << "</fields>\n";
	oss << prefix.str() << "</BitField>\n";
	return oss.str();
}

} // namespace environment
} // namespace openjaus

