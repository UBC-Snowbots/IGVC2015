

/**
\file MeteringModesBitField.h

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
#include "openjaus/environment/Triggers/Fields/MeteringModesBitField.h"

namespace openjaus
{
namespace environment
{

MeteringModesBitField::MeteringModesBitField() : 
	matrixOrAuto(),
	centerWeighted(),
	spot()
{

}

MeteringModesBitField::~MeteringModesBitField()
{

}

bool MeteringModesBitField::setMatrixOrAuto(bool value)
{
	this->matrixOrAuto = value;
	return true;
}

bool MeteringModesBitField::getMatrixOrAuto(void) const
{
	return this->matrixOrAuto;
}


bool MeteringModesBitField::setCenterWeighted(bool value)
{
	this->centerWeighted = value;
	return true;
}

bool MeteringModesBitField::getCenterWeighted(void) const
{
	return this->centerWeighted;
}


bool MeteringModesBitField::setSpot(bool value)
{
	this->spot = value;
	return true;
}

bool MeteringModesBitField::getSpot(void) const
{
	return this->spot;
}



int MeteringModesBitField::to(system::Buffer *dst)
{
	uint8_t intValue = 0;

	intValue |= ((this->matrixOrAuto & MeteringModesBitField::MATRIXORAUTO_BIT_MASK) << MeteringModesBitField::MATRIXORAUTO_START_BIT);
	intValue |= ((this->centerWeighted & MeteringModesBitField::CENTERWEIGHTED_BIT_MASK) << MeteringModesBitField::CENTERWEIGHTED_START_BIT);
	intValue |= ((this->spot & MeteringModesBitField::SPOT_BIT_MASK) << MeteringModesBitField::SPOT_START_BIT);
	return dst->pack(intValue);
}

int MeteringModesBitField::from(system::Buffer *src)
{
	int byteSize = 0;
	uint8_t intValue = 0;
	byteSize = src->unpack(intValue);

	this->matrixOrAuto = (intValue >> (MeteringModesBitField::MATRIXORAUTO_START_BIT)) & MeteringModesBitField::MATRIXORAUTO_BIT_MASK;
	this->centerWeighted = (intValue >> (MeteringModesBitField::CENTERWEIGHTED_START_BIT)) & MeteringModesBitField::CENTERWEIGHTED_BIT_MASK;
	this->spot = (intValue >> (MeteringModesBitField::SPOT_START_BIT)) & MeteringModesBitField::SPOT_BIT_MASK;

	return byteSize;
}

int MeteringModesBitField::length(void)
{
	return sizeof(uint8_t);
}

void MeteringModesBitField::copy(MeteringModesBitField& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	setMatrixOrAuto(source.getMatrixOrAuto());
	setCenterWeighted(source.getCenterWeighted());
	setSpot(source.getSpot());

}

std::string MeteringModesBitField::toXml(unsigned char level) const
{
	uint8_t intValue = 0;

	intValue |= ((this->matrixOrAuto & MeteringModesBitField::MATRIXORAUTO_BIT_MASK) << MeteringModesBitField::MATRIXORAUTO_START_BIT);
	intValue |= ((this->centerWeighted & MeteringModesBitField::CENTERWEIGHTED_BIT_MASK) << MeteringModesBitField::CENTERWEIGHTED_START_BIT);
	intValue |= ((this->spot & MeteringModesBitField::SPOT_BIT_MASK) << MeteringModesBitField::SPOT_START_BIT);

	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<BitField name=\"" << this->name << "\">\n";
	oss << prefix.str() << "\t" << "<intValue>" << intValue << "</intValue>\n";
	oss << prefix.str() << "\t" << "<fields>\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"MatrixOrAuto\" value=\"" << getMatrixOrAuto() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"CenterWeighted\" value=\"" << getCenterWeighted() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"Spot\" value=\"" << getSpot() << "\" />\n";
	oss << prefix.str() << "\t" << "</fields>\n";
	oss << prefix.str() << "</BitField>\n";
	return oss.str();
}

} // namespace environment
} // namespace openjaus

