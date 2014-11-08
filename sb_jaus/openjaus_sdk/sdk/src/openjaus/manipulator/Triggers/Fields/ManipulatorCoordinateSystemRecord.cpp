/**
\file ManipulatorCoordinateSystemRecord.h

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
#include "openjaus/manipulator/Triggers/Fields/ManipulatorCoordinateSystemRecord.h"

namespace openjaus
{
namespace manipulator
{

ManipulatorCoordinateSystemRecord::ManipulatorCoordinateSystemRecord():
	manipulatorCoordinateSysX_m(),
	manipulatorCoordinateSysY_m(),
	manipulatorCoordinateSysZ_m(),
	dComponentOfUnitQuaternionQ(),
	aComponentOfUnitQuaternionQ(),
	bComponentOfUnitQuaternionQ(),
	cComponentOfUnitQuaternionQ()
{

	fields.push_back(&manipulatorCoordinateSysX_m);
	manipulatorCoordinateSysX_m.setName("ManipulatorCoordinateSysX");
	manipulatorCoordinateSysX_m.setOptional(false);
	manipulatorCoordinateSysX_m.setInterpretation("x coordinate of origin of manipulator coordinate system measured with respect to vehicle coordinate system");
	// Nothing to init

	fields.push_back(&manipulatorCoordinateSysY_m);
	manipulatorCoordinateSysY_m.setName("ManipulatorCoordinateSysY");
	manipulatorCoordinateSysY_m.setOptional(false);
	manipulatorCoordinateSysY_m.setInterpretation("y coordinate of origin of manipulator coordinate system measured with respect to vehicle coordinate system");
	// Nothing to init

	fields.push_back(&manipulatorCoordinateSysZ_m);
	manipulatorCoordinateSysZ_m.setName("ManipulatorCoordinateSysZ");
	manipulatorCoordinateSysZ_m.setOptional(false);
	manipulatorCoordinateSysZ_m.setInterpretation("z coordinate of origin of manipulator coordinate system measured with respect to vehicle coordinate system");
	// Nothing to init

	fields.push_back(&dComponentOfUnitQuaternionQ);
	dComponentOfUnitQuaternionQ.setName("DComponentOfUnitQuaternionQ");
	dComponentOfUnitQuaternionQ.setOptional(false);
	dComponentOfUnitQuaternionQ.setInterpretation("quaternion q = d + ai +bj + ck defines the orientation of the manipulator coordinate system measured with respect to the vehicle coordinate system");
	// Nothing to init

	fields.push_back(&aComponentOfUnitQuaternionQ);
	aComponentOfUnitQuaternionQ.setName("AComponentOfUnitQuaternionQ");
	aComponentOfUnitQuaternionQ.setOptional(false);
	aComponentOfUnitQuaternionQ.setInterpretation("quaternion q = d + ai +bj + ck defines the orientation of the manipulator coordinate system measured with respect to the vehicle coordinate system");
	// Nothing to init

	fields.push_back(&bComponentOfUnitQuaternionQ);
	bComponentOfUnitQuaternionQ.setName("BComponentOfUnitQuaternionQ");
	bComponentOfUnitQuaternionQ.setOptional(false);
	bComponentOfUnitQuaternionQ.setInterpretation("quaternion q = d + ai +bj + ck defines the orientation of the manipulator coordinate system measured with respect to the vehicle coordinate system");
	// Nothing to init

	fields.push_back(&cComponentOfUnitQuaternionQ);
	cComponentOfUnitQuaternionQ.setName("CComponentOfUnitQuaternionQ");
	cComponentOfUnitQuaternionQ.setOptional(false);
	cComponentOfUnitQuaternionQ.setInterpretation("quaternion q = d + ai +bj + ck defines the orientation of the manipulator coordinate system measured with respect to the vehicle coordinate system");
	// Nothing to init

}

ManipulatorCoordinateSystemRecord::ManipulatorCoordinateSystemRecord(const ManipulatorCoordinateSystemRecord &source)
{
	this->copy(const_cast<ManipulatorCoordinateSystemRecord&>(source));
}

ManipulatorCoordinateSystemRecord::~ManipulatorCoordinateSystemRecord()
{

}


double ManipulatorCoordinateSystemRecord::getManipulatorCoordinateSysX_m(void)
{
	return this->manipulatorCoordinateSysX_m.getValue();
}

void ManipulatorCoordinateSystemRecord::setManipulatorCoordinateSysX_m(double value)
{
	this->manipulatorCoordinateSysX_m.setValue(value);
}

double ManipulatorCoordinateSystemRecord::getManipulatorCoordinateSysY_m(void)
{
	return this->manipulatorCoordinateSysY_m.getValue();
}

void ManipulatorCoordinateSystemRecord::setManipulatorCoordinateSysY_m(double value)
{
	this->manipulatorCoordinateSysY_m.setValue(value);
}

double ManipulatorCoordinateSystemRecord::getManipulatorCoordinateSysZ_m(void)
{
	return this->manipulatorCoordinateSysZ_m.getValue();
}

void ManipulatorCoordinateSystemRecord::setManipulatorCoordinateSysZ_m(double value)
{
	this->manipulatorCoordinateSysZ_m.setValue(value);
}

double ManipulatorCoordinateSystemRecord::getDComponentOfUnitQuaternionQ(void)
{
	return this->dComponentOfUnitQuaternionQ.getValue();
}

void ManipulatorCoordinateSystemRecord::setDComponentOfUnitQuaternionQ(double value)
{
	this->dComponentOfUnitQuaternionQ.setValue(value);
}

double ManipulatorCoordinateSystemRecord::getAComponentOfUnitQuaternionQ(void)
{
	return this->aComponentOfUnitQuaternionQ.getValue();
}

void ManipulatorCoordinateSystemRecord::setAComponentOfUnitQuaternionQ(double value)
{
	this->aComponentOfUnitQuaternionQ.setValue(value);
}

double ManipulatorCoordinateSystemRecord::getBComponentOfUnitQuaternionQ(void)
{
	return this->bComponentOfUnitQuaternionQ.getValue();
}

void ManipulatorCoordinateSystemRecord::setBComponentOfUnitQuaternionQ(double value)
{
	this->bComponentOfUnitQuaternionQ.setValue(value);
}

double ManipulatorCoordinateSystemRecord::getCComponentOfUnitQuaternionQ(void)
{
	return this->cComponentOfUnitQuaternionQ.getValue();
}

void ManipulatorCoordinateSystemRecord::setCComponentOfUnitQuaternionQ(double value)
{
	this->cComponentOfUnitQuaternionQ.setValue(value);
}

int ManipulatorCoordinateSystemRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(manipulatorCoordinateSysX_m);
	byteSize += dst->pack(manipulatorCoordinateSysY_m);
	byteSize += dst->pack(manipulatorCoordinateSysZ_m);
	byteSize += dst->pack(dComponentOfUnitQuaternionQ);
	byteSize += dst->pack(aComponentOfUnitQuaternionQ);
	byteSize += dst->pack(bComponentOfUnitQuaternionQ);
	byteSize += dst->pack(cComponentOfUnitQuaternionQ);
	return byteSize;
}
int ManipulatorCoordinateSystemRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(manipulatorCoordinateSysX_m);
	byteSize += src->unpack(manipulatorCoordinateSysY_m);
	byteSize += src->unpack(manipulatorCoordinateSysZ_m);
	byteSize += src->unpack(dComponentOfUnitQuaternionQ);
	byteSize += src->unpack(aComponentOfUnitQuaternionQ);
	byteSize += src->unpack(bComponentOfUnitQuaternionQ);
	byteSize += src->unpack(cComponentOfUnitQuaternionQ);
	return byteSize;
}

int ManipulatorCoordinateSystemRecord::length(void)
{
	int length = 0;
	length += manipulatorCoordinateSysX_m.length(); // manipulatorCoordinateSysX_m
	length += manipulatorCoordinateSysY_m.length(); // manipulatorCoordinateSysY_m
	length += manipulatorCoordinateSysZ_m.length(); // manipulatorCoordinateSysZ_m
	length += dComponentOfUnitQuaternionQ.length(); // dComponentOfUnitQuaternionQ
	length += aComponentOfUnitQuaternionQ.length(); // aComponentOfUnitQuaternionQ
	length += bComponentOfUnitQuaternionQ.length(); // bComponentOfUnitQuaternionQ
	length += cComponentOfUnitQuaternionQ.length(); // cComponentOfUnitQuaternionQ
	return length;
}

std::string ManipulatorCoordinateSystemRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"ManipulatorCoordinateSystemRecord\">\n";
	oss << manipulatorCoordinateSysX_m.toXml(level+1); // manipulatorCoordinateSysX_m
	oss << manipulatorCoordinateSysY_m.toXml(level+1); // manipulatorCoordinateSysY_m
	oss << manipulatorCoordinateSysZ_m.toXml(level+1); // manipulatorCoordinateSysZ_m
	oss << dComponentOfUnitQuaternionQ.toXml(level+1); // dComponentOfUnitQuaternionQ
	oss << aComponentOfUnitQuaternionQ.toXml(level+1); // aComponentOfUnitQuaternionQ
	oss << bComponentOfUnitQuaternionQ.toXml(level+1); // bComponentOfUnitQuaternionQ
	oss << cComponentOfUnitQuaternionQ.toXml(level+1); // cComponentOfUnitQuaternionQ
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}



void ManipulatorCoordinateSystemRecord::copy(ManipulatorCoordinateSystemRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->manipulatorCoordinateSysX_m.setName("ManipulatorCoordinateSysX");
	this->manipulatorCoordinateSysX_m.setOptional(false);
	this->manipulatorCoordinateSysX_m.setInterpretation("x coordinate of origin of manipulator coordinate system measured with respect to vehicle coordinate system");
	this->manipulatorCoordinateSysX_m.setValue(source.getManipulatorCoordinateSysX_m()); 
 
	this->manipulatorCoordinateSysY_m.setName("ManipulatorCoordinateSysY");
	this->manipulatorCoordinateSysY_m.setOptional(false);
	this->manipulatorCoordinateSysY_m.setInterpretation("y coordinate of origin of manipulator coordinate system measured with respect to vehicle coordinate system");
	this->manipulatorCoordinateSysY_m.setValue(source.getManipulatorCoordinateSysY_m()); 
 
	this->manipulatorCoordinateSysZ_m.setName("ManipulatorCoordinateSysZ");
	this->manipulatorCoordinateSysZ_m.setOptional(false);
	this->manipulatorCoordinateSysZ_m.setInterpretation("z coordinate of origin of manipulator coordinate system measured with respect to vehicle coordinate system");
	this->manipulatorCoordinateSysZ_m.setValue(source.getManipulatorCoordinateSysZ_m()); 
 
	this->dComponentOfUnitQuaternionQ.setName("DComponentOfUnitQuaternionQ");
	this->dComponentOfUnitQuaternionQ.setOptional(false);
	this->dComponentOfUnitQuaternionQ.setInterpretation("quaternion q = d + ai +bj + ck defines the orientation of the manipulator coordinate system measured with respect to the vehicle coordinate system");
	this->dComponentOfUnitQuaternionQ.setValue(source.getDComponentOfUnitQuaternionQ()); 
 
	this->aComponentOfUnitQuaternionQ.setName("AComponentOfUnitQuaternionQ");
	this->aComponentOfUnitQuaternionQ.setOptional(false);
	this->aComponentOfUnitQuaternionQ.setInterpretation("quaternion q = d + ai +bj + ck defines the orientation of the manipulator coordinate system measured with respect to the vehicle coordinate system");
	this->aComponentOfUnitQuaternionQ.setValue(source.getAComponentOfUnitQuaternionQ()); 
 
	this->bComponentOfUnitQuaternionQ.setName("BComponentOfUnitQuaternionQ");
	this->bComponentOfUnitQuaternionQ.setOptional(false);
	this->bComponentOfUnitQuaternionQ.setInterpretation("quaternion q = d + ai +bj + ck defines the orientation of the manipulator coordinate system measured with respect to the vehicle coordinate system");
	this->bComponentOfUnitQuaternionQ.setValue(source.getBComponentOfUnitQuaternionQ()); 
 
	this->cComponentOfUnitQuaternionQ.setName("CComponentOfUnitQuaternionQ");
	this->cComponentOfUnitQuaternionQ.setOptional(false);
	this->cComponentOfUnitQuaternionQ.setInterpretation("quaternion q = d + ai +bj + ck defines the orientation of the manipulator coordinate system measured with respect to the vehicle coordinate system");
	this->cComponentOfUnitQuaternionQ.setValue(source.getCComponentOfUnitQuaternionQ()); 
 
}

} // namespace manipulator
} // namespace openjaus

