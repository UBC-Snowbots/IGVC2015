
/**
\file ManipulatorCoordinateSystemRec.h

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

#ifndef MANIPULATORCOORDINATESYSTEMRECORD_H
#define MANIPULATORCOORDINATESYSTEMRECORD_H

#include <openjaus.h>
#include "openjaus/manipulator/Triggers/Fields/ManipulatorCoordinateSysXScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/ManipulatorCoordinateSysYScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/ManipulatorCoordinateSysZScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/DComponentOfUnitQuaternionQScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/AComponentOfUnitQuaternionQScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/BComponentOfUnitQuaternionQScaledInteger.h"
#include "openjaus/manipulator/Triggers/Fields/CComponentOfUnitQuaternionQScaledInteger.h"

namespace openjaus
{
namespace manipulator
{

class OPENJAUS_EXPORT ManipulatorCoordinateSystemRecord : public openjaus::model::fields::Record
{
public:
	ManipulatorCoordinateSystemRecord();
	ManipulatorCoordinateSystemRecord(const ManipulatorCoordinateSystemRecord &source);
	~ManipulatorCoordinateSystemRecord();

	void copy(ManipulatorCoordinateSystemRecord& source);
	virtual int to(system::Buffer *dst);
	virtual int from(system::Buffer *src);
	virtual int length(void);
	std::string toXml(unsigned char level=0) const;
	

	double getManipulatorCoordinateSysX_m(void);
	void setManipulatorCoordinateSysX_m(double value);

	double getManipulatorCoordinateSysY_m(void);
	void setManipulatorCoordinateSysY_m(double value);

	double getManipulatorCoordinateSysZ_m(void);
	void setManipulatorCoordinateSysZ_m(double value);

	double getDComponentOfUnitQuaternionQ(void);
	void setDComponentOfUnitQuaternionQ(double value);

	double getAComponentOfUnitQuaternionQ(void);
	void setAComponentOfUnitQuaternionQ(double value);

	double getBComponentOfUnitQuaternionQ(void);
	void setBComponentOfUnitQuaternionQ(double value);

	double getCComponentOfUnitQuaternionQ(void);
	void setCComponentOfUnitQuaternionQ(double value);

protected:
	ManipulatorCoordinateSysXScaledInteger manipulatorCoordinateSysX_m;
	ManipulatorCoordinateSysYScaledInteger manipulatorCoordinateSysY_m;
	ManipulatorCoordinateSysZScaledInteger manipulatorCoordinateSysZ_m;
	DComponentOfUnitQuaternionQScaledInteger dComponentOfUnitQuaternionQ;
	AComponentOfUnitQuaternionQScaledInteger aComponentOfUnitQuaternionQ;
	BComponentOfUnitQuaternionQScaledInteger bComponentOfUnitQuaternionQ;
	CComponentOfUnitQuaternionQScaledInteger cComponentOfUnitQuaternionQ;

};

} // namespace manipulator
} // namespace openjaus

#endif // MANIPULATORCOORDINATESYSTEMRECORD_H

