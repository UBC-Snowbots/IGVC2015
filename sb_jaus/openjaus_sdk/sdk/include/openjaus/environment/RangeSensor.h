/**
\file RangeSensor.h

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

#ifndef RANGESENSOR_COMPONENT_H
#define RANGESENSOR_COMPONENT_H

#include <openjaus.h>
#include "openjaus/core/Base.h"
#include "openjaus/environment/RangeSensorInterface.h"
#include "openjaus/environment/Transitions/RangeSensorDefaultLoop.h"
#include "openjaus/environment/Transitions/RangeSensorControlledLoop.h"
#include "openjaus/environment/Triggers/SetRangeSensorConfiguration.h"
#include "openjaus/environment/Triggers/QuerySensorGeometricProperties.h"
#include "openjaus/environment/Triggers/QueryRangeSensorCompressedData.h"
#include "openjaus/environment/Triggers/QueryRangeSensorData.h"
#include "openjaus/environment/Triggers/QueryRangeSensorCapabilities.h"
#include "openjaus/environment/Triggers/QueryRangeSensorConfiguration.h"
#include "openjaus/environment/Triggers/ConfirmRangeSensorConfiguration.h"
#include "openjaus/environment/Triggers/ReportRangeSensorCapabilities.h"
#include "openjaus/environment/Triggers/ReportRangeSensorConfiguration.h"
#include "openjaus/environment/Triggers/ReportRangeSensorData.h"
#include "openjaus/environment/Triggers/ReportRangeSensorCompressedData.h"
#include "openjaus/environment/Triggers/ReportRangeSensorGeometricProperties.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace environment
{

/// \class RangeSensor RangeSensor.h
/// \brief %RangeSensor Component implements the urn:jaus:jss:environmentSensing:RangeSensor services.
/// The %RangeSensor component provides an implementation of the following service(s). This component can 
/// be extended for customized functionality or used via instantiation.
/// <dl>
/// <dt>%RangeSensor Service</dt>
/// <dd><p>
/// The function of the Range Sensor Service is to provide information from proximity sensors. This service will output
/// the location of various Data Points with a certain measure of accuracy. A given Range Sensor service may be
/// comprised of one to many actual physical sensors or technologies. Each sub-sensor can be assigned (by the developer)
/// a unique Sensor ID. When appropriate, the reserved Sensor ID of 0 may be used to refer to all sensors attached to a
/// given Range Sensor Service. The Data Points are measured in the sensor’s native coordinate system and are expressed
/// in terms of range, bearing and inclination. Range is the distance, in meters, along the line from the origin of the
/// sensor’s native coordinate system (sensor’s origin) to the specified point. Bearing is the angle, in radians, that
/// the line from the sensor’s origin to the specified point makes about the sensor’s z-axis in the right handed sense
/// (Figure 2). Inclination is the the angle, in radians, that the line from the sensor origin to the specified point
/// makes about the sensor’s y-axis in the right handed sense (Figure 2).  Each data point has an optional ID parameter.
/// This parameter is provided for those sensor technologies which may assign and/or track entities based on unique ID
/// values; however, such tracking capabilities are not required for a compliant Range Sensor Service. The behavior of
/// the data point ID is not specified, i.e. IDs may repeat in a given report and IDs may persist from one report to
/// another. No semantic value should be placed on the ID values in a generalized way. Data Point ID behavior should be
/// derived from the underlying sensor or algorithm technology and is mearly provided to be used in those situations
/// where mutiple parties can agree upon the behavior and semantics of the ID values. Data from the range sensor can be
/// reported in both a compressed and uncompressed format, different query and report messages are provided for each
/// exchange and the kind of data compression supported by the service is reported in the Report Range Sensor
/// Capabilities message. Requests for unsupported data compression algorithms will result in the generation of a Report
/// Sensor Error message indicating an unsupported compression request. The range sensor can express the bearing,
/// inclination and range terms with respect to either its native coordinate system or the vehicle coordinate system if
/// coordinate transforms are supported. The Query Sensor Geometric Properties message can be used to determine the
/// geometric relationship between the sensor and the vehicle coordinate system. Three possible coordinate responses are
/// possible: (a) the service does not know the sensor’s position, (b) the sensor coordinate system is fixed with
/// respect to the vehicle and (c) the sensor is attached to some manipulator. These cases are supported by the Report
/// Sensor Geometric Properties message and are described therein.
/// </p><br/><br/>
/// <b>URI:</b> urn:jaus:jss:environmentSensing:RangeSensor<br/><br/>
/// <b>Version:</b> 1.0<br/>
/// <dl><dt><b>Inherits From:</b></dt>
/// <dd>urn:jaus:jss:core:AccessControl</dd>
/// </dl></dd>
/// </dl>
class OPENJAUS_EXPORT RangeSensor : public core::Base, public environment::RangeSensorInterface
{

public:
	RangeSensor();
	virtual ~RangeSensor();

	/// \brief Send a ReportRangeSensorData message in native coordinate system
	/// Send a ReportRangeSensorData message in native coordinate system
	/// \param[in] queryRangeSensorData - Input Trigger.
	/// \return ReportRangeSensorData Output Message.
	virtual ReportRangeSensorData getReportRangeSensorData(QueryRangeSensorData *queryRangeSensorData);

	/// \brief Send a ReportRangeSensorGeometricProperties message
	/// Send a ReportRangeSensorGeometricProperties message
	/// \param[in] querySensorGeometricProperties - Input Trigger.
	/// \return ReportRangeSensorGeometricProperties Output Message.
	virtual ReportRangeSensorGeometricProperties getReportRangeSensorGeometricProperties(QuerySensorGeometricProperties *querySensorGeometricProperties);

	/// \brief Update the sensor user controllable configuration parameters according to the ones specified.
	/// Update the sensor user controllable configuration parameters according to the ones specified.
	/// \param[in]  setRangeSensorConfiguration - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool updateRangeSensorConfiguration(SetRangeSensorConfiguration *setRangeSensorConfiguration);

	/// \brief Send sendConfirmRangeSensorConfiguration message
	/// Send sendConfirmRangeSensorConfiguration message
	/// \param[in] setRangeSensorConfiguration - Input Trigger.
	/// \return ConfirmRangeSensorConfiguration Output Message.
	virtual ConfirmRangeSensorConfiguration getConfirmRangeSensorConfiguration(SetRangeSensorConfiguration *setRangeSensorConfiguration);

	/// \brief Send a ReportRangeSensorConfiguration message
	/// Send a ReportRangeSensorConfiguration message
	/// \param[in] queryRangeSensorConfiguration - Input Trigger.
	/// \return ReportRangeSensorConfiguration Output Message.
	virtual ReportRangeSensorConfiguration getReportRangeSensorConfiguration(QueryRangeSensorConfiguration *queryRangeSensorConfiguration);

	/// \brief Send a ReportRangeSensorCompressedData message in native coordinate system
	/// Send a ReportRangeSensorCompressedData message in native coordinate system
	/// \param[in] queryRangeSensorCompressedData - Input Trigger.
	/// \return ReportRangeSensorCompressedData Output Message.
	virtual ReportRangeSensorCompressedData getReportRangeSensorCompressedData(QueryRangeSensorCompressedData *queryRangeSensorCompressedData);

	/// \brief Send a Report Range Sensor Capabilities message
	/// Send a Report Range Sensor Capabilities message
	/// \param[in] queryRangeSensorCapabilities - Input Trigger.
	/// \return ReportRangeSensorCapabilities Output Message.
	virtual ReportRangeSensorCapabilities getReportRangeSensorCapabilities(QueryRangeSensorCapabilities *queryRangeSensorCapabilities);

	/// \brief Send a ReportRangeSensorData message in the native coordinate system
	/// Send a ReportRangeSensorData message in the native coordinate system
	/// \param[in] queryRangeSensorData - Input Trigger.
	/// \return ReportRangeSensorData Output Message.
	virtual ReportRangeSensorData getReportRangeSensorDataInNativeSystem(QueryRangeSensorData *queryRangeSensorData);

	/// \brief Send a ReportRangeSensorCompressedData message using the requested coordinate system
	/// Send a ReportRangeSensorCompressedData message using the requested coordinate system
	/// \param[in] queryRangeSensorCompressedData - Input Trigger.
	/// \return ReportRangeSensorCompressedData Output Message.
	virtual ReportRangeSensorCompressedData getReportRangeSensorCompressedDataInNativeSystem(QueryRangeSensorCompressedData *queryRangeSensorCompressedData);


	/// \brief True if the service supports coordinate system transformations for data reports.
	/// True if the service supports coordinate system transformations for data reports.
	/// \param[in]  queryRangeSensorData - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isCoordinateTransformSupported(QueryRangeSensorData *queryRangeSensorData);

	/// \brief True if the service supports coordinate system transformations for data reports.
	/// True if the service supports coordinate system transformations for data reports.
	/// \param[in]  queryRangeSensorCompressedData - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isCoordinateTransformSupported(QueryRangeSensorCompressedData *queryRangeSensorCompressedData);


	/// \brief isControllingRangeSensorClient condition.
	/// isControllingRangeSensorClient condition.
	/// \param[in]  trigger - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingRangeSensorClient(model::Trigger *trigger);


	// Start of user code for additional methods:
	// End of user code

protected:
	RangeSensorDefaultLoop rangeSensorDefaultLoop;
	RangeSensorControlledLoop rangeSensorControlledLoop;


	// Start of user code for additional members:
	// End of user code
};

} // namespace component
} // namespace openjaus

#endif // RANGESENSOR_H
