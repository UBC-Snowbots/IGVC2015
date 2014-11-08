/**
\file ListManager.h

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

#ifndef LISTMANAGER_SERVICE_INTERFACE_H
#define LISTMANAGER_SERVICE_INTERFACE_H

#include <openjaus.h>
#include "openjaus/core/ManagementInterface.h"
#include "openjaus/mobility/Triggers/SetElement.h"
#include "openjaus/mobility/Triggers/DeleteElement.h"
#include "openjaus/mobility/Triggers/QueryElement.h"
#include "openjaus/mobility/Triggers/QueryElementList.h"
#include "openjaus/mobility/Triggers/QueryElementCount.h"
#include "openjaus/mobility/Triggers/ConfirmElementRequest.h"
#include "openjaus/mobility/Triggers/RejectElementRequest.h"
#include "openjaus/mobility/Triggers/ReportElement.h"
#include "openjaus/mobility/Triggers/ReportElementList.h"
#include "openjaus/mobility/Triggers/ReportElementCount.h"
namespace openjaus
{
namespace mobility
{

/// \class ListManagerInterface ListManagerInterface.h
/// \brief Provides an abstract interface for the %ListManager service. 
/// <p>
/// The List Manager Service permits operations on a single ordered sequence of connected elements. It supports
/// operations to add, replace or delete elements from the list, as well as querying the entire list or individual
/// elements. Elements within the list are uniquely identified by the Element UID. The Element UID is used as an
/// identifier only, and the value of the UID does not imply a sequence or order. When a new element is added to the
/// list, the previous (parent) and next (child) elements are specified to denote sequencing, similar to a doubly linked
/// list. Circular lists can be created when the last element in the list specifies the first element as a child. A list
/// is considered valid when the following conditions are met: 1) A list must contain exactly one head element which is
/// defined as having a previous (parent) identifier of zero (0). 2) For non-circular lists, the list must contain
/// exactly one tail element which is defined as having a next (child) identifier of zero (0). 3) Each element must
/// reference existing previous (parent) and next (child) elements, or zero. 4) Elements cannot be orphaned. An orphan
/// is defined as an element that is not connected in any way to the other elements in the list. 5) The previous
/// (parent) and next(child) reference for each element cannot point to itself. The list manager service is designed to
/// be inherited, and is trivial on its own. Derived services should redefine isElementSupported condition as shown by
/// example in the Global Waypoint List Driver.
/// </p><br/><br/>
/// <b>URI:</b> %urn:jaus:jss:mobility:ListManager<br/><br/>
/// <b>Version:</b> 1.0<br/><br/>
/// <b>Inherits From:</b><ul>
/// <li>urn:jaus:jss:core:Management</li>
/// </ul>
/// </dd>

class OPENJAUS_EXPORT ListManagerInterface
{
public:
	virtual ~ListManagerInterface(){};
	
	/// \brief Send a Report Element message with the requested element.
	/// Send a Report Element message with the requested element.
	/// \param[in] queryElement - Input Trigger.
	/// \return ReportElement Output Message.
	virtual ReportElement getReportElement(QueryElement *queryElement) = 0;

	/// \brief Send action for ReportElementList with input message QueryElementList.
	/// Send action for ReportElementList with input message QueryElementList.
	/// \param[in] queryElementList - Input Trigger.
	/// \return ReportElementList Output Message.
	virtual ReportElementList getReportElementList(QueryElementList *queryElementList) = 0;

	/// \brief Send action for ReportElementCount with input message QueryElementCount.
	/// Send action for ReportElementCount with input message QueryElementCount.
	/// \param[in] queryElementCount - Input Trigger.
	/// \return ReportElementCount Output Message.
	virtual ReportElementCount getReportElementCount(QueryElementCount *queryElementCount) = 0;

	/// \brief Send action for ConfirmElementRequest with input message SetElement.
	/// Send action for ConfirmElementRequest with input message SetElement.
	/// \param[in] setElement - Input Trigger.
	/// \return ConfirmElementRequest Output Message.
	virtual ConfirmElementRequest getConfirmElementRequest(SetElement *setElement) = 0;

	/// \brief Send action for ConfirmElementRequest with input message DeleteElement.
	/// Send action for ConfirmElementRequest with input message DeleteElement.
	/// \param[in] deleteElement - Input Trigger.
	/// \return ConfirmElementRequest Output Message.
	virtual ConfirmElementRequest getConfirmElementRequest(DeleteElement *deleteElement) = 0;

	/// \brief Send action for RejectElementRequest with input message SetElement.
	/// Send action for RejectElementRequest with input message SetElement.
	/// \param[in] setElement - Input Trigger.
	/// \return RejectElementRequest Output Message.
	virtual RejectElementRequest getRejectElementRequest(SetElement *setElement) = 0;

	/// \brief Send action for RejectElementRequest with input message DeleteElement.
	/// Send action for RejectElementRequest with input message DeleteElement.
	/// \param[in] deleteElement - Input Trigger.
	/// \return RejectElementRequest Output Message.
	virtual RejectElementRequest getRejectElementRequest(DeleteElement *deleteElement) = 0;

	/// \brief Store the element(s) in the list with sequence specified by the previous and next element IDs. If this action represents an insert or append into an existing list, the service should modify the NextUID of the previous element and/or the Previous UID of the next element to reflect the updated sequence.
	/// Store the element(s) in the list with sequence specified by the previous and next element IDs. If this action represents an insert or append into an existing list, the service should modify the NextUID of the previous element and/or the Previous UID of the next element to reflect the updated sequence.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool setElement(SetElement *setElement) = 0;

	/// \brief Remove the specified element(s) from the list. The service should modify the NextUID of the previous element and/or the Previous UID of the next element to reflect the updated sequence.
	/// Remove the specified element(s) from the list. The service should modify the NextUID of the previous element and/or the Previous UID of the next element to reflect the updated sequence.
	/// \param[in]  deleteElement - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool deleteElement(DeleteElement *deleteElement) = 0;

	/// \brief True if the UID(s) specified in the message that triggered the transition exists in the list.
	/// True if the UID(s) specified in the message that triggered the transition exists in the list.
	/// \param[in]  queryElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool elementExists(QueryElement *queryElement) = 0;

	/// \brief True if the UID(s) specified in the message that triggered the transition exists in the list.
	/// True if the UID(s) specified in the message that triggered the transition exists in the list.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool elementExists(SetElement *setElement) = 0;

	/// \brief True if the UID(s) specified in the message that triggered the transition exists in the list.
	/// True if the UID(s) specified in the message that triggered the transition exists in the list.
	/// \param[in]  deleteElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool elementExists(DeleteElement *deleteElement) = 0;

	/// \brief isControllingListClient condition.
	/// isControllingListClient condition.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingListClient(SetElement *setElement) = 0;

	/// \brief isControllingListClient condition.
	/// isControllingListClient condition.
	/// \param[in]  deleteElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingListClient(DeleteElement *deleteElement) = 0;

	/// \brief True if the resulting list will not be invalid as defined by the List Manager Service description and the receiving entity has sufficient memory to store the element(s).
	/// True if the resulting list will not be invalid as defined by the List Manager Service description and the receiving entity has sufficient memory to store the element(s).
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isValidElementRequest(SetElement *setElement) = 0;

	/// \brief False. This condition must be overridden by derived services to allow the list to be populated.
	/// False. This condition must be overridden by derived services to allow the list to be populated.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isElementSupported(SetElement *setElement) = 0;

};

} // namespace mobility
} // namespace openjaus

#endif // LISTMANAGER_SERVICE_INTERFACE_H
