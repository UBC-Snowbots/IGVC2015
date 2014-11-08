/**
\file ListManaged.h

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

#ifndef LISTMANAGED_COMPONENT_H
#define LISTMANAGED_COMPONENT_H

#include <openjaus.h>
#include "openjaus/core/Managed.h"
#include "openjaus/mobility/ListManagerInterface.h"
#include "openjaus/mobility/Transitions/ListManagerDefaultLoop.h"
#include "openjaus/mobility/Transitions/ListManagerControlledLoop.h"
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
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace mobility
{

/// \class ListManaged ListManaged.h
/// \brief %ListManaged Component implements the urn:jaus:jss:mobility:ListManager services.
/// The %ListManaged component provides an implementation of the following service(s). This component can 
/// be extended for customized functionality or used via instantiation.
/// <dl>
/// <dt>%ListManager Service</dt>
/// <dd><p>
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
/// <b>URI:</b> urn:jaus:jss:mobility:ListManager<br/><br/>
/// <b>Version:</b> 1.0<br/>
/// <dl><dt><b>Inherits From:</b></dt>
/// <dd>urn:jaus:jss:core:Management</dd>
/// </dl></dd>
/// </dl>
class OPENJAUS_EXPORT ListManaged : public core::Managed, public mobility::ListManagerInterface
{

public:
	ListManaged();
	virtual ~ListManaged();

	/// \brief Send a Report Element message with the requested element.
	/// Send a Report Element message with the requested element.
	/// \param[in] queryElement - Input Trigger.
	/// \return ReportElement Output Message.
	virtual ReportElement getReportElement(QueryElement *queryElement);

	/// \brief Send action for ReportElementList with input message QueryElementList.
	/// Send action for ReportElementList with input message QueryElementList.
	/// \param[in] queryElementList - Input Trigger.
	/// \return ReportElementList Output Message.
	virtual ReportElementList getReportElementList(QueryElementList *queryElementList);

	/// \brief Send action for ReportElementCount with input message QueryElementCount.
	/// Send action for ReportElementCount with input message QueryElementCount.
	/// \param[in] queryElementCount - Input Trigger.
	/// \return ReportElementCount Output Message.
	virtual ReportElementCount getReportElementCount(QueryElementCount *queryElementCount);

	/// \brief Send action for ConfirmElementRequest with input message SetElement.
	/// Send action for ConfirmElementRequest with input message SetElement.
	/// \param[in] setElement - Input Trigger.
	/// \return ConfirmElementRequest Output Message.
	virtual ConfirmElementRequest getConfirmElementRequest(SetElement *setElement);

    
	/// \brief Send action for ConfirmElementRequest with input message DeleteElement.
	/// Send action for ConfirmElementRequest with input message DeleteElement.
	/// \param[in] deleteElement - Input Trigger.
	/// \return ConfirmElementRequest Output Message.
	virtual ConfirmElementRequest getConfirmElementRequest(DeleteElement *deleteElement);

	/// \brief Send action for RejectElementRequest with input message SetElement.
	/// Send action for RejectElementRequest with input message SetElement.
	/// \param[in] setElement - Input Trigger.
	/// \return RejectElementRequest Output Message.
	virtual RejectElementRequest getRejectElementRequest(SetElement *setElement);

    
	/// \brief Send action for RejectElementRequest with input message DeleteElement.
	/// Send action for RejectElementRequest with input message DeleteElement.
	/// \param[in] deleteElement - Input Trigger.
	/// \return RejectElementRequest Output Message.
	virtual RejectElementRequest getRejectElementRequest(DeleteElement *deleteElement);

	/// \brief Store the element(s) in the list with sequence specified by the previous and next element IDs. If this action represents an insert or append into an existing list, the service should modify the NextUID of the previous element and/or the Previous UID of the next element to reflect the updated sequence.
	/// Store the element(s) in the list with sequence specified by the previous and next element IDs. If this action represents an insert or append into an existing list, the service should modify the NextUID of the previous element and/or the Previous UID of the next element to reflect the updated sequence.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool setElement(SetElement *setElement);

	/// \brief Remove the specified element(s) from the list. The service should modify the NextUID of the previous element and/or the Previous UID of the next element to reflect the updated sequence.
	/// Remove the specified element(s) from the list. The service should modify the NextUID of the previous element and/or the Previous UID of the next element to reflect the updated sequence.
	/// \param[in]  deleteElement - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool deleteElement(DeleteElement *deleteElement);


	/// \brief True if the UID(s) specified in the message that triggered the transition exists in the list.
	/// True if the UID(s) specified in the message that triggered the transition exists in the list.
	/// \param[in]  queryElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool elementExists(QueryElement *queryElement);

	/// \brief True if the UID(s) specified in the message that triggered the transition exists in the list.
	/// True if the UID(s) specified in the message that triggered the transition exists in the list.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool elementExists(SetElement *setElement);

	/// \brief True if the UID(s) specified in the message that triggered the transition exists in the list.
	/// True if the UID(s) specified in the message that triggered the transition exists in the list.
	/// \param[in]  deleteElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool elementExists(DeleteElement *deleteElement);


	/// \brief isControllingListClient condition.
	/// isControllingListClient condition.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingListClient(SetElement *setElement);

	/// \brief isControllingListClient condition.
	/// isControllingListClient condition.
	/// \param[in]  deleteElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingListClient(DeleteElement *deleteElement);


	/// \brief True if the resulting list will not be invalid as defined by the List Manager Service description and the receiving entity has sufficient memory to store the element(s).
	/// True if the resulting list will not be invalid as defined by the List Manager Service description and the receiving entity has sufficient memory to store the element(s).
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isValidElementRequest(SetElement *setElement);


	/// \brief False. This condition must be overridden by derived services to allow the list to be populated.
	/// False. This condition must be overridden by derived services to allow the list to be populated.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isElementSupported(SetElement *setElement);


	// Start of user code for additional methods:
	// End of user code

protected:
	ListManagerDefaultLoop listManagerDefaultLoop;
	ListManagerControlledLoop listManagerControlledLoop;


	// Start of user code for additional members:
	// End of user code
};

} // namespace component
} // namespace openjaus

#endif // LISTMANAGED_H
