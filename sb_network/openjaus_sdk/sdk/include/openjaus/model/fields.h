/**
\file fields.h

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

#ifndef FIELDS_H
#define FIELDS_H

// Start of user code for additional include files
#include "openjaus/types.h"
// End of user code


// Type Definitions
typedef int8_t int8;
typedef int16_t int16;
typedef int32_t int32;
typedef int64_t int64;
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;

// Enumerations
#include "openjaus/model/fields/TypesUnsigned.h"
#include "openjaus/model/fields/Units.h"
#include "openjaus/model/fields/LimitType.h"
#include "openjaus/model/fields/RoundingType.h"

// Classes
#include "openjaus/model/fields/Field.h"
#include "openjaus/model/fields/ComplexField.h"
#include "openjaus/model/fields/FixedPoint.h"
#include "openjaus/model/fields/VariablePoint.h"
#include "openjaus/model/fields/Boolean.h"
#include "openjaus/model/fields/Signed.h"
#include "openjaus/model/fields/Unsigned.h"
#include "openjaus/model/fields/Byte.h"
#include "openjaus/model/fields/Short.h"
#include "openjaus/model/fields/Integer.h"
#include "openjaus/model/fields/Long.h"
#include "openjaus/model/fields/UnsignedByte.h"
#include "openjaus/model/fields/UnsignedShort.h"
#include "openjaus/model/fields/UnsignedInteger.h"
#include "openjaus/model/fields/UnsignedLong.h"
#include "openjaus/model/fields/Float.h"
#include "openjaus/model/fields/Double.h"
#include "openjaus/model/fields/ScaledInteger.h"
#include "openjaus/model/fields/ScaledIntegerLimit.h"
#include "openjaus/model/fields/Enumeration.h"
#include "openjaus/model/fields/EnumerationItem.h"
#include "openjaus/model/fields/EnumerationLiteral.h"
#include "openjaus/model/fields/EnumerationRange.h"
#include "openjaus/model/fields/Variant.h"
#include "openjaus/model/fields/BitField.h"
#include "openjaus/model/fields/BitFieldItem.h"
#include "openjaus/model/fields/BitFieldRange.h"
#include "openjaus/model/fields/BitFieldEnumeration.h"
#include "openjaus/model/fields/BitFieldEnumerationValue.h"
#include "openjaus/model/fields/FixedLengthString.h"
#include "openjaus/model/fields/VariableLengthString.h"
#include "openjaus/model/fields/Blob.h"
#include "openjaus/model/fields/BlobType.h"
#include "openjaus/model/fields/Array.h"
#include "openjaus/model/fields/ArrayDimension.h"
#include "openjaus/model/fields/ArrayType.h"
#include "openjaus/model/fields/Record.h"
#include "openjaus/model/fields/FieldReference.h"
#include "openjaus/model/fields/VariablePointReference.h"
#include "openjaus/model/fields/Unitized.h"
#include "openjaus/model/fields/MessageField.h"

#endif // FIELDS_H
