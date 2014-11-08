/**
\file AS5669.h

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

#ifndef AS5669_H
#define AS5669_H

// Start of user code for additional include files
// End of user code


// Type Definitions

// Enumerations
#include "openjaus/transport/AS5669/CompressionType.h"

// Classes
#include "openjaus/transport/AS5669/ConfigurationWrapper.h"
#include "openjaus/transport/AS5669/JtcpInterface.h"
#include "openjaus/transport/AS5669/JudpAddress.h"
#include "openjaus/transport/AS5669/JudpInterface.h"
#include "openjaus/transport/AS5669/JudpPacket.h"
#include "openjaus/transport/AS5669/JtcpPacket.h"
#include "openjaus/transport/AS5669/JausWrapper.h"
#include "openjaus/transport/AS5669/OjudpWrapper.h"
#include "openjaus/transport/AS5669/TCPAddress.h"
#include "openjaus/transport/AS5669/JtcpStream.h"
#include "openjaus/transport/AS5669/JudpLargeMessageBuffer.h"

#endif // AS5669_H
