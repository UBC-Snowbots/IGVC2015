/**
\file system.h

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

#ifndef SYSTEM_H
#define SYSTEM_H

// Start of user code for additional include files
// End of user code


// Type Definitions

// Enumerations
#include "openjaus/system/RunLevel.h"

// Classes
#include "openjaus/system/Accessable.h"
#include "openjaus/system/Application.h"
#include "openjaus/system/Buffer.h"
#include "openjaus/system/Configuration.h"
#include "openjaus/system/Condition.h"
#include "openjaus/system/DatagramSocket.h"
#include "openjaus/system/InetAddress.h"
#include "openjaus/system/NetworkInterface.h"
#include "openjaus/system/Event.h"
#include "openjaus/system/Exception.h"
#include "openjaus/system/Logger.h"
#include "openjaus/system/MulticastSocket.h"
#include "openjaus/system/Mutex.h"
#include "openjaus/system/Packet.h"
#include "openjaus/system/Prioritized.h"
#include "openjaus/system/PriorityQueue.h"
#include "openjaus/system/Setting.h"
#include "openjaus/system/Queue.h"
#include "openjaus/system/Socket.h"
#include "openjaus/system/StreamServer.h"
#include "openjaus/system/StreamSocket.h"
#include "openjaus/system/Thread.h"
#include "openjaus/system/Time.h"
#include "openjaus/system/Transportable.h"
#include "openjaus/system/Timer.h"
#include "openjaus/system/Compressor.h"
#include "openjaus/system/ZlibCompressor.h"
#include "openjaus/system/ScopeLock.h"

#endif // SYSTEM_H
