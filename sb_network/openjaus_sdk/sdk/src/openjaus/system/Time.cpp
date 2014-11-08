/**
\file Time.h

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

#include "openjaus/system/Time.h"
#include <sstream>
// Start of user code for additional includes
#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <time.h>
#include <windows.h>
#else
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#endif
#include <sstream>
// End of user code

namespace openjaus
{
namespace system
{

// Start of user code for default constructor:
Time::Time() :
		seconds(0),
		microseconds(0)
{
}
// End of user code

// Start of user code for default destructor:
Time::~Time()
{
}
// End of user code

int Time::getSeconds() const
{
	// Start of user code for accessor getSeconds:
	
	return seconds;
	// End of user code
}

bool Time::setSeconds(int seconds)
{
	// Start of user code for accessor setSeconds:
	this->seconds = seconds;
	
	return true;
	// End of user code
}


int Time::getMicroseconds() const
{
	// Start of user code for accessor getMicroseconds:
	
	return microseconds;
	// End of user code
}

bool Time::setMicroseconds(int microseconds)
{
	// Start of user code for accessor setMicroseconds:
	this->microseconds = microseconds;

	return true;
	// End of user code
}



// Class Methods
void Time::sleep(int milliseconds)
{
	// Start of user code for method sleep:
#ifdef WIN32
	Sleep(milliseconds);
#else
	usleep(milliseconds * 1000);
#endif
	// End of user code
}


Time Time::getTime()
{
	// Start of user code for method getTime:
	Time result;
#ifdef WIN32
	// TODO: the following code could have a 1 second inaccuracy if a second transition occurs after the GetSystemTime call
	SYSTEMTIME st;
	GetSystemTime(&st);
	result.setSeconds(time(NULL));
	result.setMicroseconds(1000 * st.wMilliseconds);
#else
	static struct timeval time;
	gettimeofday(&time, NULL);
	result.setSeconds(time.tv_sec);
	result.setMicroseconds(time.tv_usec);
#endif
	return result;
	// End of user code
}


double Time::inSec()
{
	// Start of user code for method inSec:
	return seconds + microseconds / 1000000.0;
	// End of user code
}




std::string Time::toString() const
{	
	// Start of user code for toString
	time_t systemTimeSec = seconds;
    struct tm * timeinfo = localtime ( &systemTimeSec );

    const int bufferSize = 64;
    char buf[bufferSize] = {0};
    int strLen = strftime(buf, bufferSize, "%I:%M:%S:", timeinfo);
    std::string stringValue(buf, strLen);

    std::ostringstream oss;
    oss.fill('0');
    oss.width(3);
    oss << static_cast<int>(microseconds / 1000.0);
    stringValue.append(oss.str());

    strLen = strftime(buf, bufferSize, " %p", timeinfo);
    stringValue.append(buf, strLen);

    return stringValue;
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const Time& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
// End of user code

} // namespace system
} // namespace openjaus

