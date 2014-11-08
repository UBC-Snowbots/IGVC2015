/**
\file Configuration.h

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

#include "openjaus/system/Configuration.h"
#include <sstream>
// Start of user code for additional includes
#include "openjaus/system/Exception.h"
#include <iostream>
#include <fstream>
// End of user code

namespace openjaus
{
namespace system
{

// Start of user code for default constructor:
Configuration::Configuration() :
		fileName()
{
}
// End of user code

// Start of user code for default destructor:
Configuration::~Configuration()
{
	save();

	// Free allocated settings
	for(std::map<std::string, Setting *>::const_iterator i = settingMap.begin();
		i != settingMap.end();
		++i)
	{
		delete i->second;
	}

}
// End of user code

std::string Configuration::getFileName() const
{
	// Start of user code for accessor getFileName:
	
	return fileName;
	// End of user code
}

bool Configuration::setFileName(std::string fileName)
{
	// Start of user code for accessor setFileName:
	this->fileName = fileName;
	return true;
	// End of user code
}


const std::map< std::string, Setting * >& Configuration::getSettingMap() const
{
	// Start of user code for accessor getSettingMap:
	
	return settingMap;
	// End of user code
}

bool Configuration::setSettingMap(std::map< std::string, Setting * > settingMap)
{
	// Start of user code for accessor setSettingMap:
	this->settingMap = settingMap;
	return true;
	// End of user code
}



// Class Methods
void Configuration::load(std::string fileName)
{
	// Start of user code for method load:
	this->fileName = fileName;

	std::ifstream file(fileName.c_str());
	if(!file.is_open())
	{
		LOG("Configuration file: " << this->fileName << ", not found. Using defaults...");
		return;
	}

	LOG("Loading Config File: " << this->fileName);
	while(!file.eof())
	{
		std::string line;
		std::getline(file, line);
		size_t keyPosition = line.find_first_not_of(" \t\n\r");
		if(keyPosition == std::string::npos || line[keyPosition] == '#')
		{
			continue;
		}

		LOG("\t" << line);
		std::string::size_type equalsPosition = line.find("=");
		if(equalsPosition != std::string::npos)
		{
			std::string key = line.substr(keyPosition, equalsPosition);
			settingMap[key] = new Setting();
			settingMap[key]->setKey(key);
			settingMap[key]->setValueStr(line.substr(equalsPosition+1));
			settingMap[key]->setGlobalRead(false);
			settingMap[key]->setControllerRead(true);
			settingMap[key]->setControllerWrite(false);
		}
	}
	// End of user code
}


void Configuration::save()
{
	// Start of user code for method save:
	LOG_DEBUG("Configuration: Attempting to save file: " << this->fileName.c_str());
	std::ofstream file(fileName.c_str());
	if(!file.is_open())
	{
		THROW_EXCEPTION("Could not open configuration file: " << fileName);
	}

	file << toString();
	// End of user code
}




std::string Configuration::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;

	for(std::map<std::string, Setting *>::const_iterator i = settingMap.begin();
		i != settingMap.end();
		++i)
	{
		if(i->second->getComment().length())
		{
			oss << "#" << i->first << ": " << i->second->getComment() << std::endl;
		}
		oss << i->first << "=" << i->second->getValueStr() << std::endl;
		oss << std::endl;
	}
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const Configuration& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods

// End of user code

} // namespace system
} // namespace openjaus

