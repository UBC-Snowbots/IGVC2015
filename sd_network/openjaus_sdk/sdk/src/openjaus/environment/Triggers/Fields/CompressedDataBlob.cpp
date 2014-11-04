

/**
\file CompressedDataBlob.h

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
#include "openjaus/environment/Triggers/Fields/CompressedDataBlob.h"

namespace openjaus
{
namespace environment
{

CompressedDataBlob::CompressedDataBlob() 
{

}

CompressedDataBlob::~CompressedDataBlob()
{
}

void CompressedDataBlob::copy(CompressedDataBlob& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	Blob::setPayload(source.getPayload());
}

std::string CompressedDataBlob::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}
	
	std::ostringstream oss;
	oss << prefix.str() << "<Blob name=\"" << this->name << "\">\n";
	oss << prefix.str() << "\t" << "<payloadSizeBytes>" << this->payload.containedBytes() << "</payloadSizeBytes>\n";
	oss << prefix.str() << "</Blob>\n";
	
	return oss.str();
}

int CompressedDataBlob::to(system::Buffer *dst)
{
	int result = 0;
	int payloadSizeBytes = this->payload.containedBytes();

	result += dst->pack(static_cast<uint32_t>(payloadSizeBytes));

	if(dst->remainingBytes() < payloadSizeBytes)
	{
		THROW_EXCEPTION("CompressedDataBlob::serialize: Payload length (" << payloadSizeBytes<< ") exceeds remaining buffer (" << dst->remainingBytes() << ")");
	}

	//result += dst->pack(this->payload);
	memcpy(dst->getPointer(), this->payload.getBuffer(), this->payload.containedBytes());
	dst->increment(this->payload.length());
		
	return result;
}

int CompressedDataBlob::from(system::Buffer *src)
{
	int result = 0;

	uint32_t payloadSizeBytes;
	result += src->unpack(payloadSizeBytes);
	
	this->payload.setMaxSize(payloadSizeBytes);
	result += src->unpack(this->payload);
		
	return result;
}

int CompressedDataBlob::length(void)
{
	int result = 0;

	result += sizeof(uint32_t); // Payload Size
	result += this->payload.containedBytes(); // Payload
	return result;
}


} // namespace environment
} // namespace openjaus
