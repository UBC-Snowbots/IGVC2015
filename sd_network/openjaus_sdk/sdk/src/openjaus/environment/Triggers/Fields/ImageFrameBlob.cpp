

/**
\file ImageFrameBlob.h

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
#include "openjaus/environment/Triggers/Fields/ImageFrameBlob.h"

namespace openjaus
{
namespace environment
{

ImageFrameBlob::ImageFrameBlob() :
		format()
{

}

ImageFrameBlob::~ImageFrameBlob()
{
}

void ImageFrameBlob::copy(ImageFrameBlob& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	this->format = source.getFormat();
	Blob::setPayload(source.getPayload());
}

std::string ImageFrameBlob::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}
	
	std::ostringstream oss;
	oss << prefix.str() << "<Blob name=\"" << this->name << "\">\n";
	oss << prefix.str() << "\t" << "<BlobType name=" << this->formatToString() << "\" value=" << this->format << "/>\n";
	oss << prefix.str() << "\t" << "<payloadSizeBytes>" << this->payload.containedBytes() << "</payloadSizeBytes>\n";
	oss << prefix.str() << "</Blob>\n";
	
	return oss.str();
}

int ImageFrameBlob::to(system::Buffer *dst)
{
	int result = 0;
	int payloadSizeBytes = this->payload.containedBytes();

	result += dst->pack(static_cast<uint8_t>(this->getFormat()));
	result += dst->pack(static_cast<uint32_t>(payloadSizeBytes));

	if(dst->remainingBytes() < payloadSizeBytes)
	{
		THROW_EXCEPTION("ImageFrameBlob::serialize: Payload length (" << payloadSizeBytes<< ") exceeds remaining buffer (" << dst->remainingBytes() << ")");
	}

	//result += dst->pack(this->payload);
	memcpy(dst->getPointer(), this->payload.getBuffer(), this->payload.containedBytes());
	dst->increment(this->payload.length());
		
	return result;
}

int ImageFrameBlob::from(system::Buffer *src)
{
	int result = 0;

	result += src->unpack(this->format);
	uint32_t payloadSizeBytes;
	result += src->unpack(payloadSizeBytes);
	
	this->payload.setMaxSize(payloadSizeBytes);
	result += src->unpack(this->payload);
		
	return result;
}

int ImageFrameBlob::length(void)
{
	int result = 0;

	result += sizeof(uint8_t); // Format
	result += sizeof(uint32_t); // Payload Size
	result += this->payload.containedBytes(); // Payload
	return result;
}

uint8_t ImageFrameBlob::getFormat()
{
	return this->format;
}

std::string ImageFrameBlob::formatToString() const
{
	switch(this->format)
	{
		case JPEG:
			return "JPEG";
		case GIF:
			return "GIF";
		case PNG:
			return "PNG";
		case BMP:
			return "BMP";
		case TIFF:
			return "TIFF";
		case PPM:
			return "PPM";
		case PGM:
			return "PGM";
		case PNM:
			return "PNM";
		case NEF:
			return "NEF";
		case CR_2:
			return "CR_2";
		case DNG:
			return "DNG";
		default:
			return "Unknown Type";
	}
}

bool ImageFrameBlob::setPayload(uint8_t format, system::Buffer& payload)
{
	this->format = format;
	Blob::setPayload(payload);
	
	return true;
}

} // namespace environment
} // namespace openjaus
