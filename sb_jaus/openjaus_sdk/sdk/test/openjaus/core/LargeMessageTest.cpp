// File Header Here

#include <openjaus.h>
#include "openjaus/core.h"
#include "openjaus/core/Base.h"
#include <math.h>

using namespace openjaus;

class TestMessage : public model::Message
{
public:
	static const uint16_t ID = 0x0701;

	TestMessage() : model::Message()
	{
		this->id = TestMessage::ID; // Initialize id member
		setType(transport::JAUS_MESSAGE);
	}

	TestMessage(model::Message *message) :
		model::Message(message)
	{
		this->id = TestMessage::ID; // Initialize id member
		setType(transport::JAUS_MESSAGE);

		system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
		if(payloadBuffer)
		{
			this->from(payloadBuffer);
			payloadBuffer->reset();
		}
	}

	virtual int to(system::Buffer *dst)
	{
		int byteSize = dst->pack(this->id);
		data.reset();
		uint32_t dataSize = data.remainingBytes();
		std::cout << "to: remaining bytes: " << data.remainingBytes() << std::endl;
		byteSize += dst->pack(dataSize);
		byteSize += dst->pack(data);
		return byteSize;
	}

	virtual int from(system::Buffer *src)
	{
		int byteSize = src->unpack(this->id);
		uint32_t dataSize;
		byteSize += src->unpack(dataSize);
		data.setMaxSize(dataSize);
		byteSize += src->unpack(data);

		int checksum = 0;
		for(int i = 0; i < data.getMaxSize(); i++)
		{
			unsigned char tempByte;
			data.unpack(tempByte);
			checksum += tempByte;
		}
		std::cout << "Received TestMessage[" << this->sequenceNumber << "] Checksum: " << checksum << std::endl;
		return byteSize;
	}

	virtual int length()
	{
		return sizeof(id) + sizeof(uint32_t) + data.getMaxSize();
	}

	system::Buffer data;
};

class PingComponent : public openjaus::core::Base
{
public:
	PingComponent()
	{
		name = "OpenJAUSPingComponent";
		receive.addMessageCallback(&PingComponent::processEcho, this);
	}

	bool processEcho(TestMessage &echo)
	{
		std::cout << "Response from: " << echo.getSource().toString() << ": seq=" << echo.getSequenceNumber() << std::endl;
		return true;
	}
};

int main(void)
{
	short pingId = 0;
	int testMessageSize = 64;
	openjaus::system::Application::setTerminalMode();
	try
	{
		PingComponent component;
		component.run();
		openjaus::system::Time::sleep(1000);

		openjaus::core::Base base;
		base.run();

		unsigned char choice = 0;
		while(choice != 27) // ESC
		{
			choice = openjaus::system::Application::getChar();
			switch(choice)
			{
				case 't':
					LOG(component.getSystemTree()->toString());
					break;
				case 'p':
					testMessageSize *= 2;

					TestMessage *pingMsg = new TestMessage();
					pingMsg->data.setMaxSize(testMessageSize);
					pingMsg->setSequenceNumber(pingId++);
					pingMsg->setDestination(component.getAddress());

					int checksum = 0;
					for(int i = 0; i < pingMsg->data.getMaxSize(); i++)
					{
						unsigned char tempByte = rand() % 256;
						pingMsg->data.pack(tempByte);
						checksum += tempByte;
					}
					std::cout << "Data remaining bytes: " << pingMsg->data.remainingBytes() << std::endl;
					std::cout << "Sending TestMessage[" << pingId-1 << "] Checksum: " << checksum << std::endl;

					base.sendMessage(pingMsg);
					break;
			}
		}
	}
	catch(openjaus::system::Exception expn)
	{
		openjaus::system::Logger::log(expn);
	}

	return 0;
	// End of user code
}
