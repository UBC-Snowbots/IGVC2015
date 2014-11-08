// File Header Here

#include <openjaus.h>
#include "openjaus/core.h"
// Start of user code for additional includes:
#include "openjaus/core/Triggers/QueryHeartbeatPulse.h"
#include <typeinfo>
// End of user code


namespace openjaus
{
namespace core
{

class LivenessService : public Liveness
{
public:
	LivenessService()
	{
	};
	
	// Start of user code for Constructor:
	system::Thread *hbThread;

	void hbStart()
	{
		hbThread = new system::Thread(THREAD_METHOD(LivenessService, hbThreadMethod), this);
		hbThread->create();
	}

	void hbStop()
	{
		hbThread->join();
		delete hbThread;
	}


	void *hbThreadMethod()
	{
		using namespace openjaus::transport;

		QueryHeartbeatPulse qhb;

		while(hbThread->isRunning())
		{
			try
			{
				broadcastToNode(&qhb);
			}
			catch(openjaus::system::Exception exp)
			{
				std::cout << exp.getDescription() << "\n";
			}

			sleep(1);
		}
		return NULL;
	};

	virtual bool sendReportHeartbeatPulse(model::Trigger *trigger)
	{
		LOG("LivenessService: Sending Heartbeat Pulse");
		return true;
	};
	// End of user code
};

} // namespace core
} // namespace openjaus

int main(void)
{
	// Start of user code for main:
	try
	{
		openjaus::core::LivenessService service;

		service.hbStart();

		sleep(8);

		service.hbStop();
	}
	catch(openjaus::system::Exception exp)
	{
		std::cout << exp.getDescription() << "\n";
	}

	return 0;	
	// End of user code
}


