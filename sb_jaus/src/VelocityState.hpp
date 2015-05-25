#include <jaus/core/velocitystatesensor.h>

class VelocityStateManager{
private:
    VelocityStateSensor* sensor;
public:
    VelocityStateManager(VelocityStateSensor* sensor): sensor(sensor)
    {}
    void receive_data(/* get a message type */){

    }
};
