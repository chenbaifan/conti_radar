#ifndef MTTCAN_INTERFACE_H
#define MTTCAN_INTERFACE_H

// #include <iostream>
// #include <unistd.h>
#include <dw/sensors/Sensors.h>
#include <dw/sensors/canbus/CAN.h>
#include <dw/core/VersionCurrent.h>

#include "canbus/can_interface.h"

namespace saic {
namespace drivers {
namespace canbus {

class MttcanInterface: public CanInterface
{
public:
    MttcanInterface();

    ~MttcanInterface();

    // // Close the can link
    // return_statuses close();

    // Called to pass in parameters
    bool Init(const CANCardParameter &parameter) override ;

    // Called to check open and if not open then open can link
    return_statuses Start() override;

    // Called to check open and if open then close open can link
    void Stop() override;

    // Check to see if the CAN link is open
    bool IsOpen() override;

    // Read a message
    return_statuses read(CanFrame* frame) override;

    // Send a message
    return_statuses write(const CanFrame* const frame) override;
private:
    bool on_bus_;
    dwContextHandle_t sdk;
    dwSALHandle_t hal;
    dwSensorHandle_t canSensor;
};

} // namespace canbus
} // namespace drivers
} // namespace saic

#endif //MTTCAN_INTERFACE_H