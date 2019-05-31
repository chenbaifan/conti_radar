#ifndef KVASER_INTERFACE_H
#define KVASER_INTERFACE_H

#include "canbus/can_interface.h"
#include <canlib.h>
/**
 * @namespace saic::drivers::canbus
 * @brief saic::drivers::canbus
 */
namespace saic {
namespace drivers {
namespace canbus {

/**
 * @class KvaserInterface
 * @brief The class which defines the CAN client to send and receive message.
 */
class KvaserInterface:public CanInterface {
    public:
    /**
    * @brief Constructor
    */
    KvaserInterface();

    /**
    * @brief Destructor
    */
    ~KvaserInterface();


    /**
    * @brief Called to pass in parameters
    */ 
    bool Init(const CANCardParameter &parameter) override ;

    /**
    * @brief Called to check open and if not open then open can link
    */ 
    return_statuses Start() override;

    /**
    * @brief Called to check open and if open then close open can link
    */ 
    void Stop() override;

    /**
    * @brief Check to see if the CAN link is open
    */
    bool IsOpen() override;

    /**
    * @brief Read a message
    */
    return_statuses read(CanFrame* frame) override;
    /**
    * @brief Send a message
    */
    return_statuses write(const CanFrame* const frame) override ;
  private:
    void *handle_;
    canHandle *h;
    bool on_bus_;
    int channel_ = -1;
    int bit_rate_;
    bool echo_on_;

};//KvaserInterface
} // namespace canbus
} // namespace drivers
} // namespace saic

#endif //KVASER_INTERFACE_H