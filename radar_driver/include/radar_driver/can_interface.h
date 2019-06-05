#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H

//C++ Includes
#include <iostream>
#include <cstring>
#include <ros/ros.h>

//OS Includes
#include <unistd.h>

/**
 * @namespace radar
 * @brief radar_driver
 */

namespace radar_driver {

 /**
 * @struct CANCardParameter
 * @brief The struct which defines the information to vehicle can card.
 */
typedef struct CANCardParameter{
    int bit_rate = 500000;
    int hardware_id = 0;
    int circuit_id = 0;
    std::string radar_model;
    bool echo_on = true;
}CANCardParameter;

enum return_statuses
{
    OK = 0,
    INIT_FAILED = -1,
    BAD_PARAM = -2,
    NO_CHANNELS_FOUND = -3,
    CHANNEL_CLOSED = -4,
    NO_MESSAGES_RECEIVED = -5,
    READ_FAILED = -6,
    WRITE_FAILED = -7,
    CLOSE_FAILED = -8
};

 /**
 * @class CanFrame
 * @brief The class which defines the information to send and receive.
 */
 struct CanFrame {
    /// Message id
    long id; 
    /// Message length
    unsigned int dlc;
    /// Message content
    uint8_t data[8];
    /// Time stamp
    unsigned long timestamp;
    bool is_rtr;
    bool is_extended;
    bool is_error;
    /**
     * @brief Constructor
     */
    CanFrame() : id(0), dlc(0), timestamp{0}, is_rtr(false), is_extended(false), is_error(false)  {
    std::memset(data, 0, sizeof(data));
    }

    /**
     * @brief CanFrame string including essential information about the message.
     * @return The info string.
     */
    // std::string CanFrameString() const {
    // std::stringstream output_stream("");
    // output_stream << "id:0x" << Byte::byte_to_hex(id)
    //                 << ",len:" << static_cast<int>(len) << ",data:";
    // for (uint8_t i = 0; i < len; ++i) {
    //     output_stream << Byte::byte_to_hex(data[i]);
    // }
    // output_stream << ",";
    // return output_stream.str();
    // }
};

/**
 * @class CanClient
 * @brief The class which defines the CAN client to send and receive message.
 */
class CanInterface {
    public:
    /**
    * @brief Constructor
    */
    CanInterface() = default;

    /**
    * @brief Destructor
    */
    virtual ~CanInterface() = default;
    /**
    * @brief Called to pass in parameters
    */ 
    virtual bool Init(const CANCardParameter &parameter) = 0 ;

    /**
    * @brief Called to check open and if not open then open can link
    */ 
    virtual return_statuses Start() = 0 ;

    /**
    * @brief Called to check open and if open then close open can link
    */ 
    virtual void Stop() = 0 ;

    /**
    * @brief Check to see if the CAN link is open
    */
    virtual bool IsOpen()=0;

    /**
    * @brief Read a message
    */
    virtual return_statuses read(CanFrame *frame)=0;
    /**
    * @brief Send a message
    */
    virtual return_statuses write(const CanFrame* const frame)=0;

};//CanClient

// Converts error messages to human-readable strings
std::string return_status_desc(const return_statuses& ret);

} // namespace radar

#endif //CAN_INTERFACE_H
