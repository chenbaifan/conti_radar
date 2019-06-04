#include "mttcan_interface.h"


namespace radar_driver {

//Default constructor.
MttcanInterface::MttcanInterface() :
    on_bus_(false), sdk(DW_NULL_HANDLE), hal(DW_NULL_HANDLE), canSensor(DW_NULL_HANDLE)
{
    // instantiate Driveworks SDK context
    dwContextParameters sdkParams = {};
    dwInitialize(&sdk, DW_VERSION, &sdkParams);

    // create HAL module of the SDK
    dwSAL_initialize(&hal, sdk);
}

//Default destructor.
MttcanInterface::~MttcanInterface()
{
    if (canSensor != DW_NULL_HANDLE)
    {
        Stop();
    }

    // release used objects in correct order
    dwSAL_release(&hal);
    dwRelease(&sdk);
}

return_statuses MttcanInterface::Start()
{
    // create CAN bus interface
    canSensor = DW_NULL_HANDLE;
    {
        dwSensorParams params;
        //std::string parameterString = arguments.get("params");
        params.parameters = "device=can0"; //parameterString.c_str();
        params.protocol = "can.socket"; //arguments.get("driver").c_str();
        if (dwSAL_createSensor(&canSensor, params, hal) != DW_SUCCESS) {
            std::cout << "Cannot create sensor "
                      << params.protocol << " with " << params.parameters << std::endl;
            return INIT_FAILED;
        }
    }

    // if (0) //? disable HW timestamps
    // {
    //     dwSensorCAN_setUseHwTimestamps(false, canSensor);
    // }

    on_bus_ = dwSensor_start(canSensor) == DW_SUCCESS;
    return OK;
}

bool MttcanInterface::Init(const CANCardParameter &parameter) {
    return OK;
}

bool MttcanInterface::IsOpen()
{
    if (canSensor == DW_NULL_HANDLE)
    {
        return false;
    }
    else
    {
        if (on_bus_)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}

void MttcanInterface::Stop()
{
    // if (canSensor == DW_NULL_HANDLE)
    // {
    //     return INIT_FAILED;
    // }

    dwSensor_stop(canSensor);
    dwSAL_releaseSensor(&canSensor);

    on_bus_ = false;
}

return_statuses MttcanInterface::read(CanFrame* const frame)
{
    if (canSensor == DW_NULL_HANDLE)
    {
        return INIT_FAILED;
    }

    //std::this_thread::sleep_for(std::chrono::microseconds(10));

    dwCANMessage dmsg;
    bool done = false;
    return_statuses ret_val = INIT_FAILED;
    unsigned int flag = 0;

    while (!done)
    {
        dwStatus ret = dwSensorCAN_readMessage(&dmsg, 0, canSensor);  //0, timeout_us

        if (ret == DW_SAL_SENSOR_ERROR)
        {
            ret_val = CHANNEL_CLOSED;
            on_bus_ = false;
            done = true;
        }
        else if (ret == DW_TIME_OUT)
        {
            ret_val = NO_MESSAGES_RECEIVED;
            done = true;
        }
        else if (ret != DW_SUCCESS)
        {
            ret_val = READ_FAILED;
            done = true;
        }
        else //if (!(flag & 0xF9))
        {
            // Was a received message with actual data
            frame->id = dmsg.id;
            std::copy(dmsg.data, dmsg.data + DW_SENSORS_CAN_MAX_MESSAGE_LEN, frame->data);
            frame->dlc = dmsg.size;
            frame->timestamp = dmsg.timestamp_us;
            ret_val = OK;
            done = true;
        }
    }

    frame->is_extended = false;

    return ret_val;
}

return_statuses MttcanInterface::write(const CanFrame* const frame)
{
    if (canSensor == DW_NULL_HANDLE)
    {
        return INIT_FAILED;
    }

    dwCANMessage canMsg;
    canMsg.id = frame->i} // namespace drivers
} // namespace saicd;
    canMsg.size = std::m} // namespace drivers
} // namespace saicin((uint16_t)frame->dlc, (uint16_t)DW_SENSORS_CAN_MAX_MESSAGE_LEN);
    canMsg.timestamp_us } // namespace drivers
} // namespace saic= 0;
    std::copy(frame->dat} // namespace drivers
} // namespace saica, frame->data + canMsg.size, canMsg.data);
} // namespace drivers
} // namespace saic
    dwStatus ret = dwSen} // namespace drivers
} // namespace saicsorCAN_sendMessage(&canMsg, 100000, canSensor);

    return (ret == DW_SUCCESS) ? OK : WRITE_FAILED;
}

} // namespace radar_driver