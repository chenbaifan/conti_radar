#include "kvaser_interface.h"
// install linuxcan


namespace saic {
namespace drivers {
namespace canbus {

    //Default constructor.
KvaserInterface::KvaserInterface() :
        handle_(NULL),on_bus_(false), bit_rate_(500000),echo_on_(true)
{
  handle_ = malloc(sizeof(canHandle));
}

//Default destructor.
KvaserInterface::~KvaserInterface()
{
  if (handle_ != NULL)
  {
    Stop();
  }
  free(handle_);
}

bool KvaserInterface::Init(const CANCardParameter &parameter){
  bit_rate_ = parameter.bit_rate;
  echo_on_ = parameter. echo_on;
  if (handle_ == NULL)
  {
  return INIT_FAILED;
  }

  if (!on_bus_)
  {
    h = (canHandle *) handle_;

    int numChan;
    if (canGetNumberOfChannels(&numChan) != canOK)
    {
      return INIT_FAILED;
    }

    unsigned int serial[2];
    unsigned int channel_number;


    for (int idx = 0; idx < numChan; idx++)
    {
      if (canGetChannelData(idx, canCHANNELDATA_CARD_SERIAL_NO, &serial, sizeof(serial)) == canOK)
      {
        if (serial[0] == (unsigned int) parameter.hardware_id)
        {
          if (canGetChannelData(idx, canCHANNELDATA_CHAN_NO_ON_CARD, &channel_number, sizeof(channel_number)) == canOK)
          {
            if (channel_number == (unsigned int) parameter.circuit_id)
            {
              channel_ = idx;
            }
          }
        }
      }
    }

    if (channel_ == -1)
    {
      return BAD_PARAM;
    }
  }
}
return_statuses KvaserInterface::Start()
{
  return_statuses ret;
  if(IsOpen())
  {
    ret = OK;
    return ret;
  }
  // Open channel
  *h = canOpenChannel(channel_, canOPEN_ACCEPT_VIRTUAL);
  if (*h < 0)
  {
    return INIT_FAILED;
  }

  // Set bit rate and other parameters
  long freq;
  switch (bit_rate_)
  {
    case 125000: freq = canBITRATE_125K; break;
    case 250000: freq = canBITRATE_250K; break;
    case 500000: freq = canBITRATE_500K; break;
    case 1000000: freq = canBITRATE_1M; break;
    default:
    {
      return  BAD_PARAM;
    }
  }

  if (canSetBusParams(*h, freq, 0, 0, 0, 0, 0) < 0)
  {
    ret = BAD_PARAM;
    ROS_ERROR_THROTTLE(0.5, "Kvaser CAN Interface - Error opening reader: %d - %s", ret, return_status_desc(ret).c_str());
    return ret;
  }

  // Linuxcan defaults to echo on, so if you've opened the same can channel
  // from multiple interfaces they will receive the messages that each other
  // send.  Turn it off here if desired.
  if (!echo_on_)
  {
    unsigned char off = 0;
    canIoCtl(*h, canIOCTL_SET_LOCAL_TXECHO, &off, 1);
  }

  // Set output control
  canSetBusOutputControl(*h, canDRIVER_NORMAL);
  canBusOn(*h);
  on_bus_ = true;
  ret = OK;
  return ret;
}

bool KvaserInterface::IsOpen()
{
  if (handle_ == NULL)
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


 void KvaserInterface::Stop()
{
  if(IsOpen())
  {
    if (handle_ == NULL)
    {
      ROS_ERROR_THROTTLE(0.5, "Kvaser CAN Interface - Error closing reader: %d - %s", INIT_FAILED, return_status_desc(INIT_FAILED).c_str());
      return;
    }
    canHandle *h = (canHandle *) handle_;
    // Close the channel
    if (canClose(*h) != canOK)
    {
      ROS_ERROR_THROTTLE(0.5, "Kvaser CAN Interface - Error closing reader: %d - %s", CLOSE_FAILED, return_status_desc(CLOSE_FAILED).c_str());
      return;
    }
    on_bus_ = false;
  }
}

return_statuses KvaserInterface::read(CanFrame * const frame)
{
  if (handle_ == NULL)
  {
    return INIT_FAILED;
  }

  canHandle *h = (canHandle *) handle_;

  bool done = false;
  return_statuses ret_val = INIT_FAILED;
  unsigned int flag = 0;

  while (!done)
  {
    canStatus ret = canRead(*h, &frame->id,  (frame->data), &frame->dlc, &flag, &frame->timestamp);
    if (ret == canERR_NOTINITIALIZED)
    {
      ret_val = CHANNEL_CLOSED;
      on_bus_ = false;
      done = true;
    }
    else if (ret == canERR_NOMSG)
    {
      ret_val = NO_MESSAGES_RECEIVED;
      done = true;
    }
    else if (ret != canOK)
    {
      ret_val = READ_FAILED;
      done = true;
    }
    else if (!(flag & 0xF9))
    {
      // Was a received message with actual data
      ret_val = OK;
      done = true;
    }
    // Else a protocol message, such as a TX ACK, was received
    // Keep looping until one of the other conditions above is met
  }

  if (ret_val == OK)
  {
    frame->is_extended = ((flag & canMSG_EXT) > 0);
  }
  // else
  //   ROS_WARN_THROTTLE(0.5, "Kvaser CAN Interface - Error reading CAN message: %d - %s", ret_val, return_status_desc(ret_val).c_str());
  return ret_val;
}


return_statuses KvaserInterface::write(const CanFrame* const frame)
{
  if (handle_ == NULL)
  {
    return INIT_FAILED;
  }
  canHandle *h = (canHandle *) handle_;
  unsigned int flag;
  if (frame->is_extended)
  {
    flag = canMSG_EXT;
  }
  else
  {
    flag = canMSG_STD;
  }
  // unsigned char *msg = const_cast<unsigned char*>(&(frame->data[0]));
  // printf("Frmaes ID: %d DLC: %d Flag:%d\n -- %x %x %x %x %x %x %x %x \n", frame->id, frame->dlc, flag, 
  // msg[0], msg[1], msg[2], msg[3], msg[4], msg[5], msg[6], msg[7], msg[8]);
  canStatus ret = canWrite(*h, frame->id,const_cast<unsigned char*>(&(frame->data[0])), frame->dlc, flag);
  return_statuses ret_val = (ret == canOK) ? OK : WRITE_FAILED;
  if(ret_val != OK)
    ROS_WARN_THROTTLE(0.5, "Kvaser CAN Interface - CAN send error: %d - %s", ret_val, return_status_desc(ret_val).c_str());
  return ret_val;
}

} // namespace canbus
} // namespace drivers
} // namespace saic