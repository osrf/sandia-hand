#ifndef SANDIA_HAND_MESSAGE_PROCESSOR
#define SANDIA_HAND_MESSAGE_PROCESSOR

#include <stdint.h>
#include <boost/function.hpp>

namespace sandia_hand
{

class MessageProcessor
{
public:
  MessageProcessor(const uint8_t addr);
  virtual ~MessageProcessor();
  uint8_t addr_;
  bool rx(uint8_t *data, uint16_t data_len);

  typedef boost::function<void(const uint8_t *, const uint16_t)> RawTxFunctor;
  inline void setRawTxHandler(RawTxFunctor f) { tx_ = f; }
  typedef boost::function<void(const uint8_t, const uint8_t *, 
                               const uint16_t)> RxMessageFunctor;
  inline void setRxMessageHandler(RxMessageFunctor f) { rx_message_ = f; }
  bool ping();

private:
  RawTxFunctor tx_;
  RxMessageFunctor rx_message_;
};

}

#endif
