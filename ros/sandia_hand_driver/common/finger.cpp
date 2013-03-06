#include "sandia_hand/finger.h"
#include <boost/function.hpp>
#include <boost/bind.hpp>
using namespace sandia_hand;

Finger::Finger()
: mm(10)
{
  mm.addPhalangeRxFunctor(boost::bind(&ProximalPhalange::rx, &pp, _1, _2));
  pp.setRawTx(boost::bind(&MotorModule::phalangeTxRx, &mm, _1, _2));
}

Finger::~Finger()
{
}
