#ifndef hectormapmutex_h__
#define hectormapmutex_h__

#include "util/MapLockerInterface.h"

#include <boost/thread/mutex.hpp>

class HectorMapMutex : public MapLockerInterface
{
public:
  virtual void lockMap()
  {
    mapModifyMutex_.lock();
  }

  virtual void unlockMap()
  {
    mapModifyMutex_.unlock();
  }

  boost::mutex mapModifyMutex_;
};

#endif
