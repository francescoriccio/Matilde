#ifndef _WIN32
#include <signal.h>
#endif

#include <alcommon/albroker.h>

#include "amrModule.h"

#define ALCALL

extern "C"
{
ALCALL int _createModule(boost::shared_ptr<AL::ALBroker> pBroker)
{
  // init broker with the main broker instance
  // from the parent executable
  AL::ALBrokerManager::setInstance(pBroker->fBrokerManager.lock());
  AL::ALBrokerManager::getInstance()->addBroker(pBroker);

  // create module instances
  AL::ALModule::createModule<AL::AmrModule>(pBroker,"AmrModule" );

  return 0;
}

ALCALL int _closeModule()
{
  return 0;
}

} // extern "C"

int main(int argc, char *argv[])
{
  // pointer on createModule
  TMainType sig;
  sig = &_createModule;

  // call main
  ALTools::mainFunction("amrModule", argc, argv, sig);
}
