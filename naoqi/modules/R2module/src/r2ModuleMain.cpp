#ifndef _WIN32
#include <signal.h>
#endif

#include <boost/shared_ptr.hpp>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>
#include <alcommon/albrokermanager.h>
#include <alcommon/altoolsmain.h>

#include "r2Module.h"

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
  AL::ALModule::createModule<AL::R2Module>(pBroker,"r2Module" );

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
  ALTools::mainFunction("r2Module", argc, argv, sig);
}
