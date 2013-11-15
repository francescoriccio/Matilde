#ifndef AmrModule_H
#define AmrModule_H

/*
  AmrModule class is written using the naoqi modules template and performs an interface to the robot from which we can retrieve all
  the perceptions and sensors data that are needed. This process uses those representation that are provided in order to communicate with
  the naoqi modules, namely proxies. Here we are using three of those proxies: 'alvideodeviceproxy', 'almotionproxy', 'alrobotpostureproxy',
  the first one to communicate with the Nao's camera and the others two to make the Nao robot walking.
  In the same manner, we are splitting the code in two threads, one that deals with the vision perception and the vision process and
  the other one that handles the motions of the robot.

  authors: Maria Laura Aceto, Claudio Delli Bovi, Francesco Riccio.

  Copyright (c) 2011 Aldebaran Robotics
*/

// boost includes
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

// alcommon includes
#include <alcore/alptr.h>
#include <alcommon/almodule.h>
#include <alcommon/altoolsmain.h>
#include <alcommon/albrokermanager.h>

// alproxies includes
#include <alproxies/alvideodeviceproxy.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>

#include "potentialField.h"
#include "controller.h"
#include <string>

namespace AL
{

class AmrModule : public AL::ALModule{

public:
    AmrModule( boost::shared_ptr<ALBroker> pBroker, const std::string& pName);

    virtual ~AmrModule();
    void exit();

    void registerToVideoDevice();
    void unRegisterFromVideoDevice();

    static void* motionTh(AmrModule* module){ module->motion(); return 0; }
    static void* visionTh(AmrModule* module){ module->computePotentialField(); return 0; }

    // walk: a simple wrapper for the 'move' naoqi method
    void walk(bool turn = false);

    // moveHead: a wrapper for the 'setAngle' naoqi method
    void moveHead(int redScreenCount);

    //motion: updates the controller with the newly data and guides the robot with the effort of the PD-controller
    void motion();

    //getCurrentFrame: retrieve an image from the robot (i.e. a wrapper for the 'getImageRemote' naoqi method)
    void getCurrentFrame();

    //computePotentialField: iteratively builds the potentialField through the 'class PotentialField' and provides the overall velocity
    void computePotentialField();

    // Automatically called right after the module is created.
    virtual void init();

private:

    // Proxy to the video and motion input module.
    boost::shared_ptr<AL::ALVideoDeviceProxy> fCamProxy;
    boost::shared_ptr<AL::ALMotionProxy> fMotionProxy;
    boost::shared_ptr<AL::ALRobotPostureProxy> fPostureProxy;

    boost::mutex mutex;

    // Client name that is used to talk to the Video Device.
    std::string fVideoClientName;

    // This is set to true when we have subscribed one module to the VideoDevice.
    bool fRegisteredToVideoDevice;
    int time;

    cv::Mat fcurrImageHeader;
    cv::Mat fprevImageHeader;

    pthread_t motionThreadId, cameraThreadId;

    int pResolution;
    int pColorSpace;

    PotentialField* thePotentialField;
    Controller* theController;

};

} // namespace AL

#endif // AmrModule_H
