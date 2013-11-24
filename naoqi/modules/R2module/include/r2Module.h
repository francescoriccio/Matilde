#ifndef R2Module_H
#define R2Module_H

// C++ includes
#include <string>
#include <map>
#include <set>

// Boost includes
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// Naoqi includes
#include <alcore/alptr.h>
#include <alcommon/almodule.h>
#include <alcommon/altoolsmain.h>
#include <alcommon/albrokermanager.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/alvideodeviceproxy.h>

// Soth includes
#include <soth/HCOD.hpp>

// Libmath includes
#include "libmath/kinChain.h"
#include "libmath/Rutils.h"

#include "task.h"

namespace AL
{


/** --------------------------------- R2 module declaration --------------------------------- */


class R2Module : public AL::ALModule
{

private:

    // Motion thread
    pthread_t motionThreadId;
    // Vision thread
    pthread_t cameraThreadId;

    // Proxy for the video/motion input module.
    boost::shared_ptr<AL::ALVideoDeviceProxy> fCamProxy;
    boost::shared_ptr<AL::ALMotionProxy> fMotionProxy;
    boost::shared_ptr<AL::ALRobotPostureProxy> fPostureProxy;

    // Client name for the Video Device.
    std::string fVideoClientName;
    // Flag raised when the module has subscribed to the VideoDevice.
    bool fRegisteredToVideoDevice;
    // Current camera image
    cv::Mat fcurrImageHeader;
    // Camera parameters
    int pColorSpace;
    int pResolution;
    int time;

    // Container for Naoqi's joint IDs
    std::vector<std::string> jointID;
    // Container for the jointBounds
    Eigen::MatrixXd jointBounds;
    // Joint limits primary task
    TaskBase* jointLimits;

    // Common base kinematic chain (left foot to body center)
    Rmath::KinChain* theKinChainLeftLeg;
    // All initialized tasks
    std::map<std::string, Task*> taskMap;
    // Stack of tasks currently active
    std::set<TaskBase*, taskCmp> taskSet;

public:

    // Constructor
    R2Module( boost::shared_ptr<ALBroker> pBroker, const std::string& pName);
    // Destructor
    virtual ~R2Module();

    // Automatically called right after the module is created and destroyed
    virtual void init();
    void exit();

    // Register/unregister to the camera proxy
    void registerToVideoDevice();
    void unRegisterFromVideoDevice();
    // Retrieve the current camera image
    void getCurrentFrame();

    // Motion and vision thread interacting with Naoqi
    static void* motionTh(R2Module* module){ module->motion(); return 0; }
    static void* visionTh(R2Module* module){ module->vision(); return 0; }
    void motion();
    void vision();

    // Retrieve the current joint configuration
    Eigen::VectorXd getConfiguration();
    // Update every active task with the current joint configuration
    void updateConstraints(const Eigen::VectorXd& q);
    // Call Naoqi motion proxy to actually execute the motion
    bool updateConfiguration(const Eigen::VectorXd &delta_q);

    // Estimate the joint velocity bounds given the actual joint limits and the current configuration
    void posBound2velBound( const Eigen::MatrixXd& posBound, const Eigen::VectorXd& configuration, Eigen::MatrixXd* velBound);

};

} // namespace AL

#endif // AmrModule_H
