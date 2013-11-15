#ifndef R2Module_H
#define R2Module_H

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <string>
#include <set>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <alproxies/alvideodeviceproxy.h>
#include <alcommon/almodule.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alcommon/albrokermanager.h>
#include <alcore/alptr.h>
#include <alcommon/altoolsmain.h>

#include <soth/debug.hpp>
#include <soth/HCOD.hpp>
#include <soth/Random.hpp>
#include <soth/DestructiveColPivQR.hpp>

#include "libmath/kinChain.h"
#include "libmath/Rutils.h"
#include "task.h"

namespace AL
{

class R2Module : public AL::ALModule
{
public:
    R2Module( boost::shared_ptr<ALBroker> pBroker, const std::string& pName);

    virtual ~R2Module();
    void exit();

    void registerToVideoDevice();
    void unRegisterFromVideoDevice();

    static void* motionTh(R2Module* module){ module->motion(); return 0; }
    static void* visionTh(R2Module* module){ module->vision(); return 0; }

    void motion();
    void vision();

    void updateConstraints(const Eigen::VectorXd& q);
    bool updateConfiguration(const Eigen::VectorXd &delta_q);
    Eigen::VectorXd getConfiguration();

    void getCurrentFrame();
    void posBound2velBound( const Eigen::MatrixXd& posBound, const Eigen::VectorXd& configuration, Eigen::MatrixXd* velBound);

    // Automatically called right after the module is created.
    virtual void init();

private:

    // Proxy to the video and motion input module.
    boost::shared_ptr<AL::ALVideoDeviceProxy> fCamProxy;
    boost::shared_ptr<AL::ALMotionProxy> fMotionProxy;
    boost::shared_ptr<AL::ALRobotPostureProxy> fPostureProxy;

    // Client name that is used to talk to the Video Device.
    std::string fVideoClientName;

    // This is set to true when we have subscribed one module to the VideoDevice.
    bool fRegisteredToVideoDevice;
    int time;
    
    cv::Mat fcurrImageHeader;

    pthread_t motionThreadId, cameraThreadId;
    
    int pColorSpace;
    int pResolution;

    std::vector<std::string> jointID;
    Eigen::MatrixXd jointBounds;

    std::set<TaskBase*, taskCmp> taskSet;
    TaskBase* jointLimits;
    Task* pointing;
    Task* head;

    Rmath::KinChain* theKinChainLeftLeg;
    Rmath::KinChain* theKinChain_TMP;

};

} // namespace AL

#endif // AmrModule_H
