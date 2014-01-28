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
#include <alproxies/alredballtrackerproxy.h>

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
    boost::shared_ptr<AL::ALRedBallTrackerProxy> fBallTrackerProxy;

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

    // Task handler and scheduler
    class TaskManager
    {

    private:

        // Pointer to the robot module
        R2Module* module_ptr;

        // Joint limits primary task
        TaskBase* jointLimits;
        // All initialized tasks
        std::map <std::string, Task*> taskMap;
        // Stack of tasks currently active
        std::set<Task*, taskCmp> taskSet;
        // Set of tasks changing their priority value
        std::map<std::string, int> changingTaskMap;

        // Update a single task
        void taskUpdate( const std::string& taskName, const Eigen::VectorXd& q );
        // Update all tasks fully enabled
        void updateActiveTasks( const Eigen::VectorXd& q );
        // Update all tasks in a transition phase
        void updateIntermediateTasks( const Eigen::VectorXd& q, const Eigen::VectorXd& partial_qdot );

        // Compute a partial HQP solution involving only fully enabled tasks
        void computePartialSolution(const Eigen::VectorXd& q, Eigen::VectorXd* partial_qdot);
        // Compute the final HQP involving all enabled tasks
        void computeCompleteSolution(const Eigen::VectorXd& q, const Eigen::VectorXd& partial_qdot, Eigen::VectorXd* qdot);

    public:

        // Constructor
        TaskManager( R2Module* m_ptr ): module_ptr(m_ptr){}
        // Destructor
        ~TaskManager();

        TaskBase* Rbound;

        // Set joint limits
        void setJointLimits(const Eigen::MatrixXd& velBounds);
        // Declare a new task
        inline void createTask(const std::string& taskName, Task* taskPtr)
        {
            taskMap.insert( std::pair<std::string, Task*>(taskName,taskPtr) );
        }
        // Retrieve a reference to specific task
        inline Task& task(const std::string& taskName)
        {
            assert(taskMap.find(taskName) != taskMap.end());
            return *(taskMap[taskName]);
        }
        // Change task priorities
        inline void changePriority(const std::string& taskName, int new_priority)
        {
            // Legal priority value check
            assert(new_priority > 0);
            assert(taskMap.find(taskName) != taskMap.end());

            if ( (taskMap[taskName]->getPriority() != new_priority) &&
                 (changingTaskMap.find(taskName) == changingTaskMap.end()) )
                changingTaskMap.insert( std::pair<std::string,int>(taskName, new_priority) );
        }

        // Solve the HQP problem with the current task set
        void exec(const Eigen::VectorXd& q, Eigen::VectorXd* qdot);
    };
    TaskManager taskManager;


    // Common base kinematic chain (left foot to body center)
    Rmath::KinChain* theKinChain_LLBase;
    Rmath::KinChain* theKinChain_RLBase;

    Rmath::KinChain* theKinChainLeftLeg;
    Rmath::KinChain* theKinChainRightLeg;
    Rmath::KinChain* theKinChainLeftArm;
    Rmath::KinChain* theKinChainRightArm;
    Rmath::KinChain* theKinChainHead;

    Rmath::KinChain* CoM_LeftLeg;
    Rmath::KinChain* CoM_RightLeg;
    Rmath::KinChain* CoM_LeftArm;
    Rmath::KinChain* CoM_RightArm;
    Rmath::KinChain* CoM_Head;

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

    void closeHand(const std::string& handID);
    void openHand(const std::string& handID);

    Eigen::Vector3d getRedBallPosition();

    // Retrieve the current joint configuration
    Eigen::VectorXd getConfiguration();
    // Call Naoqi motion proxy to actually execute the motion
    bool updateConfiguration(const Eigen::VectorXd &delta_q);

    // Estimate the joint velocity bounds given the actual joint limits and the current configuration
    void posBound2velBound( const Eigen::MatrixXd& posBound, const Eigen::VectorXd& configuration, Eigen::MatrixXd* velBound);

};

} // namespace AL

#endif // AmrModule_H
