#include <iostream>
#include <boost/assign/std/vector.hpp>

#include <alcommon/albroker.h>
#include <alvision/alvisiondefinitions.h>
#include <alvision/alimage.h>
#include <qi/log.hpp>

#include "r2Module.h"
#include "configReader.h"

/**
  TODO:
    - check transformations
    - task formalization
*/

#define INFO(x) std::cerr << "\033[22;34;1m" << "[r2module] " << x << "\033[0m" << std::endl;
//#define DEBUG_MODE
#define TEST_KINCHAIN

// Nao joints number
#define JOINTS_NUM 24
// Nao kinematic chain sizes
#define HEAD 2
#define R_ARM 5
#define L_ARM 5
#define R_LEG 6
#define L_LEG 6

// Position task control gain
#define K_HEAD 1
// Discrete integration time step
#define TIME_STEP 1

namespace AL
{

// Config (.cfg) files directory and paths
static const std::string CONFIG_PATH = "/../config/";
static const std::string JOINT_BOUNDS_CFG = "joints_params.cfg";
static const std::string LEFT_LEG_CFG = "dh_leftLeg.cfg";
static const std::string LEFT_ARM_CFG = "dh_leftArm.cfg";
static const std::string RIGHT_LEG_CFG = "dh_rightLeg.cfg";
static const std::string RIGHT_ARM_CFG = "dh_rightArm.cfg";
static const std::string HEAD_CFG = "dh_head.cfg";

#ifdef TEST_KINCHAIN
static Rmath::KinChain* theKinChain_TMP;
static Eigen::Matrix4d H_TMP;
static Eigen::MatrixXd J_TMP;
static Eigen::VectorXd q_TMP(L_ARM);
static Eigen::Vector3d r;
#endif

/*
 * Module constructor using Naoqi API methods.
 */
R2Module::R2Module( boost::shared_ptr<ALBroker> pBroker, const std::string& pName ):
  ALModule(pBroker , pName), fRegisteredToVideoDevice(false)
{
  // Module description
  setModuleDescription("Robotics2 Project");

  // Bound methods definition and description
  functionName( "registerToVideoDevice", getName(), "Register to the V.I.M." );
  BIND_METHOD( R2Module::registerToVideoDevice );

  functionName( "unRegisterFromVideoDevice", getName(), "Unregister from the V.I.M." );
  BIND_METHOD( R2Module::unRegisterFromVideoDevice );

  functionName( "getCurrentFrame", getName(), "Save an image received from the camera." );
  BIND_METHOD( R2Module::getCurrentFrame );

#ifdef DEBUG_MODE
  INFO("Module generated.");
#endif
}

/*
 * Module destructor: unsubscribes proxies, destroys threads and deallocates all memory used
 */
R2Module::~R2Module()
{
    try
    {
        // Unregister the video module
        if(fCamProxy) fCamProxy->unsubscribe(fVideoClientName);
        // Reset all proxies
        fCamProxy.reset();
        fMotionProxy.reset();
        fPostureProxy.reset();
    }
    catch(const AL::ALError& e)
    {
        qiLogError("vision.R2Module") <<  e.toString() << std::endl;
    }

    // Destroy thread
    pthread_cancel( motionThreadId );
    pthread_cancel( cameraThreadId );

    // Deallocate memory
    if (jointLimits) delete jointLimits;
    // Cleaning up containers
    std::map<std::string, Task*>::iterator destroyer = taskMap.begin();
    while (destroyer != taskMap.end())
    {
        if(destroyer->second) delete destroyer->second;
        ++destroyer;
    }
    taskMap.clear();
    taskSet.clear();
}

/*
 * Method automatically called when the module is created:
 * Initializes threads, proxy interfaces along with the private members of the module itself.
 */
void R2Module::init()
{
  // Initialize the base kinematic chain (from Nao's left foot to body center)
  ConfigReader theConfigReader( CONFIG_PATH + LEFT_LEG_CFG, CONFIG_PATH + JOINT_BOUNDS_CFG );
  theConfigReader.storeJointsID(&jointID);
  theKinChainLeftLeg = new Rmath::KinChain( theConfigReader, HEAD + R_ARM + L_ARM + R_LEG,
                                            HEAD + R_ARM + L_ARM + R_LEG + L_LEG);

#ifdef TEST_KINCHAIN
  // Initialize the test kinematic chain (left arm)
  ConfigReader theConfigReader_TMP( CONFIG_PATH + LEFT_ARM_CFG, CONFIG_PATH + JOINT_BOUNDS_CFG );
  theKinChain_TMP = new Rmath::KinChain( theConfigReader_TMP, HEAD + R_ARM, HEAD + R_ARM + L_ARM);
#endif

#ifdef DEBUG_MODE
  INFO("Initializing: building up base kinematic chain (left foot to body center)... ");
#endif

  /* ---------------------------------------- Proxies and threads initialization -------------------------------------------- */

  // Create a proxy for ALVideoDevice, ALMotion and ALRobotPosture
  try
  {
      fCamProxy = boost::shared_ptr<ALVideoDeviceProxy>(new ALVideoDeviceProxy(getParentBroker()));
      fMotionProxy = boost::shared_ptr<ALMotionProxy>(new ALMotionProxy(getParentBroker()));
      fPostureProxy = boost::shared_ptr<ALRobotPostureProxy>(new ALRobotPostureProxy(getParentBroker()));
  }
  catch (const AL::ALError& e)
  {
      qiLogError("vision.R2Module") << "Error while getting proxy on ALVideoDevice.  Error msg: " << e.toString() << std::endl;
      R2Module::exit();
      return;
  }

  // Check whether the proxy has been correctly initialized
  if(!fCamProxy || !fMotionProxy || !fPostureProxy)
  {
      qiLogError("vision.AmrModule") << "Error while getting proxy on ALVideoDevice. Check ALVideoDevice is running." << std::endl;
      R2Module::exit();
      return;
  }

  // Initialize resolution, image size and color space
  pResolution = AL::kQVGA;
  pColorSpace = AL::kYuvColorSpace;
  int imgWidth = 0, imgHeight = 0;
  setSizeFromResolution(pResolution, imgWidth, imgHeight);
  // Subscrive to the camera
  registerToVideoDevice();
  if(fRegisteredToVideoDevice)
  {
      // Initialize image
      fcurrImageHeader = cv::Mat::zeros(imgHeight, imgWidth, CV_8UC1);

      // Create the camera thread
      pthread_create( &cameraThreadId, NULL, (void*(*)(void*)) visionTh, this);
      // Create the motion thread
      pthread_create( &motionThreadId, NULL, (void*(*)(void*)) motionTh, this);

#ifdef DEBUG_MODE
      INFO("Proxies and threads correctly initialized...");
#endif

  /* -------------------------- First and foremost task (top priority): joint limits -------------------------------------- */


  // Extract joint bounds from the configuration file
  jointBounds = Eigen::MatrixXd::Zero(JOINTS_NUM, 2);
  theConfigReader.extractJointBounds(&jointBounds);

  // Convert joint ranges into velocity bound estimates
  Eigen::MatrixXd velBounds(JOINTS_NUM,2);
  posBound2velBound(jointBounds, getConfiguration(), &velBounds);

  // Actual task initialization
  jointLimits = new TaskBase(0, Eigen::MatrixXd::Identity( JOINTS_NUM, JOINTS_NUM ), velBounds);

#ifdef DEBUG_MODE
  INFO("Initializing: retrieving joint limits... ");
#endif

  /* ------------------------------ Other tasks: head (priority = 1), left arm (priority = 2) ------------------------------ */

  // Extract joint bounds from the configuration file
  ConfigReader headConfig( CONFIG_PATH + HEAD_CFG, CONFIG_PATH + JOINT_BOUNDS_CFG );
  // Task initialization
  Task* head = new Task(3, HEAD, 1, headConfig, 0, HEAD);

  // Extract joint bounds from the configuration file
  ConfigReader pointingConfig( CONFIG_PATH + LEFT_ARM_CFG, CONFIG_PATH + JOINT_BOUNDS_CFG );
  // Task initialization
  Task* pointing = new Task(3, L_ARM, 2, pointingConfig, HEAD + R_ARM, HEAD + R_ARM + L_ARM);

  // Push every task into the task map
  taskMap.insert( std::pair<std::string, Task*> ("Head task", head) );
  taskMap.insert( std::pair<std::string, Task*> ("Left arm task", pointing) );

#ifdef DEBUG_MODE
  INFO("Initializing tasks: ");
  INFO("- Joint limits, with priority " << jointLimits->getPriority());
  INFO("- Head task, with priority " << head->getPriority());
  INFO("- Left arm task, with priority " << pointing->getPriority());
#endif
  }

}

/*
 * Method automatically called when the module is destroyed.
 */
void R2Module::exit()
{
  AL::ALModule::exit();
}

/*
 * Register/unregister to the camera proxy.
 */
void R2Module::registerToVideoDevice()
{
  // Check if another module has subscribed already
  if (fRegisteredToVideoDevice)
  {
    throw ALError(getName(), "registerToVideoDevice()", "A video module has already been "
      "registered. Call unRegisterFromVideoDevice() before trying to register a new module.");
  }
  // Release any image header previously allocated.
  if (!fcurrImageHeader.empty()) fcurrImageHeader.release();

  if(fCamProxy)
      fVideoClientName = fCamProxy->subscribeCamera("r2Module", 1, pResolution, pColorSpace, 30);
  qiLogInfo("vision.r2Module") << "Module registered as " << fVideoClientName << std::endl;

  // Registration successful, set flag to true
  fRegisteredToVideoDevice = true;
}

void R2Module::unRegisterFromVideoDevice()
{
    // Check if the module is actually registered
    if (!fRegisteredToVideoDevice)
    {
        throw ALError(getName(), "unRegisterFromVideoDevice()", "No video module is currently "
                      "registered! Call registerToVideoDevice first.");
    }
    // Release any image header previously allocated.
    if (!fcurrImageHeader.empty()) fcurrImageHeader.release();

    qiLogInfo("vision.R2Module") << "Unregister " << fVideoClientName << " module..." << std::endl;
    if(fCamProxy)
        fCamProxy->unsubscribe(fVideoClientName);
    qiLogInfo("vision.R2Module") << "Done." << std::endl;

  // Unregistration successful, set flag to false.
    fRegisteredToVideoDevice = false;
}

/*
 * Retrieve the current camera image.
 */
void R2Module::getCurrentFrame()
{
  // Check if a video module is actually registered
  if (!fRegisteredToVideoDevice)
    throw ALError(getName(), "saveImageRemote()",  "No video module is currently "
      "registered! Call registerToVideoDevice() first.");

  // Image request via the camera proxy
  ALValue frame = fCamProxy->getImageRemote(fVideoClientName);

  // Check for correctness of the camera frame
  if (frame.getType()!= ALValue::TypeArray && frame.getSize() != 7)
    throw ALError(getName(), "getImages", "Invalid image returned.");

  // Put a time stamp on the image header
  const long long timeStamp = ((long long)(int)frame[4])*1000000LL + ((long long)(int)frame[5]);
  time = (int)(timeStamp/1000000LL);
  // Retrieve image data
  fcurrImageHeader.data = (uchar*) frame[6].GetBinary();

  // Prompt the image in a window
  if ((char) cv::waitKey(33) != 27)
      cv::imshow("Frame", fcurrImageHeader);

  // Release the proxy
  fCamProxy->releaseImage(fVideoClientName);
}

/*
 * The vision process.
 */
void R2Module::vision()
{
    // Wait a while before starting acquisition
    qi::os::sleep(1.5);
    // Main loop
    while(true)
    {
        qi::os::msleep(100);
        /* WORK IN PROGRESS */
    }
}

/*
 * The motion process:
 * - At each time step, the current robot configuration is retrieved to update all the robot tasks;
 * - Active tasks are then pushed into a priority-ordered set, so to generate a task hierarchy for the HQP solver;
 * - Once the solver is configured, active search is enabled and a feasible solution is sought;
 * - The solution obtained for q_dot is integrated in discrete time and eventually used to update the robot configuration.
 */
void R2Module::motion()
{
    // Initialize the posture proxy
    fPostureProxy->goToPosture("StandInit", 0.5f);

#ifdef DEBUG_MODE
    INFO("Posture proxy initialized.");
    INFO("Turning tasks to 'active'... ");
    INFO("Executing main loop...");
#endif

    // Turn tasks to active
    jointLimits->activate();
    taskMap["Head task"]->activate();
    taskMap["Left arm task"]->activate();

    // Main loop
    while(true)
    {
        // Retrieve current robot configuration
        Eigen::VectorXd q(jointID.size());
        q = getConfiguration();

#ifdef DEBUG_MODE
        INFO("Current joint configuration:");
        for(unsigned int i=0; i<q.size(); ++i)
            INFO(jointID.at(i) << "\t" << q(i));
#endif

        // Update all tasks
        updateConstraints(q);

        // Push active tasks into the ordered set
        taskSet.clear();
        std::map<std::string, Task*>::const_iterator adder;
        for ( adder = taskMap.begin(); adder != taskMap.end(); ++adder )
            if( (adder->second)->isActive())
                taskSet.insert(adder->second);

        // Initialize the HQP solver
        soth::HCOD hsolver( q.size(), taskSet.size()+1 );
        Eigen::VectorXd qdot(q.size());

#ifdef DEBUG_MODE
        INFO("Initializing HQP solver...");
        INFO( "Active tasks: " << (taskSet.size()+1) << " out of " << (taskMap.size()+1) );
        INFO("Generating stack...");
#endif
        // Set up the hierarchy
        std::vector<Eigen::MatrixXd> A;
        std::vector<soth::VectorBound> b;
        // Joint limits first (top priority task)
        A.push_back(jointLimits->constraintMatrix());
        b.push_back(jointLimits->vectorBounds());
        // All the remaining tasks (in descending order of priority)
        std::set<TaskBase*>::iterator updater = taskSet.begin();
        while( updater != taskSet.end() )
        {
            A.push_back((*updater)->constraintMatrix());
            b.push_back((*updater)->vectorBounds());
            ++updater;
        }
        hsolver.pushBackStages(A, b);

        // Configure the solver
        hsolver.setDamping(0.0);
        hsolver.setInitialActiveSet();

#ifdef DEBUG_MODE
        INFO("Configuring HQP solver...");
        INFO("Start active search... ");
#endif
//        // Run the solver
//        hsolver.activeSearch(qdot);
//        // Trim ssolution
//        Rmath::trim(&qdot);

//#ifdef DEBUG_MODE
//        INFO("Done.");
//        INFO("Active set: ");
//        hsolver.showActiveSet(std::cerr);
//        INFO("Solution: [\n" << qdot << "]");
//#endif

#ifdef TEST_KINCHAIN
        qdot = Eigen::VectorXd::Zero(q.size());
        Eigen::MatrixXd J_TMP_pinv;
        Rmath::pseudoInverse(J_TMP, &J_TMP_pinv);
        qdot.segment<L_ARM>(HEAD + R_ARM) = J_TMP_pinv * r;
        INFO("Solution: [\n" << qdot << "]");
#endif

        // Update the robot configuration with the obtained solution
        updateConfiguration(q + qdot*TIME_STEP);

    }
}

/*
 * Retrieve the current joint configuration with the motion proxy.
 */
Eigen::VectorXd R2Module::getConfiguration()
{
    // Store the resulting values in a Eigen::VectorXd
    Eigen::VectorXd currentConfiguration(jointID.size());

    // Use Naoqi ID for each joint to request the joint current measured value
    for(int i=0; i <jointID.size(); ++i)
        currentConfiguration(i) = ( fMotionProxy->getAngles(jointID.at(i), true) ).at(0);

    return currentConfiguration;
}

/*
 * Update every active task with the current joint configuration.
 */
void R2Module::updateConstraints(const Eigen::VectorXd& q)
{
    // Updating the base kinematic chain (left foot to body center)
    Eigen::Matrix4d H_base;
    Eigen::MatrixXd J_base(6, L_LEG);
    // Left leg joints are those on the bottom part of the IDs vector
    theKinChainLeftLeg->update(q.tail(L_LEG));
    // Compute kinematics
    theKinChainLeftLeg->forward(&H_base);
    theKinChainLeftLeg->differential(&J_base);

    /* ------------------------- Joint bounds update -------------------------*/

    Eigen::MatrixXd velBounds(q.size(), 2);
    posBound2velBound(jointBounds, q, &velBounds);
    jointLimits->setVectorBounds(velBounds);

#ifdef DEBUG_MODE
    INFO("Updating joint bounds: ");
    INFO(std::endl << *jointLimits);
#endif

#ifdef TEST_KINCHAIN
    INFO("Updating test kinematic chain (body to left hand): ");
    q_TMP << q.segment<L_ARM>(HEAD + R_ARM);
    INFO("Joint configuration:");
    for(unsigned int i=0; i<q_TMP.size(); ++i)
        INFO(jointID.at(HEAD + R_ARM + i) << "\t" << q_TMP(i));
    theKinChain_TMP->update(q_TMP);
    theKinChain_TMP->forward(&H_TMP);
    theKinChain_TMP->differential(&J_TMP, 3);
    INFO("Direct kinematics transform: ");
    INFO(std::endl << H_TMP);
    INFO("Jacobian (linear velocities only): ");
    INFO(std::endl << J_TMP);
#endif

    /* -------------------- Head and left arm task update --------------------*/

    // Defining the desired pose vectors in the task space
    Eigen::VectorXd desiredHeadPose(3), desiredLHandPose(3);

    //desiredHeadPose << 53.9, 0.0, 194.4; // zero-position HEAD
    desiredHeadPose << -20.0, 0.0, 194.4;

    //desiredLHandPose << 218.7, 133, 112.31; // zero-position L ARM
//    desiredLHandPose << 0.0, 329.01, 112.31; // point-left L ARM
    desiredLHandPose << 15.1, 112.31, 316.0; // point-up L ARM

    //desiredRHandPose << 218.7, -133, 112.31, 0.0, 0.0, 0.0; //zero-position R ARM
    //desiredRHandPose <<-15, -329.01, 112.31, 0.0, 0.0, 0.0; //point-right R ARM

    // Head task update
    ( taskMap["Head task"] )->update( q.head(HEAD), K_HEAD, desiredHeadPose, Eigen::VectorXd::Zero(3) );

#ifdef DEBUG_MODE
    INFO("Desired head position in space: [\n" << desiredHeadPose << "]");
    INFO("Head task constraint equation: ");
    INFO(std::endl << *( taskMap["Head task"] ) );
#endif

    // Left arm task update
    ( taskMap["Left arm task"] )->update( q.segment<L_ARM>(HEAD+R_ARM), K_HEAD, desiredLHandPose, Eigen::VectorXd::Zero(3) );

#ifdef DEBUG_MODE
    INFO("Desired left hand position in space: [\n" << desiredLHandPose << "]");
    INFO("Left arm task constraint equation: ");
    INFO(std::endl << *( taskMap["Left arm task"] ) );
#endif

#ifdef TEST_KINCHAIN
    INFO("Desired left hand position in space: [\n" << desiredLHandPose << "]");
    r = K_HEAD * ( desiredLHandPose - H_TMP.topRightCorner(3,1) );
    INFO("New target velocity:\n[" << r << "]");
#endif

}

/*
 * Call Naoqi motion proxy to actually execute the motion.
 */
bool R2Module::updateConfiguration(const Eigen::VectorXd& delta_q)
{
#ifdef TEST_KINCHAIN
    Eigen::VectorXd delta_q_ = delta_q;
#endif

    // Initialize variables
    AL::ALValue jointId, angles, setStiff, zeroArray;
    // Push into the vector all joint value increments to be requested
    for(int i=0; i< delta_q.size(); ++i)
    {

#ifdef TEST_KINCHAIN
        if (delta_q_(i) > 0)
        {
            if ( delta_q_(i) > jointBounds(i,1) )
                delta_q_(i) = jointBounds(i,1);
        }
        else
        {
            if ( delta_q_(i) < jointBounds(i,0) )
                delta_q_(i) = jointBounds(i,0);
        }

        if(delta_q_(i) != 0)
        {
            jointId.arrayPush( jointID.at(i) );
            angles.arrayPush( delta_q_(i) );
            setStiff.arrayPush( 1.0 );
            zeroArray.arrayPush( 0.0 );
        }
        continue;
#endif
        if(delta_q(i) != 0)
        {
            jointId.arrayPush( jointID.at(i) );
            angles.arrayPush( delta_q(i) );
            setStiff.arrayPush( 1.0 );
            zeroArray.arrayPush( 0.0 );
        }
    }
    // Set up joint stiffness
    fMotionProxy->stiffnessInterpolation( jointId, setStiff, 0.3f );
    fMotionProxy->setStiffnesses(jointId, setStiff);

    // Request motion to the motion proxy
    fMotionProxy->setAngles(jointId, angles, 0.3f);

    // Reset stiffness
    fMotionProxy->setStiffnesses(jointId, zeroArray);
}

/*
 * Estimate the joint velocity bounds given the actual joint limits and the current configuration.
 */
void R2Module::posBound2velBound( const Eigen::MatrixXd& posBound, const Eigen::VectorXd& configuration, Eigen::MatrixXd* velBound)
{
    Eigen::MatrixXd currConf(configuration.size(), 2);
    currConf << configuration, configuration;

    *velBound = (posBound - currConf)/TIME_STEP;
}


} // namespace AL
