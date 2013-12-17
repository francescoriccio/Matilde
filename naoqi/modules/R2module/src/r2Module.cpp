#include <iostream>
#include <boost/assign/std/vector.hpp>

#include <alcommon/albroker.h>
#include <alvision/alvisiondefinitions.h>
#include <alvision/alimage.h>
#include <qi/log.hpp>

#include "r2Module.h"
#include "configReader.h"
#include "configParams.h"

/**
  TODO:
    - rearraging priorities
    - Simulations
*/

#define INFO(x) std::cerr << "\033[22;34;1m" << "[r2module] " << x << "\033[0m" << std::endl;

namespace AL
{

/*
 * Module constructor using Naoqi API methods.
 */
R2Module::R2Module( boost::shared_ptr<ALBroker> pBroker, const std::string& pName ):
    ALModule(pBroker , pName), fRegisteredToVideoDevice(false), taskManager(this)
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
}

/*
 * Method automatically called when the module is created:
 * Initializes threads, proxy interfaces along with the private members of the module itself.
 */
void R2Module::init()
{
    // Static initialization of parameters
    initialization();

    ConfigReader theConfigReader(CONFIG_PATH + JOINT_BOUNDS_CFG);
    theConfigReader.storeJointsID(&jointID);

    // Initialize the base kinematic chain (from Nao's left foot to body center)
    theConfigReader.setKinChain(CONFIG_PATH + LEFT_LEG_BASE_CFG);
    theKinChain_LLBase = new Rmath::KinChain( theConfigReader );

    // Initialize the base kinematic chain (from Nao's right foot to body center)
    theConfigReader.setKinChain(CONFIG_PATH + RIGHT_LEG_BASE_CFG);
    theKinChain_RLBase = new Rmath::KinChain( theConfigReader );

    // initialize kinChain transofrmation w.r.t. CoM
    theConfigReader.setKinChain(CONFIG_PATH + COM_HEAD_CFG);
    CoM_Head = new Rmath::KinChain( theConfigReader );
    theConfigReader.setKinChain(CONFIG_PATH + COM_LEFT_ARM_CFG);
    CoM_LeftArm = new Rmath::KinChain( theConfigReader );
    theConfigReader.setKinChain(CONFIG_PATH + COM_RIGHT_ARM_CFG);
    CoM_RightArm = new Rmath::KinChain( theConfigReader );
    theConfigReader.setKinChain(CONFIG_PATH + COM_RIGHT_LEG_CFG);
    CoM_RightLeg= new Rmath::KinChain( theConfigReader );
    theConfigReader.setKinChain(CONFIG_PATH + COM_LEFT_LEG_CFG);
    CoM_LeftLeg = new Rmath::KinChain( theConfigReader );

    // Initialize kinChains
    theConfigReader.setKinChain(CONFIG_PATH + LEFT_LEG_CFG);
    theKinChainLeftLeg = new Rmath::KinChain( theConfigReader );
    theConfigReader.setKinChain(CONFIG_PATH + RIGHT_LEG_CFG);
    theKinChainRightLeg = new Rmath::KinChain( theConfigReader );
    theConfigReader.setKinChain(CONFIG_PATH + LEFT_ARM_CFG);
    theKinChainLeftArm = new Rmath::KinChain( theConfigReader );
    theConfigReader.setKinChain(CONFIG_PATH + RIGHT_ARM_CFG);
    theKinChainRightArm = new Rmath::KinChain( theConfigReader );
    theConfigReader.setKinChain(CONFIG_PATH + HEAD_CFG);
    theKinChainHead = new Rmath::KinChain( theConfigReader );

#ifdef DEBUG_MODE
    INFO("Initializing: building up kinematic chains... ");
#endif

  /* ---------------------------------------- Proxies and threads initialization -------------------------------------------- */

  // Create a proxy for ALVideoDevice, ALMotion and ALRobotPosture
  try
  {
      fCamProxy = boost::shared_ptr<ALVideoDeviceProxy>(new ALVideoDeviceProxy(getParentBroker()));
      fMotionProxy = boost::shared_ptr<ALMotionProxy>(new ALMotionProxy(getParentBroker()));
      fPostureProxy = boost::shared_ptr<ALRobotPostureProxy>(new ALRobotPostureProxy(getParentBroker()));
      fBallTrackerProxy = boost::shared_ptr<AL::ALRedBallTrackerProxy>(new ALRedBallTrackerProxy(getParentBroker()));
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
  Eigen::MatrixXd velBounds(JOINTS_NUM, 2);
  posBound2velBound(jointBounds, getConfiguration(), &velBounds);

  // Actual task initialization
  taskManager.setJointLimits(velBounds);

#ifdef DEBUG_MODE
  INFO("Initializing: retrieving joint limits... ");
#endif

  /* ------------------------------------------------- Other tasks -------------------------------------------------------- */


  // Task initialization
  Task* Rsupporting = new Task(RLEG_TASK_DIM, RLEG_CHAIN_SIZE, RLEG_TASK_PRIORITY, *theKinChainRightLeg);

  Task* Lsupporting = new Task(LLEG_TASK_DIM, LLEG_CHAIN_SIZE, LLEG_TASK_PRIORITY, *theKinChainLeftLeg);

  Task* Lpointing = new Task(LARM_TASK_DIM, LARM_CHAIN_SIZE+LLEG_CHAIN_SIZE,
                             LARM_TASK_PRIORITY, *theKinChain_LLBase + *CoM_LeftArm + *theKinChainLeftArm,
                             LLEG_CHAIN_SIZE);

  Task* Rpointing = new Task(RARM_TASK_DIM, RARM_CHAIN_SIZE+LLEG_CHAIN_SIZE,
                             RARM_TASK_PRIORITY, *theKinChain_LLBase + *CoM_RightArm + *theKinChainRightArm,
                             LLEG_CHAIN_SIZE);

  Task* looking = new Task(HEAD_TASK_DIM, HEAD_CHAIN_SIZE+LLEG_CHAIN_SIZE,
                             HEAD_TASK_PRIORITY, *theKinChain_LLBase + *CoM_Head + *theKinChainHead,
                             LLEG_CHAIN_SIZE);

  //  // Push every task into the task map
  taskManager.createTask(HEAD_TASK, looking);
  taskManager.createTask(RIGHT_ARM, Rpointing);
  taskManager.createTask(LEFT_ARM, Lpointing);
  taskManager.createTask(RIGHT_LEG, Rsupporting);
  taskManager.createTask(LEFT_LEG, Lsupporting);

#ifdef DEBUG_MODE
  INFO("Initializing tasks: ");
  INFO("- Head task, with priority " << looking->getPriority());
  INFO("- Right arm task, with priority " << Rpointing->getPriority());
  INFO("- Left arm task, with priority " << Lpointing->getPriority());
  INFO("- Right leg task, with priority " << Rsupporting->getPriority());
  INFO("- Left leg task, with priority " << Lsupporting->getPriority());
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

  // Release the proxystd::cout<<"jesus: \n"<<desiredLHandPose<<std::endl;
  fCamProxy->releaseImage(fVideoClientName);
}

/*
 * The vision process.
 */
void R2Module::vision()
{
//    fBallTrackerProxy->startTracker();
    // Wait a while before starting acquisition
    qi::os::sleep(1.5);
    // Main loop
    while(true)
    {
        qi::os::msleep(100);
        /* WORK IN PROGRESS */
//        INFO("ball position w.r.t. CoM: \n"<<getRedBallPosition()); /*TODEBUG*/
    }
//    fBallTrackerProxy->stopTracker();
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
    taskManager.task(HEAD_TASK).activate(HEAD_TRANSITION_STEP);
    taskManager.task(RIGHT_ARM).activate(RARM_TRANSITION_STEP);
    taskManager.task(LEFT_ARM).activate(LARM_TRANSITION_STEP);
    taskManager.task(RIGHT_LEG).activate(RLEG_TRANSITION_STEP);
    taskManager.task(LEFT_LEG).activate(LLEG_TRANSITION_STEP);

    Eigen::Matrix4d h_CoMRL, h_CoMLL;
    CoM_RightLeg->forward(&h_CoMRL);
    CoM_LeftLeg->forward(&h_CoMLL);

    // Set fixed base transform for the tasks
    taskManager.task(RIGHT_LEG).set_baseTransform(h_CoMRL);
    taskManager.task(LEFT_LEG).set_baseTransform(h_CoMLL);
    taskManager.task(HEAD_TASK).set_baseTransform(base_ankle);
    taskManager.task(LEFT_ARM).set_baseTransform(base_ankle);
    taskManager.task(RIGHT_ARM).set_baseTransform(base_ankle);

    if(taskManager.task(RIGHT_LEG).taskStatus() != inactive)
        taskManager.task(RIGHT_LEG).setDesiredPose( desiredRLegPose.head(RLEG_TASK_DIM), RLEG_TASK_NSTEPS );

    if(taskManager.task(LEFT_LEG).taskStatus() != inactive)
        taskManager.task(LEFT_LEG).setDesiredPose( desiredLLegPose.head(LLEG_TASK_DIM), LLEG_TASK_NSTEPS );

    if(taskManager.task(HEAD_TASK).taskStatus() != inactive)
        taskManager.task(HEAD_TASK).setDesiredPose( desiredHeadPose.head(HEAD_TASK_DIM), HEAD_TASK_NSTEPS );

    if(taskManager.task(LEFT_ARM).taskStatus() != inactive)
#ifndef LARM_CIRCLE_TASK
        taskManager.task(LEFT_ARM).setDesiredPose( desiredLHandPose.head(LARM_TASK_DIM), LARM_TASK_NSTEPS );
#else
        taskManager.task(LEFT_ARM).circularPathGenerator(desiredLHandPose.head(LARM_TASK_DIM), CIRCLE_Z_DEPTH,
                                                        LARM_TASK_NSTEPS, CIRCLE_RADIUS, CIRCLE_LAPS );
#endif

#ifndef RARM_LARM_JOINT_TASK
    if(taskManager.task(RIGHT_ARM).taskStatus() != inactive)
#ifndef RARM_CIRCLE_TASK
        taskManager.task(RIGHT_ARM).setDesiredPose(desiredRHandPose.head(RARM_TASK_DIM), RARM_TASK_NSTEPS );
#else
        taskManager.task(RIGHT_ARM).circularPathGenerator(desiredRHandPose.head(RARM_TASK_DIM), CIRCLE_Z_DEPTH,
                                                         RARM_TASK_NSTEPS, CIRCLE_RADIUS, CIRCLE_LAPS );
#endif
#endif


#ifdef DEBUG_MODE
    INFO("Desired head position in space: [\n" << desiredHeadPose << "]");
    INFO("Desired left hand position in space: [\n" << desiredLHandPose << "]");
#ifndef RARM_LARM_JOINT_TASK
    INFO("Desired right hand position in space: [\n" << desiredRHandPose << "]");
#endif
    INFO("Desired right foot position in space: [\n" << desiredRLegPose << "]");
    INFO("Desired Hip position in space: [\n" << desiredLLegPose << "]");
#endif

    // Main loop
    while(true)
    {

#ifdef UP_DOWN_TASK
        if ( (taskManager.task(RIGHT_LEG).done()) && (taskManager.task(LEFT_LEG).done()) )
        {
            UP_DOWN *= -1;

            Eigen::VectorXd desiredRLegPose = taskManager.task(RIGHT_LEG).getTargetPose();
            Eigen::VectorXd desiredLLegPose = taskManager.task(LEFT_LEG).getTargetPose();
            desiredRLegPose(2) += UP_DOWN;
            desiredLLegPose(2) += UP_DOWN;
            taskManager.task(RIGHT_LEG).setDesiredPose(desiredRLegPose.head(RLEG_TASK_DIM), RLEG_TASK_NSTEPS);
            taskManager.task(LEFT_LEG).setDesiredPose(desiredLLegPose.head(LLEG_TASK_DIM), LLEG_TASK_NSTEPS);
        }
#endif
        // Retrieve current robot configuration
        Eigen::VectorXd q(JOINTS_NUM);
        q = getConfiguration();

    #ifdef DEBUG_MODE
        INFO("\nCurrent joint configuration:");
        for(unsigned int i = 0; i < q.size(); ++i)
            INFO(jointID.at(i) << "\t" << q(i));
    #endif

        // Solve the HQP problem with the current task set
        Eigen::VectorXd qdot(JOINTS_NUM);
        taskManager.exec(q, &qdot);

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

    AL::ALValue jointId, angles;

    // Use Naoqi ID for each joint to request the joint current measured value
    for(int i=0; i <jointID.size(); ++i)
        jointId.arrayPush( jointID.at(i) );

    angles = fMotionProxy->getAngles( jointId, true );

    for(int i=0; i <jointID.size(); ++i)
        currentConfiguration(i) = angles[i];

    return currentConfiguration;
}

/*
 * Call Naoqi motion proxy to actually execute the motion.
 */
bool R2Module::updateConfiguration(const Eigen::VectorXd& delta_q)
{
    // Initialize variables
    AL::ALValue jointId, angles, setStiff, zeroArray;
    // Push into the vector all joint value increments to be requested
    for(int i=0; i< delta_q.size(); ++i)
    {
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

void R2Module::closeHand(const std::string& handID)
{
    if(handID == "both")
    {
        fMotionProxy->setStiffnesses("LHand", 1.0);
        fMotionProxy->setStiffnesses("RHand", 1.0);
        fMotionProxy->closeHand("LHand");
        fMotionProxy->closeHand("RHand");
        fMotionProxy->setStiffnesses("LHand", 0.0);
        fMotionProxy->setStiffnesses("RHand", 0.0);
    }
    else
    {
        fMotionProxy->setStiffnesses(handID, 1.0);
        fMotionProxy->closeHand(handID);
        fMotionProxy->setStiffnesses(handID, 0.0);
    }

}

void R2Module::openHand(const std::string& handID)
{
    if(handID == "both")
    {
        fMotionProxy->setStiffnesses("LHand", 1.0);
        fMotionProxy->setStiffnesses("RHand", 1.0);
        fMotionProxy->openHand("LHand");
        fMotionProxy->openHand("RHand");
        fMotionProxy->setStiffnesses("LHand", 0.0);
        fMotionProxy->setStiffnesses("RHand", 0.0);
    }
    else
    {
        fMotionProxy->setStiffnesses(handID, 1.0);
        fMotionProxy->openHand(handID);
        fMotionProxy->setStiffnesses(handID, 0.0);
    }

}

Eigen::Vector3d R2Module::getRedBallPosition()
{
    Eigen::Vector3d ballPosition;
    ballPosition << fBallTrackerProxy->getPosition().at(0),
                    fBallTrackerProxy->getPosition().at(1),
                    fBallTrackerProxy->getPosition().at(2);

    return ballPosition;
}

} // namespace AL
