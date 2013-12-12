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
    - force soth convergence (stop criteria)
    - task formalization
*/

#define INFO(x) std::cerr << "\033[22;34;1m" << "[r2module] " << x << "\033[0m" << std::endl;

namespace AL
{

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
    ConfigReader theConfigReader(CONFIG_PATH + JOINT_BOUNDS_CFG);
    theConfigReader.storeJointsID(&jointID);

    // Initialize the base kinematic chain (from Nao's left foot to body center)
    theConfigReader.setKinChain(CONFIG_PATH + LEFT_LEG_BASE_CFG);
    theKinChain_LLBase = new Rmath::KinChain( theConfigReader );

    // Initialize the base kinematic chain (from Nao's right foot to body center)
    theConfigReader.setKinChain(CONFIG_PATH + RIGHT_LEG_BASE_CFG);
    theKinChain_RLBase = new Rmath::KinChain( theConfigReader );

    // Initialize the fixed transformation base-ankle
    base_ankle << 0.0,      0.0,     1.0,    BASE_ANKLE_X,
                  0.0,     -1.0,     0.0,    BASE_ANKLE_Y,
                  1.0,      0.0,     0.0,    BASE_ANKLE_Z,
                  0.0,      0.0,     0.0,             1.0;

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
  jointLimits = new TaskBase(0, Eigen::MatrixXd::Identity( JOINTS_NUM, JOINTS_NUM ), velBounds);

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
  taskMap.insert( std::pair<std::string, Task*> ("Head task", looking) );
  taskMap.insert( std::pair<std::string, Task*> ("Right arm task", Rpointing) );
  taskMap.insert( std::pair<std::string, Task*> ("Left arm task", Lpointing) );
  taskMap.insert( std::pair<std::string, Task*> ("Right leg task", Rsupporting) );
  taskMap.insert( std::pair<std::string, Task*> ("Left leg task", Lsupporting) );


#ifdef DEBUG_MODE
  INFO("Initializing tasks: ");
  INFO("- Joint limits, with priority " << jointLimits->getPriority());
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

  // Release the proxy
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

//    // Turn tasks to active
    jointLimits->activate();
    taskMap["Head task"]->activate();
    taskMap["Right arm task"]->activate();
    taskMap["Left arm task"]->activate();
    taskMap["Right leg task"]->activate();
    taskMap["Left leg task"]->activate();

    // Defining the desired pose vectors in the task space
    Eigen::VectorXd desiredHeadPose(HEAD_TASK_DIM),
                    desiredLHandPose(LARM_TASK_DIM),
                    desiredRHandPose(RARM_TASK_DIM),
                    desiredRLegPose(RLEG_TASK_DIM),
                    desiredLLegPose(LLEG_TASK_DIM);
    Eigen::VectorXd desiredHeadPosition(3),
                    desiredRHandPosition(3),
                    desiredLHandPosition(3),
                    desiredRLegPosition(3),
                    desiredLLegPosition(3);
    Eigen::VectorXd desiredHeadOrientation(3),
                    desiredRHandOrientation(3),
                    desiredLHandOrientation(3),
                    desiredRLegOrientation(3),
                    desiredLLegOrientation(3);

    // head position in 3D space
    desiredHeadPosition << HEAD_DESIRED_X, HEAD_DESIRED_Y, HEAD_DESIRED_Z;
    // head orientation in 3D space
    desiredHeadOrientation << HEAD_DESIRED_ROLL, HEAD_DESIRED_PITCH, HEAD_DESIRED_YAW;
    // Right arm position in 3D space
    desiredRHandPosition << RARM_DESIRED_X, RARM_DESIRED_Y, RARM_DESIRED_Z;
    // Right arm orientation in 3D space
    desiredRHandOrientation << RARM_DESIRED_ROLL, RARM_DESIRED_PITCH, RARM_DESIRED_YAW;
    // left arm position in 3D space
    desiredLHandPosition << LARM_DESIRED_X, LARM_DESIRED_Y, LARM_DESIRED_Z;
    // left arm orientation in 3D space
    desiredLHandOrientation << LARM_DESIRED_ROLL, LARM_DESIRED_PITCH, LARM_DESIRED_YAW;
    // Right leg position in 3D space
    desiredRLegPosition << RLEG_DESIRED_X, RLEG_DESIRED_Y, RLEG_DESIRED_Z;
    // Right leg orientation in 3D space
    desiredRLegOrientation << RLEG_DESIRED_ROLL, RLEG_DESIRED_PITCH, RLEG_DESIRED_YAW;
    // Left leg position in 3D space
    desiredLLegPosition << LLEG_DESIRED_X, LLEG_DESIRED_Y, LLEG_DESIRED_Z;
    // Left leg orientation in 3D space
    desiredLLegOrientation << LLEG_DESIRED_ROLL, LLEG_DESIRED_PITCH, LLEG_DESIRED_YAW;

    if( HEAD_TASK_DIM > 3 )
        desiredHeadPose << desiredHeadPosition, desiredHeadOrientation.head(HEAD_TASK_DIM-3);
    else
        desiredHeadPose << desiredHeadPosition.head(HEAD_TASK_DIM);

    if( LARM_TASK_DIM > 3 )
        desiredLHandPose << desiredLHandPosition, desiredLHandOrientation.head(LARM_TASK_DIM-3);
    else
        desiredLHandPose << desiredLHandPosition.head(LARM_TASK_DIM);

    if( RARM_TASK_DIM > 3 )
        desiredRHandPose << desiredRHandPosition, desiredRHandOrientation.head(RARM_TASK_DIM-3);
    else
        desiredRHandPose << desiredRHandPosition.head(RARM_TASK_DIM);

    if( RLEG_TASK_DIM > 3 )
        desiredRLegPose << desiredRLegPosition, desiredRLegOrientation.head(RLEG_TASK_DIM-3);
    else
        desiredRLegPose << desiredRLegPosition.head(RLEG_TASK_DIM);

    if( LLEG_TASK_DIM > 3 )
        desiredLLegPose << desiredLLegPosition, desiredLLegOrientation.head(LLEG_TASK_DIM-3);
    else
        desiredLLegPose << desiredLLegPosition.head(LLEG_TASK_DIM);

    Eigen::Vector3d iDesiredPose;
    iDesiredPose << 0.0, 48, 590;

    Eigen::Matrix4d h_CoMRL;
    CoM_RightLeg->forward(&h_CoMRL);
    if(taskMap["Right leg task"]->isActive())
        taskMap["Right leg task"]->setDesiredPose( desiredRLegPose.head(RLEG_TASK_DIM), RLEG_TASK_NSTEPS, h_CoMRL );

    Eigen::Matrix4d h_CoMLL;
    CoM_LeftLeg->forward(&h_CoMLL);
    if(taskMap["Left leg task"]->isActive())
        taskMap["Left leg task"]->setDesiredPose( desiredLLegPose.head(LLEG_TASK_DIM), LLEG_TASK_NSTEPS, h_CoMLL );

    if(taskMap["Head task"]->isActive())
        taskMap["Head task"]->setDesiredPose( desiredHeadPose.head(HEAD_TASK_DIM), HEAD_TASK_NSTEPS, base_ankle );

    if(taskMap["Left arm task"]->isActive())
#ifndef LARM_CIRCLE_TASK
        taskMap["Left arm task"]->setDesiredPose(iDesiredPose, desiredLHandPose.head(LARM_TASK_DIM), LARM_TASK_NSTEPS, base_ankle );
#else
        taskMap["Left arm task"]->circularPathGenerator(desiredLHandPose.head(LARM_TASK_DIM), CIRCLE_Z_DEPTH,
                                                        LARM_TASK_NSTEPS, CIRCLE_RADIUS, CIRCLE_LAPS, base_ankle );
#endif

#ifndef RARM_LARM_JOINT_TASK
    if(taskMap["Right arm task"]->isActive())
#ifndef RARM_CIRCLE_TASK
        taskMap["Right arm task"]->setDesiredPose(desiredRHandPose.head(RARM_TASK_DIM), RARM_TASK_NSTEPS, base_ankle );
#else
        taskMap["Right arm task"]->circularPathGenerator(desiredRHandPose.head(RARM_TASK_DIM), CIRCLE_Z_DEPTH,
                                                         RARM_TASK_NSTEPS, CIRCLE_RADIUS, CIRCLE_LAPS, base_ankle );
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

    updateConstraints(getConfiguration());

    // Main loop
    while(true)
    {
#ifdef UP_DOWN_TASK
        if ( (taskMap["Right leg task"]->done()) && (taskMap["Left leg task"]->done()) )
        {
            UP_DOWN *= -1;

            Eigen::Matrix4d h_CoMRL, h_CoMLL;
            CoM_RightLeg->forward(&h_CoMRL);
            CoM_LeftLeg->forward(&h_CoMLL);

            Eigen::VectorXd desiredRLegPose = taskMap["Right leg task"]->getTargetPose();
            Eigen::VectorXd desiredLLegPose = taskMap["Left leg task"]->getTargetPose();
            desiredRLegPose(2) += UP_DOWN;
            desiredLLegPose(2) += UP_DOWN;
            taskMap["Right leg task"]->setDesiredPose(desiredRLegPose, RLEG_TASK_NSTEPS, h_CoMRL );
            taskMap["Left leg task"]->setDesiredPose(desiredLLegPose, LLEG_TASK_NSTEPS, h_CoMLL );
        }
#endif

        // Retrieve current robot configuration
        Eigen::VectorXd q(jointID.size());
        q = getConfiguration();

#ifdef DEBUG_MODE
        INFO("\nCurrent joint configuration:");
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

        int task_i = 2;
#endif
        // Set up the hierarchy
        std::vector<Eigen::MatrixXd> A;
        std::vector<soth::VectorBound> b;
        // Joint limits first (top priority task)
        A.push_back(jointLimits->constraintMatrix());
        b.push_back(jointLimits->vectorBounds());
        // All the remaining tasks (in descending order of priority)
        std::set<Task*>::iterator updater = taskSet.begin();
        while( updater != taskSet.end() )
        {
            // Retrieve current kinchain joints information
            std::map<std::string, int> jointsIndicesMap;
            (*updater)->getJointsIDs(&jointsIndicesMap);
            // Current task constraint matrix (restricted to the joint involved)
            Eigen::MatrixXd currentA_task = (*updater)->constraintMatrix();

            // Rewrite the task constraint matrix with the complete joint configuration
            Eigen::MatrixXd currentA = Eigen::MatrixXd::Zero( currentA_task.rows(), JOINTS_NUM );
            // The task constraint matrix is the only non-zero block in A
            for (unsigned int i = 0; i < jointID.size(); ++i)
            {
                if (jointsIndicesMap.find(jointID.at(i)) != jointsIndicesMap.end())
                    currentA.col(i) = currentA_task.col( jointsIndicesMap[jointID.at(i)] );
                else
                    currentA.col(i) = Eigen::VectorXd::Zero(currentA_task.rows());
            }

            A.push_back(currentA);
            b.push_back((*updater)->vectorBounds());
            ++updater;

#ifdef DEBUG_MODE
            INFO("Task n. " << task_i << ", constraint matrix (enlarged):\n" << currentA);
            ++task_i;
#endif
        }
        hsolver.pushBackStages(A, b);

        // Configure the solver
        hsolver.setDamping(0.0);
        hsolver.setInitialActiveSet();

#ifdef DEBUG_MODE
        INFO("Configuring HQP solver...");
        INFO("Start active search... ");
#endif
        // Run the solver
        hsolver.activeSearch(qdot);
        // Trim ssolution
        Rmath::trim(&qdot);

#ifdef DEBUG_MODE
        INFO("Done.");
        INFO("Active set: ");
        hsolver.showActiveSet(std::cerr);
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
 * Update every active task with the current joint configuration.
 */
void R2Module::updateConstraints(const Eigen::VectorXd& q)
{
    /* ------------------------- Joint bounds update -------------------------*/

    Eigen::MatrixXd velBounds(q.size(), 2);
    posBound2velBound(jointBounds, q, &velBounds);
    jointLimits->setVectorBounds(velBounds);

#ifdef DEBUG_MODE
    INFO("Updating joint bounds: ");
    INFO(std::endl << *jointLimits);
#endif

    /* -------------------- Head and arms tasks update --------------------*/


    // Head task update
    Eigen::VectorXd q_H (LLEG_CHAIN_SIZE+HEAD_CHAIN_SIZE);
    q_H << q.segment<LLEG_CHAIN_SIZE>(LLEG_CHAIN_BEGIN),
            q.segment<HEAD_CHAIN_SIZE>(HEAD_CHAIN_BEGIN);
    if(taskMap["Head task"]->isActive())
        ( taskMap["Head task"] )->update( q_H, Eigen::VectorXd::Zero(HEAD_TASK_DIM), K_HEAD, base_ankle );

#ifdef DEBUG_MODE
    INFO("Head task constraint equation: ");
    INFO(std::endl << *( taskMap["Head task"] ) );
#endif

    // Left arm task update
    Eigen::VectorXd q_LA (LLEG_CHAIN_SIZE+LARM_CHAIN_SIZE);
    q_LA << q.segment<LLEG_CHAIN_SIZE>(LLEG_CHAIN_BEGIN),
            q.segment<LARM_CHAIN_SIZE>(LARM_CHAIN_BEGIN);
    if( taskMap["Left arm task"]->isActive() )
        ( taskMap["Left arm task"] )->update( q_LA, Eigen::VectorXd::Zero(LARM_TASK_DIM), K_LARM, base_ankle );

#ifdef DEBUG_MODE
    INFO("Left arm task constraint equation: ");
    INFO(std::endl << *( taskMap["Left arm task"] ) );
#endif

    // Right arm task update
    Eigen::VectorXd q_RA (LLEG_CHAIN_SIZE+RARM_CHAIN_SIZE);
    q_RA << q.segment<LLEG_CHAIN_SIZE>(LLEG_CHAIN_BEGIN),
            q.segment<RARM_CHAIN_SIZE>(RARM_CHAIN_BEGIN);
#ifdef RARM_LARM_JOINT_TASK
    if ( ARMS_TASK == MIMIC_TASK )
    {
        Eigen::VectorXd desiredRHandPose = ( taskMap["Left arm task"] )->getTargetPose();
        assert(RARM_TASK_DIM == LARM_TASK_DIM);
        assert(LARM_TASK_DIM >= 2);
        desiredRHandPose(1) = desiredRHandPose(1)-MINIMUM_HANDS_DISTANCE;

        if (LARM_TASK_DIM > 3)
            for (int i=3; i < LARM_TASK_DIM; ++i)
                desiredRHandPose(i) = -desiredRHandPose(i);

        ( taskMap["Right arm task"] )->setDesiredPose(desiredRHandPose, 1, base_ankle );
        ( taskMap["Right arm task"] )->update( q_RA, Eigen::VectorXd::Zero(RARM_TASK_DIM), K_RARM, base_ankle );
    }
    else if ( ARMS_TASK == MIRROR_TASK )
    {
        Eigen::VectorXd desiredRHandVel = ( taskMap["Left arm task"] )->getTargetVelocity();

        assert(RARM_TASK_DIM == LARM_TASK_DIM);
        assert(LARM_TASK_DIM >= 2);
        desiredRHandVel(1) = -desiredRHandVel(1);

        if (LARM_TASK_DIM > 3)
            for (int i=3; i < LARM_TASK_DIM; ++i)
                desiredRHandVel(i) = -desiredRHandVel(i);

        ( taskMap["Right arm task"] )->update( q_RA, desiredRHandVel, K_RARM, base_ankle );
    }
#else
    if( taskMap["Right arm task"]->isActive() )
        ( taskMap["Right arm task"] )->update( q_RA, Eigen::VectorXd::Zero(RARM_TASK_DIM), K_RARM, base_ankle );
#endif

#ifdef DEBUG_MODE
    INFO("Right arm task constraint equation: ");
    INFO(std::endl << *( taskMap["Right arm task"] ) );
#endif

    // Right leg task update
    Eigen::Matrix4d h_CoMRL;
    CoM_RightLeg->forward(&h_CoMRL);
    if( taskMap["Right leg task"]->isActive() )
        ( taskMap["Right leg task"] )->update( q.segment<RLEG_CHAIN_SIZE>(RLEG_CHAIN_BEGIN),
                                               Eigen::VectorXd::Zero(RLEG_TASK_DIM),
                                               K_RLEG, h_CoMRL);

#ifdef DEBUG_MODE
    INFO("Right leg task constraint equation: ");
    INFO(std::endl << *( taskMap["Right leg task"] ) );
#endif


    // Left leg task update
    Eigen::Matrix4d h_CoMLL;
    CoM_LeftLeg->forward(&h_CoMLL);
    if( taskMap["Left leg task"]->isActive() )
        ( taskMap["Left leg task"] )->update( q.segment<LLEG_CHAIN_SIZE>(LLEG_CHAIN_BEGIN),
                                              Eigen::VectorXd::Zero(LLEG_TASK_DIM),
                                              K_LLEG, h_CoMLL );

#ifdef DEBUG_MODE
    INFO("Left leg task constraint equation: ");
    INFO(std::endl << *( taskMap["Left leg task"] ) );
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
