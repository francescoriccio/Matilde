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
//#define DEBUG_MODE
//#define TEST_KINCHAIN


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
  // Initialize the base kinematic chain (from Nao's left foot to body center)
  ConfigReader theConfigReader( CONFIG_PATH + LEFT_LEG_CFG, CONFIG_PATH + JOINT_BOUNDS_CFG );
  theConfigReader.storeJointsID(&jointID);
  theKinChainLeftLeg = new Rmath::KinChain( theConfigReader, LLEG_CHAIN_BEGIN, LLEG_CHAIN_END );

#ifdef TEST_KINCHAIN
  // Initialize the test kinematic chain (left arm)
  ConfigReader theConfigReader_TMP( CONFIG_PATH + LEFT_ARM_CFG, CONFIG_PATH + JOINT_BOUNDS_CFG );
  theKinChain_TMP = new Rmath::KinChain( theConfigReader_TMP, LARM_CHAIN_BEGIN, LARM_CHAIN_SIZE);
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

  /* ------------------------------ Other tasks: head (priority = 2), left arm (priority = 1) ------------------------------ */

  // Extract joint bounds from the configuration file
  ConfigReader headConfig( CONFIG_PATH + HEAD_CFG, CONFIG_PATH + JOINT_BOUNDS_CFG );
  // Task initialization
  Task* head = new Task(HEAD_TASK_DIM, HEAD_CHAIN_SIZE, HEAD_TASK_PRIORITY, headConfig,
                        HEAD_CHAIN_BEGIN, HEAD_CHAIN_END);

  // Extract joint bounds from the configuration file
  ConfigReader LpointingConfig( CONFIG_PATH + LEFT_ARM_CFG, CONFIG_PATH + JOINT_BOUNDS_CFG );
  // Task initialization
  Task* Lpointing = new Task(LARM_TASK_DIM, LARM_CHAIN_SIZE, LARM_TASK_PRIORITY, LpointingConfig,
                             LARM_CHAIN_BEGIN, LARM_CHAIN_END);

  // Extract joint bounds from the configuration file
  ConfigReader RpointingConfig( CONFIG_PATH + RIGHT_ARM_CFG, CONFIG_PATH + JOINT_BOUNDS_CFG );
  // Task initialization
  Task* Rpointing = new Task(RARM_TASK_DIM, RARM_CHAIN_SIZE, RARM_TASK_PRIORITY, RpointingConfig,
                             RARM_CHAIN_BEGIN, RARM_CHAIN_END);

  // Extract joint bounds from the configuration file
  ConfigReader RlegConfig( CONFIG_PATH + RIGHT_LEG_CFG, CONFIG_PATH + JOINT_BOUNDS_CFG );
  // Task initialization
  Task* Rsupporting = new Task(RLEG_TASK_DIM, RLEG_CHAIN_SIZE, RLEG_TASK_PRIORITY, RlegConfig,
                             RLEG_CHAIN_BEGIN, RLEG_CHAIN_END);

  // Extract joint bounds from the configuration file
  ConfigReader LlegConfig( CONFIG_PATH + LEFT_LEG_CFG, CONFIG_PATH + JOINT_BOUNDS_CFG );
  // Task initialization
  Task* Lsupporting = new Task(LLEG_TASK_DIM, LLEG_CHAIN_SIZE, LLEG_TASK_PRIORITY, LlegConfig,
                             LLEG_CHAIN_BEGIN, LLEG_CHAIN_END);

  // Push every task into the task map
  taskMap.insert( std::pair<std::string, Task*> ("Head task", head) );
  taskMap.insert( std::pair<std::string, Task*> ("Right arm task", Rpointing) );
  taskMap.insert( std::pair<std::string, Task*> ("Left arm task", Lpointing) );
  taskMap.insert( std::pair<std::string, Task*> ("Right leg task", Rsupporting) );
  taskMap.insert( std::pair<std::string, Task*> ("Left leg task", Lsupporting) );

  // Storing tasks location with respect to the whole Nao kinematic chain
  taskLoc.insert( std::pair<Task*,int> (head, HEAD_CHAIN_BEGIN) );
  taskLoc.insert( std::pair<Task*,int> (Rpointing, RARM_CHAIN_BEGIN) );
  taskLoc.insert( std::pair<Task*,int> (Lpointing, LARM_CHAIN_BEGIN) );
  taskLoc.insert( std::pair<Task*,int> (Rsupporting, RLEG_CHAIN_BEGIN) );
  taskLoc.insert( std::pair<Task*,int> (Lsupporting, LLEG_CHAIN_BEGIN) );

#ifdef DEBUG_MODE
  INFO("Initializing tasks: ");
  INFO("- Joint limits, with priority " << jointLimits->getPriority());
  INFO("- Head task, with priority " << head->getPriority());
  INFO("- Right arm task, with priority " << Rpointing->getPriority());
  INFO("- Left arm task, with priority " << Lpointing->getPriority());
  INFO("- Right leg task, with priority " << Rsupporting->getPriority());
  INFO("- Left leg task, with priority " << Lsupporting>getPriority());
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

    // Shift CoM-right arm
    Eigen::Vector3d CoM_RA;
    CoM_RA << COM_RARM_SHIFT_X, COM_RARM_SHIFT_Y, COM_RARM_SHIFT_Z;
    // Shift CoM-left arm
    Eigen::Vector3d CoM_LA;
    CoM_LA << COM_LARM_SHIFT_X, COM_LARM_SHIFT_Y, COM_LARM_SHIFT_Z;

    // Shift CoM-right leg
    Eigen::Vector3d CoM_RL;
    CoM_RL << COM_RLEG_SHIFT_X, COM_RLEG_SHIFT_Y, COM_RLEG_SHIFT_Z;
    // Shift CoM-left leg
    Eigen::Vector3d Base_Lankle;
    Base_Lankle << BASE_LANKLE_SHIFT_X, BASE_LANKLE_SHIFT_Y, BASE_LANKLE_SHIFT_Z;

    // Defining the desired pose vectors in the task space
    Eigen::VectorXd desiredHeadPose(HEAD_TASK_DIM), desiredLHandPose(LARM_TASK_DIM), desiredRHandPose(RARM_TASK_DIM),
            desiredRLegPose(RLEG_TASK_DIM), desiredLLegPose(LLEG_TASK_DIM);
    Eigen::VectorXd desiredHeadPosition(3), desiredRHandPosition(3), desiredLHandPosition(3),
            desiredRLegPosition(3), desiredLLegPosition(3);
    Eigen::VectorXd desiredHeadOrientation(3), desiredRHandOrientation(3), desiredLHandOrientation(3),
            desiredRLegOrientation(3), desiredLLegOrientation(3);

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

    taskMap["Right leg task"]->setDesiredPose(desiredRLegPose.head(RLEG_TASK_DIM), RLEG_TASK_NSTEPS,
                                             Rmath::homX(-M_PI/4, CoM_RL) );

//    taskMap["Head task"]->setDesiredConfiguration(desiredHeadPose.head(HEAD_TASK_DIM), HEAD_TASK_NSTEPS);
//    taskMap["Head task"]->setDesiredPose(desiredHeadPose.head(HEAD_TASK_DIM), HEAD_TASK_NSTEPS,
//                                         Rmath::hTranslation(Eigen::Vector3d::Zero()));

//    taskMap["Left arm task"]->setDesiredPose(desiredLHandPose.head(LARM_TASK_DIM), LARM_TASK_NSTEPS,
//                                             Rmath::homX(-M_PI/2, CoM_LA) );
//    taskMap["Left arm task"]->circularPathGenerator(desiredLHandPose.head(LARM_TASK_DIM), -70, LARM_TASK_NSTEPS, 100, 3,
//                                                    Rmath::homX(-M_PI/2, CoM_LA));

//    taskMap["Right arm task"]->setDesiredPose(desiredRHandPose.head(RARM_TASK_DIM), RARM_TASK_NSTEPS,
//                                              Rmath::homX(-M_PI/2, CoM_RA) );
//    taskMap["Right arm task"]->circularPathGenerator(desiredRHandPose.head(RARM_TASK_DIM), -70, RARM_TASK_NSTEPS, 100, 3,
//                                                     Rmath::homX(-M_PI/2, CoM_RA));

#ifdef DEBUG_MODE
    INFO("Desired head position in space: [\n" << desiredHeadPose << "]");
    INFO("Desired left hand position in space: [\n" << desiredLHandPose << "]");
    INFO("Desired right hand position in space: [\n" << desiredRHandPose << "]");
#endif

    // Turn tasks to active
    jointLimits->activate();
//    taskMap["Head task"]->activate();
//    taskMap["Left arm task"]->activate();
//    taskMap["Right arm task"]->activate();
    taskMap["Right leg task"]->activate();


    closeHand("RHand");
    openHand("LHand");
    bool init = true;
    // Main loop
    while(true)
    {
//        if(init)
//        {
//            INFO("starting pointing task... ");
//            taskMap["Left arm task"]->setDesiredPose(desiredLHandPose.head(LARM_TASK_DIM),
//                                                     LARM_TASK_NSTEPS,
//                                                     Rmath::homX(-M_PI/2, CoM_LA) );
////            taskMap["Right arm task"]->circularPathGenerator(desiredRHandPose.head(RARM_TASK_DIM), -70,
////                                                             RARM_TASK_NSTEPS, 100, 3,
////                                                             Rmath::homX(-M_PI/2, CoM_RA));
//            init = false;
//        }
//        if( taskMap["Left arm task"]->isConverged() )
//            init = true;

//        if(init)
//        {
//            INFO("starting circular path... ");
//            taskMap["Left arm task"]->circularPathGenerator(desiredLHandPose.head(LARM_TASK_DIM), -50,
//                                                            LARM_TASK_NSTEPS, 70, 3,
//                                                            Rmath::homX(-M_PI/2, CoM_LA));
//            init=false;
//        }


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
            // Current task constraint matrix (restricted to the joint involved)
            Eigen::MatrixXd currentA_task = (*updater)->constraintMatrix();

            // Rewrite the task constraint matrix with the complete joint configuration
            Eigen::MatrixXd currentA = Eigen::MatrixXd::Zero( currentA_task.rows(), JOINTS_NUM );
            // The task constraint matrix is the only non-zero block in A
            currentA.block( 0, taskLoc[*updater], currentA_task.rows(), currentA_task.cols() ) = currentA_task;

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

#ifdef TEST_KINCHAIN
        qdot = Eigen::VectorXd::Zero(q.size());
        Eigen::MatrixXd J_TMP_pinv;
        Rmath::pseudoInverse(J_TMP, &J_TMP_pinv);

        qdot.segment<LARM_CHAIN_SIZE>(LARM_CHAIN_BEGIN) = J_TMP_pinv * r;
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
    // Updating the base kinematic chain (left foot to body center)
    Eigen::Matrix4d H_base;
    Eigen::MatrixXd J_base(6, LLEG_CHAIN_SIZE);
    // Left leg joints are those on the bottom part of the IDs vector
    theKinChainLeftLeg->update(q.tail(LLEG_CHAIN_SIZE));
    // Compute kinematics
    theKinChainLeftLeg->forward(&H_base);
    theKinChainLeftLeg->differential(&J_base);

    /*fast DEBUG*/
    INFO( std::endl << *taskMap["Right leg task"]->kinChain() );
    Eigen::Matrix4d H_RL;
    (taskMap["Right leg task"]->kinChain())->forward(&H_RL);
    INFO("H right Leg \n"<< H_RL);

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
    q_TMP << q.segment<LARM_CHAIN_SIZE>(LARM_CHAIN_BEGIN);
    INFO("Joint configuration:");
    for(unsigned int i=0; i<q_TMP.size(); ++i)
        INFO(jointID.at(LARM_CHAIN_BEGIN + i) << "\t" << q_TMP(i));

    theKinChain_TMP->update(q_TMP);
    theKinChain_TMP->forward(&H_TMP);
    theKinChain_TMP->differential(&J_TMP, 3);

    INFO("Direct kinematics transform: ");
    INFO(std::endl << H_TMP);
    INFO("Jacobian (linear velocities only): ");
    INFO(std::endl << J_TMP);
#endif

    /* -------------------- Head and arms tasks update --------------------*/

    // Shift CoM-right arm
    Eigen::Vector3d CoM_RA;
    CoM_RA << COM_RARM_SHIFT_X, COM_RARM_SHIFT_Y, COM_RARM_SHIFT_Z;
    // Shift CoM-left arm
    Eigen::Vector3d CoM_LA;
    CoM_LA << COM_LARM_SHIFT_X, COM_LARM_SHIFT_Y, COM_LARM_SHIFT_Z;

    // Shift CoM-right leg
    Eigen::Vector3d CoM_RL;
    CoM_RL << COM_RLEG_SHIFT_X, COM_RLEG_SHIFT_Y, COM_RLEG_SHIFT_Z;
    // Shift CoM-left leg
    Eigen::Vector3d Base_Lankle;
    Base_Lankle << BASE_LANKLE_SHIFT_X, BASE_LANKLE_SHIFT_Y, BASE_LANKLE_SHIFT_Z;

    // Head task update
    ( taskMap["Head task"] )->update( q.head(HEAD_CHAIN_SIZE), Eigen::VectorXd::Zero(HEAD_TASK_DIM), K_HEAD,
                                      Rmath::hTranslation(Eigen::Vector3d::Zero()));

#ifdef DEBUG_MODE
    INFO("Head task constraint equation: ");
    INFO(std::endl << *( taskMap["Head task"] ) );
#endif

    // Left arm task update
    ( taskMap["Left arm task"] )->update( q.segment<LARM_CHAIN_SIZE>(LARM_CHAIN_BEGIN), Eigen::VectorXd::Zero(LARM_TASK_DIM),
                                          K_LARM, Rmath::homX(-M_PI/2, CoM_LA) );
//    ( taskMap["Left arm task"] )->update( q.segment<LARM_CHAIN_SIZE>(LARM_CHAIN_BEGIN), Eigen::VectorXd::Zero(LARM_CHAIN_SIZE), K_LARM);

#ifdef DEBUG_MODE
    INFO("Left arm task constraint equation: ");
    INFO(std::endl << *( taskMap["Left arm task"] ) );
#endif

    // Right arm task update
    ( taskMap["Right arm task"] )->update( q.segment<RARM_CHAIN_SIZE>(RARM_CHAIN_BEGIN), Eigen::VectorXd::Zero(RARM_TASK_DIM),
                                           K_RARM, Rmath::homX(-M_PI/2, CoM_RA) );
//    ( taskMap["Right arm task"] )->update( q.segment<RARM_CHAIN_SIZE>(RARM_CHAIN_BEGIN), Eigen::VectorXd::Zero(RARM_CHAIN_SIZE), K_RARM);

#ifdef DEBUG_MODE
    INFO("Right arm task constraint equation: ");
    INFO(std::endl << *( taskMap["Right arm task"] ) );
#endif

    // Right leg task update
    ( taskMap["Right leg task"] )->update( q.segment<RLEG_CHAIN_SIZE>(RLEG_CHAIN_BEGIN), Eigen::VectorXd::Zero(RLEG_TASK_DIM),
                                           K_RLEG, Rmath::homX(-M_PI/4, CoM_RL) );
//    ( taskMap["Right leg task"] )->update( q.segment<RLEG_CHAIN_SIZE>(RLEG_CHAIN_BEGIN), Eigen::VectorXd::Zero(RLEG_CHAIN_SIZE), K_RLEG);

#ifdef DEBUG_MODE
    INFO("Right arm task constraint equation: ");
    INFO(std::endl << *( taskMap["Right leg task"] ) );
#endif

#ifdef TEST_KINCHAIN
    INFO("Desired left hand position in space: [\n" << desiredLHandPose << "]");
//    r = K_HEAD * ( desiredLHandPose - H_TMP.topRightCorner(3,1) ) + Eigen::Vector3d::UnitZ()*100;
    r = ( Eigen::Vector3d::UnitX()*100 +
          Eigen::Vector3d::UnitY()*100 +
          Eigen::Vector3d::UnitZ()*100 );
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
//            angles.arrayPush( delta_q(i) );

            if(jointID.at(i) == "RHipYawPitch") angles.arrayPush( /*delta_q(i)*/-M_PI/2 );
            else if(jointID.at(i) == "RHipRoll") angles.arrayPush( /*delta_q(i)*/-M_PI/4 );
            else if(jointID.at(i) == "RHipPitch") angles.arrayPush( /*delta_q(i)*/0.0 );
            else if(jointID.at(i) == "RKneePitch") angles.arrayPush( /*delta_q(i)*/0.0 );
            else if(jointID.at(i) == "RAnklePitch") angles.arrayPush( /*delta_q(i)*/0.0 );
            else if(jointID.at(i) == "RAnkleRoll") angles.arrayPush( /*delta_q(i)*/0.0 );
            else
                angles.arrayPush( 0.0 );

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
