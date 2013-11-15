#include "r2Module.h"

#include <iostream>
#include <boost/assign/std/vector.hpp>

#include <alvision/alvisiondefinitions.h>
#include <alvision/alimage.h>
#include <alcommon/albroker.h>
#include <qi/log.hpp>

#include "configReader.h"

//using namespace boost::assign;

/*
  TODO:
    - check transformations
    - task formalization
*/
#define JOINTS_NUM 24

#define HEAD 2
#define R_ARM 5
#define L_ARM 5
#define R_LEG 6
#define L_LEG 6

#define K_HEAD 1

#define TIME_STEP 1

namespace AL
{

R2Module::R2Module( boost::shared_ptr<ALBroker> pBroker, const std::string& pName ):
  ALModule(pBroker , pName), fRegisteredToVideoDevice(false)
{
  // Describe the module here.
  setModuleDescription("Robotics2 Project");

  //Define bound methods with their description.
  functionName( "registerToVideoDevice", getName(), "Register to the V.I.M." );
  BIND_METHOD( R2Module::registerToVideoDevice );

  functionName( "unRegisterFromVideoDevice", getName(), "Unregister from the V.I.M." );
  BIND_METHOD( R2Module::unRegisterFromVideoDevice );

  functionName( "getCurrentFrame", getName(), "Save an image received from the camera." );
  BIND_METHOD( R2Module::getCurrentFrame );
}

void R2Module::exit()
{
  AL::ALModule::exit();
}

void R2Module::init()
{

  // Create a proxy to ALVideoDevice, ALMotion, ALRobotPosture.
  try
    {
        fCamProxy = boost::shared_ptr<ALVideoDeviceProxy>(new ALVideoDeviceProxy(getParentBroker()));
        fMotionProxy = boost::shared_ptr<ALMotionProxy>(new ALMotionProxy(getParentBroker()));
        fPostureProxy = boost::shared_ptr<ALRobotPostureProxy>(new ALRobotPostureProxy(getParentBroker()));
    }
    catch (const AL::ALError& e)
    {
        qiLogError("vision.R2Module") << "Error while getting proxy on ALVideoDevice.  Error msg " << e.toString() << std::endl;

        R2Module::exit();

        return;
    }

  if(!fCamProxy || !fMotionProxy || !fPostureProxy)
  {
    qiLogError("vision.AmrModule") << "Error while getting proxy on ALVideoDevice. Check ALVideoDevice is running." << std::endl;
    R2Module::exit();
    return;
  }

  // config reader initialization
  std::string path = "../config/";
  ConfigReader theConfigReader( path + "dh_leftLeg.cfg", path + "joints_params.cfg" );
  theConfigReader.storeJointsID(&jointID);

  ConfigReader theConfigReader_TMP( path + "dh_leftArm.cfg", path + "joints_params.cfg" );

//  // base-kinematic chain initialization
  theKinChainLeftLeg = new Rmath::KinChain( theConfigReader, HEAD+R_ARM+L_ARM+R_LEG, HEAD+R_ARM+L_ARM+R_LEG+L_LEG);
  theKinChain_TMP = new Rmath::KinChain( theConfigReader_TMP, HEAD+R_ARM, HEAD+R_ARM+L_ARM);

//  rightLeg = new Task(path + "dh_rightLeg.cfg", HEAD+R_ARM+L_ARM, HEAD+R_ARM+L_ARM+R_LEG);
//  leftArm = new Task(path + "dh_leftArm.cfg",HEAD+R_ARM, HEAD+R_ARM+L_ARM);
//  rightArm = new Task(path + "dh_rightArm.cfg", HEAD, HEAD+R_ARM);

  Eigen::MatrixXd jbounds(JOINTS_NUM,2), velBounds(JOINTS_NUM,2);
  theConfigReader.extractJointBounds(&jbounds);
  jointBounds = jbounds;

  posBound2velBound(jointBounds, getConfiguration(), &velBounds);

//  // tasks initialization
  jointLimits = new TaskBase(0, Eigen::MatrixXd::Identity(jointID.size(),jointID.size()), velBounds);

  head = new Task(path + "dh_head.cfg", 3, jointID.size(), 0, HEAD, 2);
//  pointing = new Task(path + "dh_rightArm.cfg", TASK_SPACE_DIM, jointID.size(), HEAD, HEAD+R_ARM, 1);
  pointing = new Task(path + "dh_leftArm.cfg", 3, jointID.size(), HEAD+R_ARM, HEAD+R_ARM+L_ARM, 1);

  // initialize resolution and color space
  pResolution = AL::kQVGA;
  pColorSpace = AL::kYuvColorSpace;

  int imgWidth = 0;
  int imgHeight = 0;

  setSizeFromResolution(pResolution, imgWidth, imgHeight);

  // create the motion thread
  pthread_create( &motionThreadId, NULL, (void*(*)(void*)) motionTh, this);

  registerToVideoDevice();
  if(fRegisteredToVideoDevice)
  {
      // initialize image
      fcurrImageHeader = cv::Mat::zeros(imgHeight, imgWidth, CV_8UC1);

      // create the camera thread
      pthread_create( &cameraThreadId, NULL, (void*(*)(void*)) visionTh, this);
  }

}

R2Module::~R2Module()
{
    // Unregister the video module.
    try
    {
        if(fCamProxy)
            fCamProxy->unsubscribe(fVideoClientName);

        fCamProxy.reset();
        fMotionProxy.reset();
        fPostureProxy.reset();
    }
    catch(const AL::ALError& e)
    {
        qiLogError("vision.R2Module") <<  e.toString() << std::endl;
    }

    // destroy thread
    pthread_cancel(motionThreadId);
    pthread_cancel(cameraThreadId);
}

void R2Module::registerToVideoDevice()
{
  // If we've already registered a module, we need to unregister it first !
  if (fRegisteredToVideoDevice)
  {
    throw ALError(getName(), "registerToVideoDevice()", "A video module has already been "
      "registered. Call unRegisterFromVideoDevice() before trying to register a new module.");
  }

  const std::string kOriginalName = "r2Module";
  const int kFps = 30;

//   Release Image Headers if they have been allocated.
  if (!fcurrImageHeader.empty())
    fcurrImageHeader.release();

  if(fCamProxy)
      fVideoClientName = fCamProxy->subscribeCamera(kOriginalName, 1, pResolution, pColorSpace, kFps);


  qiLogInfo("vision.r2Module") << "Module registered as " << fVideoClientName << std::endl;

  // Registration is successful, set fRegisteredToVim to true.
  fRegisteredToVideoDevice = true;
}

void R2Module::unRegisterFromVideoDevice()
{
  if (!fRegisteredToVideoDevice)
  {
    throw ALError(getName(), "unRegisterFromVideoDevice()", "No video module is currently "
      "registered! Call registerToVideoDevice first.");
  }

//   Release Image Headers if they have been allocated.
  if (!fcurrImageHeader.empty())
    fcurrImageHeader.release();

  qiLogInfo("vision.R2Module") << "try to unregister " << fVideoClientName << " module." << std::endl;
  if(fCamProxy)
    fCamProxy->unsubscribe(fVideoClientName);

  qiLogInfo("vision.R2Module") << "Done." << std::endl;

  // UnRegistration is successful, set fRegisteredToVim to false.
  fRegisteredToVideoDevice = false;
}

void R2Module::vision()
{
    qi::os::sleep(1.5);

    while(true)
    {
        qi::os::msleep(100);
//        getCurrentFrame();

    }
}

void R2Module::motion()
{
//    qi::os::sleep(0.5);

    // initialize posture proxy
    fPostureProxy->goToPosture("StandInit", 0.5f);

    while(true)
    {
        std::vector<Eigen::MatrixXd> A;
        std::vector<soth::VectorBound> b;

        Eigen::VectorXd q(jointID.size());
        q = getConfiguration();

        updateConstraints(q);

        // update set
        taskSet.clear();
        taskSet.insert(jointLimits);
        taskSet.insert(head);
        taskSet.insert(pointing);

        soth::HCOD hsolver( q.size(), taskSet.size() );
        Eigen::VectorXd qdot(q.size());

        std::set<TaskBase*>::iterator updater = taskSet.begin();
        while( updater != taskSet.end() )
        {
//            std::cout<<"task: \n"<< *(*updater) <<std::endl;
            A.push_back((*updater)->constraintMatrix());
            b.push_back((*updater)->vectorBounds());

            ++updater;
        }

        hsolver.pushBackStages(A, b);

        hsolver.setDamping(0.0);
        hsolver.setInitialActiveSet();

        hsolver.activeSearch(qdot);
        Rmath::trim(&qdot);

        updateConfiguration(q + qdot*TIME_STEP);

//        std::cout << "active set = ";
//        hsolver.showActiveSet(std::cout);
    }
}

void R2Module::getCurrentFrame()
{

  // Check that a video module has been registered.
  if (!fRegisteredToVideoDevice)
  {
    throw ALError(getName(), "saveImageRemote()",  "No video module is currently "
      "registered! Call registerToVideoDevice() first.");
  }

  ALValue frame = fCamProxy->getImageRemote(fVideoClientName);
  
  if (frame.getType()!= ALValue::TypeArray && frame.getSize() != 7)
  {
    throw ALError(getName(), "getImages", "Invalid image returned.");
  }
  
  const long long timeStamp = ((long long)(int)frame[4])*1000000LL + ((long long)(int)frame[5]);
  time = (int)(timeStamp/1000000LL);
  
  fcurrImageHeader.data = (uchar*) frame[6].GetBinary();

  if ((char) cv::waitKey(33) != 27)
      cv::imshow("Frame", fcurrImageHeader);

  fCamProxy->releaseImage(fVideoClientName);
}

void R2Module::updateConstraints(const Eigen::VectorXd& q)
{
    // taskBase update
    Eigen::MatrixXd velBounds(JOINTS_NUM, 2);
    posBound2velBound(jointBounds, q, &velBounds);
    jointLimits->setVectorBounds(velBounds);

//    // base
//    Eigen::Matrix4d H_base;
//    Eigen::MatrixXd J_base(6, L_LEG);
//    Eigen::VectorXd q_base(L_LEG);
//    q_base << q.tail(L_LEG);

//    theKinChainLeftLeg->update(q_base);
//    theKinChainLeftLeg->forward(&H_base);
//    theKinChainLeftLeg->differential(&J_base);

    Eigen::Matrix4d H_TMP;
    Eigen::MatrixXd J_TMP;
    Eigen::VectorXd q_TMP(L_ARM), qjesus(12);
    qjesus << q.head(12);
    q_TMP << qjesus.tail(L_ARM);

    theKinChain_TMP->update(q_TMP);
    theKinChain_TMP->differential(&J_TMP,3);
    theKinChain_TMP->forward(&H_TMP);

    std::cout<<"q_TMP: \n "<< q_TMP <<std::endl;
//    std::cout<<"kinchain_TMP: \n "<< (*theKinChain_TMP) <<std::endl;
//    std::cout<<"H_TMP: \n "<< H_TMP <<std::endl;
    std::cout<<"J_TMP: \n "<< J_TMP <<std::endl;

    // desired poses
    Eigen::VectorXd desiredHeadPose(3), desiredRHandPose(3);
//    desiredHeadPose << 53.9, 0.0, 194.4, 0.0, 0.0, 0.0; // zero HEAD
    desiredHeadPose << -20.0, 0.0, 194.4;

//    desiredRHandPose << 218.7, 133, 112.31; //point L ARM
//    desiredRHandPose << 0.0, 316.0, 112.31; //point L ARM
    desiredRHandPose << -18, 110.0, 300.31; //point L ARM

//    desiredRHandPose << 218.7, -133, 112.31, 0.0, 0.0, 0.0; //point R ARM
//    desiredRHandPose <<-15, -329.01, 112.31, 0.0, 0.0, 0.0; //point R ARM

    // task head update
    head->update( q, K_HEAD, desiredHeadPose, Eigen::VectorXd::Zero(3) );// , J_base, H_base);

    // task pointing update
    pointing->update( q, K_HEAD, desiredRHandPose, Eigen::VectorXd::Zero(3) );
}

bool R2Module::updateConfiguration(const Eigen::VectorXd& delta_q)
{
    AL::ALValue jointId, angles, setStiff, zeroArray;

    for(int i=0; i< delta_q.size()/2; ++i)
    {
        if(delta_q(i) != 0)
        {
            jointId.arrayPush( jointID.at(i) );
            angles.arrayPush( delta_q(i) );
            setStiff.arrayPush( 1.0 );
            zeroArray.arrayPush( 0.0 );
        }
    }

    fMotionProxy->stiffnessInterpolation( jointId, setStiff, 0.3f );
    fMotionProxy->setStiffnesses(jointId, setStiff);

    fMotionProxy->setAngles(jointId, angles, 0.3f);

    //    qi::os::sleep(1.0f);
    fMotionProxy->setStiffnesses(jointId, zeroArray);
}

void R2Module::posBound2velBound( const Eigen::MatrixXd& posBound, const Eigen::VectorXd& configuration, Eigen::MatrixXd* velBound)
{
    Eigen::MatrixXd currConf(configuration.size(), 2);
    currConf << configuration, configuration;

    *velBound = (posBound - currConf)/TIME_STEP;
}

Eigen::VectorXd R2Module::getConfiguration()
{
    Eigen::VectorXd currentConfiguration(jointID.size());
    for(int i=0; i <jointID.size(); ++i)
        currentConfiguration(i) = ( fMotionProxy->getAngles(jointID.at(i), true) ).at(0);

    return currentConfiguration;
}

} // namespace AL
