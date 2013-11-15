#include "amrModule.h"

#include <iostream>

#include <alvision/alvisiondefinitions.h>
#include <alvision/alimage.h>
#include <alcommon/albroker.h>

#include <qi/log.hpp>

#define FRAME_INTERVAL 300 // [ms]
#define IMAGE_AVG_NUM 1

#define FEED_FORWARD 10
#define RED_SCREEN_TRIGGER 2
#define RED_SCREEN_ANGLE 0.17

namespace AL
{

AmrModule::AmrModule( boost::shared_ptr<ALBroker> pBroker, const std::string& pName ):
  ALModule(pBroker , pName), fRegisteredToVideoDevice(false){
  // Describe the module here.
  setModuleDescription("AMR Project");

  //Define bound methods with their description.
  functionName( "registerToVideoDevice", getName(), "Register to the V.I.M." );
  BIND_METHOD( AmrModule::registerToVideoDevice );

  functionName( "unRegisterFromVideoDevice", getName(), "Unregister from the V.I.M." );
  BIND_METHOD( AmrModule::unRegisterFromVideoDevice );

  functionName( "getCurrentFrame", getName(), "Save an image received from the camera." );
  BIND_METHOD( AmrModule::getCurrentFrame );
}

void AmrModule::exit(){
  AL::ALModule::exit();
}

void AmrModule::init(){

  // Create a proxy to ALVideoDevice, ALMotion, ALRobotPosture.
  try {
    fCamProxy = boost::shared_ptr<ALVideoDeviceProxy>(new ALVideoDeviceProxy(getParentBroker()));
    fMotionProxy = boost::shared_ptr<ALMotionProxy>(new ALMotionProxy(getParentBroker()));
    fPostureProxy = boost::shared_ptr<ALRobotPostureProxy>(new ALRobotPostureProxy(getParentBroker()));

  } catch (const AL::ALError& e) {
    qiLogError("vision.AmrModule") << "Error while getting proxy on ALVideoDevice.  Error msg " << e.toString() << std::endl;
    AmrModule::exit();
    return;
  }

  if(!fCamProxy || !fMotionProxy || !fPostureProxy)
  {
    qiLogError("vision.AmrModule") << "Error while getting proxy on ALVideoDevice. Check ALVideoDevice is running." << std::endl;
    AmrModule::exit();
    return;
  }

  thePotentialField = new PotentialField(0, -10, true);
  theController = new Controller();

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
      // initialize images
      fprevImageHeader = cv::Mat::zeros(imgHeight, imgWidth, CV_8UC1);
      fcurrImageHeader = cv::Mat::zeros(imgHeight, imgWidth, CV_8UC1);

      // create the camera thread
//      pthread_create( &cameraThreadId, NULL, (void*(*)(void*)) visionTh, this);
  }

}

AmrModule::~AmrModule(){

    // Unregister the video module.
    try{
        if(fCamProxy)
            fCamProxy->unsubscribe(fVideoClientName);

        fCamProxy.reset();
        fMotionProxy.reset();
        fPostureProxy.reset();
    }
    catch(const AL::ALError& e){
        qiLogError("vision.AmrModule") <<  e.toString() << std::endl;
    }

    // destroy thread
    pthread_cancel(motionThreadId);
    pthread_cancel(cameraThreadId);
}

void AmrModule::registerToVideoDevice(){

  // If we've already registered a module, we need to unregister it first !
  if (fRegisteredToVideoDevice) {
    throw ALError(getName(), "registerToVideoDevice()", "A video module has already been "
      "registered. Call unRegisterFromVideoDevice() before trying to register a new module.");
  }

  const std::string kOriginalName = "amrModule";
  const int kFps = 30;

  // Release Image Headers if they have been allocated.
  if (!fcurrImageHeader.empty())
    fcurrImageHeader.release();
  if (!fprevImageHeader.empty())
    fprevImageHeader.release();

  if(fCamProxy)
      fVideoClientName = fCamProxy->subscribeCamera(kOriginalName, 1, pResolution, pColorSpace, kFps);


  qiLogInfo("vision.amrModule") << "Module registered as " << fVideoClientName << std::endl;

  // Registration is successful, set fRegisteredToVim to true.
  fRegisteredToVideoDevice = true;
}

void AmrModule::unRegisterFromVideoDevice(){

  if (!fRegisteredToVideoDevice) {
    throw ALError(getName(), "unRegisterFromVideoDevice()", "No video module is currently "
      "registered! Call registerToVideoDevice first.");
  }

  // Release Image Headers if they have been allocated.
  if (!fcurrImageHeader.empty())
    fcurrImageHeader.release();
  if (!fprevImageHeader.empty())
    fprevImageHeader.release();

  qiLogInfo("vision.AmrModule") << "try to unregister " << fVideoClientName << " module." << std::endl;
  if(fCamProxy)
    fCamProxy->unsubscribe(fVideoClientName);

  qiLogInfo("vision.AmrModule") << "Done." << std::endl;

  // UnRegistration is successful, set fRegisteredToVim to false.
  fRegisteredToVideoDevice = false;
}

void AmrModule::motion(){

    qi::os::sleep(0.5f);

    // initialize posture proxy
    fPostureProxy->goToPosture("StandInit", 0.5f);

    int redScreenCount = 0;
    bool turn = false;

    while(true){

        mutex.lock();
        // retrieve computed velocity and "reds" percentage
        std::vector<float> vel = thePotentialField->getVelocity();
        float reds = thePotentialField->getRedsPercentage();
        mutex.unlock();

        // update head controller
        theController->updateHead(vel.at(0), vel.at(1), reds);

        // update reds count
        if( reds >= 0.9 ) ++redScreenCount;
        else redScreenCount = 0;

        // move Nao's head
        moveHead(redScreenCount);

        // update body controller
        theController->updateBody( vel.at(0), vel.at(1), (fMotionProxy->getAngles("HeadYaw", true) ).at(0) );

        // update turn variable
        if(vel.at(1) > 0.25 || vel.at(1) < -0.25) turn = true;
        else turn = false;

        // make Nao walking
        walk(turn);
    }

}

void AmrModule::walk(bool turn){

    fMotionProxy->move( 0.4, 0.0, 0.0);
    // call 'move' method through the motion proxy using the controller output
//    if(turn) fMotionProxy->move( theController->linearVelocityX(), theController->linearVelocityY(), theController->bodySteeringVelocity());
//    else fMotionProxy->move( theController->linearVelocityX(), 0.0, theController->bodySteeringVelocity());
}

void AmrModule::moveHead(int redScreenCount){

    // initialize motion proxy
    fMotionProxy->stiffnessInterpolation("HeadYaw", 1.0f, 1.0f);

    // update the angle using the controller output
    AL::ALValue angle;
    if( redScreenCount > RED_SCREEN_TRIGGER)
        angle = AL::ALValue::array( theController->getThetaSign()* RED_SCREEN_ANGLE );
    else
        angle = AL::ALValue::array( theController->headSteeringVelocity() );

    // set stiffness
    fMotionProxy->setStiffnesses("HeadYaw", AL::ALValue::array(1.0f));
    qi::os::sleep(.1f);

    // call 'setAngles' method through the motion proxy
    fMotionProxy->setAngles("HeadYaw", 0.0, 0.1f);
    qi::os::sleep(.1f);

    // reset stiffness
    fMotionProxy->setStiffnesses("HeadYaw", AL::ALValue::array(0.0f));
}

void AmrModule::computePotentialField(){

    qi::os::sleep(1.0f);

    while(true){

        // update previous frame
        fprevImageHeader = fcurrImageHeader.clone();

        // artificial delay introduced in between two consecutive frames
        qi::os::msleep(FRAME_INTERVAL);

        // obtain the new frame
        getCurrentFrame();

        mutex.lock();

        // call the PotentialField update method to generate the new desired velocities
        thePotentialField->update(fprevImageHeader, fcurrImageHeader,
                                  thePotentialField->getVelocity().at(1), thePotentialField->getVelocity().at(0) +FEED_FORWARD );
        mutex.unlock();
    }
}

void AmrModule::getCurrentFrame(){

//  // Check that a video module has been registered.
//  if (!fRegisteredToVideoDevice) {
//    throw ALError(getName(), "saveImageRemote()",  "No video module is currently "
//      "registered! Call registerToVideoDevice() first.");
//  }

//  cv::Mat averageFrame = cv::Mat::zeros(fcurrImageHeader.rows, fcurrImageHeader.cols, CV_8UC1);

//  int count =0;

//  // performe an average over a given number of images
//  while(count < IMAGE_AVG_NUM)
//  {
//      ALValue frame = fCamProxy->getImageRemote(fVideoClientName);

//      if (frame.getType()!= ALValue::TypeArray && frame.getSize() != 7) {
//        throw ALError(getName(), "getImages", "Invalid image returned.");
//      }

//      const long long timeStamp = ((long long)(int)frame[4])*1000000LL + ((long long)(int)frame[5]);
//      time = (int)(timeStamp/1000000LL);

////      fcurrImageHeader.data = (uchar*) frame[6].GetBinary();
//      uchar* data = (uchar*) frame[6].GetBinary();

//      for(int i=0; i< fcurrImageHeader.rows; ++i){
//          for(int j=0; j< fcurrImageHeader.cols; ++j){
//              averageFrame.at<uchar>(i, j) += data[j+ i*fcurrImageHeader.cols];
//          }
//      }

//      ++count;
//  }

//  // store the resulting image in its proper container, namely 'fcurrImageHeader'.
//  for(int i=0; i< fcurrImageHeader.rows; ++i){
//      for(int j=0; j< fcurrImageHeader.cols; ++j){
//          fcurrImageHeader.at<uchar>(i, j) = averageFrame.at<uchar>(i, j) / IMAGE_AVG_NUM;
//      }
//  }

//  fCamProxy->releaseImage(fVideoClientName);

}

} // namespace AL
