#ifndef CONTROLLER_H
#define CONTROLLER_H

/*
  A proportional-integral-derivative controller (PID controller) is a generic control loop feedback mechanism (controller).
  A PID controller calculates an "error" value as the difference between a measured process variable and a desired setpoint.
  The controller attempts to minimize the error by adjusting the process control inputs.

  Here we are using a kind of PD controller (Proportional - Derivative):
  The proportional term produces an output value that is proportional to the current error value. The proportional response can be adjusted
  by multiplying the error by a constant Kp, called the proportional gain constant.
  The derivative of the process error is calculated by determining the slope of the error over time and multiplying this rate of change by the
  derivative gain Kd. The magnitude of the contribution of the derivative term to the overall control action is termed the derivative gain, Kd.

  The class Controller here implemetend provides the linear velocity and both the steering velocities of the body and the head.
  It is updated directly with the output of the vision process, mainly with the x and y component of the desired velocity and the HeadYaw angle
  of the Nao robot retrived from the motion proxy.

  authors: Maria Laura Aceto, Claudio Delli Bovi, Francesco Riccio.
*/

#include <vector>

class Controller{
private:
    float Kw;
    float Kdw;

    float Kw_h;

    float Kdx;
    float Kdy;
    float Kdvx;
    float Kdvy;

    float timeStep;

    std::vector<float> prevLinearVelocity;
    std::vector<float> currLinearVelocity;

    float prevThetaHead;
    float currThetaHead;

    float vx;
    float vy;
    float omegaHead;
    float omegaBody;

    int thetaSign;

public:

    /*
      The default constructor set the several gains of the controller to those values that we tuned during our work.
      Furthermore, it initializes the variables and the time step used in the derivate part of the controller.
    */
    Controller(): Kw(0.95), Kdw(0.125), Kw_h(1.1), Kdx(0.125), Kdy(0.125), Kdvx(0.0625), Kdvy(0.0625){

        currThetaHead = 0;
        prevThetaHead = 0;

        prevLinearVelocity.push_back(0);
        prevLinearVelocity.push_back(0);

        currLinearVelocity.push_back(0);
        currLinearVelocity.push_back(0);

        vx = 0;
        vy = 0;
        omegaBody = 0;
        omegaHead = 0;

        thetaSign = 0;

        timeStep = 0.5;
    }

    /*
      This constructor lets the possiblity to set the controllers gains to desired ones.
      Furthermore, it initializes the variables and the time step used in the derivate part of the controller.
    */
    Controller(float _Kw, float _Kdw, float _Kw_h, float _Kdx, float _Kdy, float _Kdvx = 0, float _Kdvy= 0)
        : Kw(_Kw), Kdw(_Kdw), Kw_h(_Kw_h), Kdx(_Kdx), Kdy(_Kdy), Kdvx(_Kdvx), Kdvy(_Kdvy){

        currThetaHead = 0;
        prevThetaHead = 0;

        prevLinearVelocity.push_back(0);
        prevLinearVelocity.push_back(0);

        currLinearVelocity.push_back(0);
        currLinearVelocity.push_back(0);

        vx = 0;
        vy = 0;
        omegaBody = 0;
        omegaHead = 0;

        thetaSign = 0;

        timeStep = 0.5;
    }

    ~Controller(){}

    /*
      updateHead: The head control law is updated using the theta angle in between the Y-axis and the current desired velocity with a proportional
      controller.
      This method takes as input a measure of the detected obstacles in the images 'reds' and multiplies this amout with theta
      in order to set dynamically the "stiffness" of the HeadYaw angle.

      NOTE: - The class keeps track of the last meaningful sign of the theta angle which will be used in the 'red screen behavior' of the Nao.
            - In order to avoid that the Nao sees its own shoulder and processes noisy images the theta angle is bounded [-M_PI/3, +M_PI/3].
    */
    void updateHead(float _vx, float _vy, float reds = 1){

        //compute theta
        float theta = 0.0;
        if( _vx != 0) theta = atan2(_vy, _vx);

        //set bound on theta
        if(theta >= M_PI/3) theta = M_PI/3;
        else if(theta <= -M_PI/3) theta = -M_PI/3;

        //store theta sign
        if(theta > 0) thetaSign = -1;
        else if(theta < 0) thetaSign = 1;

        //compute the head steering velocity
        omegaHead = -Kw_h* reds* theta;
    }

    /*
      updateBody: The body control law is updated using the x and y component of the desired linear velocity and current theta angle in
      between the head X-axis and the body X-axis of the robot. In this way, the body will follow the head (updated with the desired theta)
      making smooth trajectories while walking (see Section 3.3 of the report).
      All variables are updated with a proportional-derivative controller using their current values computed at time 't' and their previous
      values at 't-1'.
    */
    void updateBody(float _vx, float _vy, float _thetaHead = 0){

        // update the previous theta angle value
        prevThetaHead = currThetaHead;

        // update the previous linear velocities values
        prevLinearVelocity.clear();
        prevLinearVelocity = currLinearVelocity;

        // update the current linear velocities values
        currLinearVelocity.clear();
        currLinearVelocity.push_back(_vx);
        currLinearVelocity.push_back(_vy);

        // update the current theta angle value
        currThetaHead = _thetaHead;

        //compute the body steering velocity
        omegaBody = Kw* currThetaHead + Kdw* (currThetaHead - prevThetaHead)/ timeStep; //iteration time ~0.5s

        //compute the body linear velocities
        vx =  Kdx* currLinearVelocity.at(0) + Kdvx* (currLinearVelocity.at(0) - prevLinearVelocity.at(0))/ timeStep;
        vy = -Kdy* currLinearVelocity.at(1) + Kdvy* (currLinearVelocity.at(1) - prevLinearVelocity.at(1))/ timeStep;
    }

    // getters
    inline int getThetaSign(){ return thetaSign; }

    inline float linearVelocityX(){ return vx; }
    inline float linearVelocityY(){ return vy; }
    inline float bodySteeringVelocity(){ return omegaBody; }
    inline float headSteeringVelocity(){ return omegaHead; }
};

#endif //CONTROLLER
