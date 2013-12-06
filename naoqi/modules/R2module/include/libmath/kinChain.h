/**
* @class: KinChain
* @namespace: Rmath
* This class models the kinematic chain of a robotic manipulator.
*
* @file kinChain.h
* @author Claudio Delli Bovi, Chiara Picardi, Francesco Riccio
*/

#ifndef KIN_CHAIN
#define KIN_CHAIN

#include <cmath>
#include <vector>
#include <map>
#include <Eigen/Core>
#include <Eigen/LU>
#include <soth/HCOD.hpp>

#include "configReader.h"
#include "libmath/Rutils.h"

namespace Rmath
{


/** ---------------------------- Kinematic chain class declaration ---------------------------- */


class KinChain
{

protected:

    // Chain of transformations
    std::vector<Rmath::Transform*> transformations;

    // Inner class describing a single joint and its mechanical limits
    class Joint
    {
    public:

        double value;
        double max;
        double min;

        Joint(double _value = 0.0, double _max = Eigen::Infinity, double _min = -Eigen::Infinity):
            value(_value), max(_max), min(_min) {}
    };
    // Indices of the robot joints within the chain
    std::map<int, Joint> joints;


public:

    // Constructors
    KinChain(ConfigReader& theConfigReader, int _i, int _j);
    KinChain(std::vector<Rmath::Transform*> _transf);
    // Copy constructors
    KinChain(const Rmath::KinChain& kc);
    // Destructors
    ~KinChain();

    // Getters for the current joint configuration
    Eigen::VectorXd jointConfiguration(int base, int ee);
    Eigen::VectorXd jointConfiguration();
    // Getter for the joint limits
    const soth::VectorBound jointLimits();

    // Joint configuration update
    void update(int first, int last, Eigen::VectorXd &q);
    void update(const Eigen::VectorXd& q);

    // Forward kinematics computation
    void forward(int base, int ee, Eigen::Matrix4d* H);
    void forward(Eigen::Matrix4d* H);

    // Differential kinematics computation
    void differential(int base, int ee, Eigen::MatrixXd* J, int taskDim = 6);
    void differential(Eigen::MatrixXd* J, int taskDim = 6);

    // Operators overload
    KinChain& operator=(const KinChain& kc);
    KinChain& operator+=(const KinChain& kc);
    KinChain operator+(const KinChain& kc);
    friend std::ostream& operator<<(std::ostream& out, const KinChain& kc);

};

}

#endif
