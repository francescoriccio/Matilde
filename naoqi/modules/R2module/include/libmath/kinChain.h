/**
* @class: KinChain
* @namespace: Rmath
* This class models the kinematic chain of a robotic manipulator
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
#include <soth/HCOD.hpp>

#include "configReader.h"
#include "libmath/Rutils.h"

namespace Rmath
{

class KinChain
{
protected:

    std::vector<Rmath::Transform*> transformations;

    class Joint
    {
    public:
        double value;
        double max;
        double min;

        Joint(double _value = 0.0, double _max = Eigen::Infinity, double _min = -Eigen::Infinity):
            value(_value), max(_max), min(_min) {}
    };

    std::map<int, Joint> joints;


public:

    KinChain(ConfigReader& theConfigReader, int _i, int _j);
    KinChain(std::vector<Rmath::Transform*> _transf);
    KinChain(const Rmath::KinChain& kc);

    ~KinChain();

    Eigen::VectorXd jointConfiguration(int base, int ee);
    Eigen::VectorXd jointConfiguration();

    soth::VectorBound& jointLimits();

    void update(int first, int last, Eigen::VectorXd &q);
    void update(Eigen::VectorXd& q);

    void forward(int base, int ee, Eigen::Matrix4d* H);
    void forward(Eigen::Matrix4d* H);

    void differential(int base, int ee, Eigen::MatrixXd* J, int taskDim = 6);
    void differential(Eigen::MatrixXd* J, int taskDim = 6);

    KinChain& operator=(const KinChain& kc);
    KinChain& operator+=(const KinChain& kc);
    KinChain operator+(const KinChain& kc);
    friend std::ostream& operator<<(std::ostream& out, const KinChain& kc);

};

}

#endif
