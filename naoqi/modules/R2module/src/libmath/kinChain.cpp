/**
* @class: KinChain
* @namespace: Rmath
* This class models the kinematic chain of a robotic manipulator
*
* @file kinChain.h
* @author Claudio Delli Bovi, Chiara Picardi, Francesco Riccio
*/

#include <assert.h>

#include "libmath/kinChain.h"

Rmath::KinChain::KinChain(ConfigReader& theConfigReader, int _i, int _j)
{
    transformations = theConfigReader.Transformations();

    std::vector<double> jointBounds_max, jointBounds_min, jointZeroPose;
    theConfigReader.extractJointParams(&jointBounds_min, &jointBounds_max, &jointZeroPose, _i, _j);

    assert(jointBounds_max.size() == jointBounds_min.size());
    assert(jointBounds_max.size() <= transformations.size());
    assert(jointZeroPose.size() >= 0);

    int joint_count = 0;
    for(unsigned int i = 0; i < transformations.size(); ++i)
    {
        if ( transformations.at(i)->getType() == Rmath::dh_transform )
        {
            if(jointBounds_min.size() != 0 && jointBounds_max.size() != 0 && jointZeroPose.size() != 0)
            {
                if(jointZeroPose.at(joint_count) != 0.0)
                    transformations.at(i)->update(jointZeroPose.at(joint_count));

                Rmath::KinChain::Joint current_q(jointZeroPose.at(joint_count),
                                                 jointBounds_max.at(joint_count),
                                                 jointBounds_min.at(joint_count));
                joints[i] = current_q;
            }
            else
            {
                Rmath::KinChain::Joint current_q(0.0);
                joints[i] = current_q;
            }

            ++joint_count;
        }
    }
}

Rmath::KinChain::KinChain(std::vector<Rmath::Transform*> _transf): transformations(_transf)
{
    int joint_count = 0;
    for(unsigned int i = 0; i < transformations.size(); ++i)
    {
        if ( transformations.at(i)->getType() == Rmath::dh_transform )
        {
            Rmath::KinChain::Joint current_q;
            joints[i] = current_q;

            ++joint_count;
        }
    }

}

Rmath::KinChain::KinChain(const Rmath::KinChain& kc)
{
    *this = kc;
}

Rmath::KinChain::~KinChain()
{
    for(unsigned int i = 0; i < transformations.size(); ++i)
        if(transformations.at(i)) delete transformations.at(i);
}

soth::VectorBound& Rmath::KinChain::jointLimits()
{
    soth::VectorBound bounds(joints.size());

    int i = 0;
    std::map<int, Rmath::KinChain::Joint>::iterator joint_reader = joints.begin();
    while (joint_reader != joints.end())
    {
        bounds[i] = soth::Bound((joint_reader->second).min, (joint_reader->second).max);
        ++i;
        ++joint_reader;
    }
}

Eigen::VectorXd Rmath::KinChain::jointConfiguration(int base, int ee)
{
    assert((base >= 0) && (ee < joints.size()));

    Eigen::VectorXd currentConfiguration(ee-base+1);

    int i = 0, j=0;
    std::map<int, Rmath::KinChain::Joint>::iterator joint_reader = joints.begin();
    while (joint_reader != joints.end())
    {
        if ( (j >= base) && (j <= ee) )
        {
            currentConfiguration(i) = (joint_reader->second).value;
            ++i;
        }

        ++joint_reader; ++j;
    }

    return currentConfiguration;
}

Eigen::VectorXd Rmath::KinChain::jointConfiguration()
{
    Eigen::VectorXd currentConfiguration(joints.size());

    int i = 0;
    std::map<int, Rmath::KinChain::Joint>::iterator joint_reader = joints.begin();
    while (joint_reader != joints.end())
    {
        currentConfiguration(i) = (joint_reader->second).value;
        ++joint_reader; ++i;
    }

    return currentConfiguration;
}

void Rmath::KinChain::update(int first, int last, Eigen::VectorXd& q)
{
    assert(q.size() == (last-first+1) );
    assert(q.size() <= joints.size());

    Rmath::trim(&q, 10e-2);

    int i = 0, j = 0;
    std::map<int, Rmath::KinChain::Joint>::iterator joint_updater = joints.begin();
    while (joint_updater != joints.end())
    {
        if ((j >= first) && (j <= last) )
        {
            joint_updater->second = q(i);
            transformations.at(joint_updater->first)->update(q(i));
            ++i;
        }

        ++joint_updater; ++j;
    }
}

void Rmath::KinChain::update(Eigen::VectorXd &q)
{
    assert(q.size() == joints.size());

    Rmath::trim(&q, 10e-2);

    int i = 0;
    std::map<int, Rmath::KinChain::Joint>::iterator joint_updater = joints.begin();
    while (joint_updater != joints.end())
    {
        joint_updater->second = q(i);
        transformations.at(joint_updater->first)->update(q(i));

        ++joint_updater; ++i;
    }
}

void Rmath::KinChain::forward(int base, int ee, Eigen::Matrix4d* H)
{
    assert((base >= 0) && (ee <= transformations.size()));

    std::vector<Rmath::Transform*> subchain(transformations.begin()+base, transformations.begin()+ee);
    Rmath::DirectKin(subchain, H);
}

void Rmath::KinChain::forward(Eigen::Matrix4d* H)
{
    Rmath::DirectKin(transformations, H);
}

void Rmath::KinChain::differential(int base, int ee, Eigen::MatrixXd* J, int taskDim)
{
     assert((base >= 0) && (ee <= transformations.size()));

     std::vector<Rmath::Transform*> subchain(transformations.begin()+base, transformations.begin()+ee);
     std::vector<int> q_indices;

     int i = 0;
     std::map<int, Rmath::KinChain::Joint>::iterator selector = joints.begin();
     while(selector != joints.end())
     {
         if ( (i >= base) && (i <= ee) ) q_indices.push_back(selector->first);
         ++selector; ++i;
     }
     Rmath::geomJacobian(subchain, q_indices, J, taskDim);
}

void Rmath::KinChain::differential(Eigen::MatrixXd* J, int taskDim)
{
    std::vector<int> q_indices;
    std::map<int, Rmath::KinChain::Joint>::iterator selector = joints.begin();
    while(selector != joints.end())
    {
        q_indices.push_back(selector->first);
        ++selector;
    }
    Rmath::geomJacobian(transformations, q_indices, J, taskDim);
}

Rmath::KinChain& Rmath::KinChain::operator=(const Rmath::KinChain& kc)
{
    transformations.clear();
    joints.clear();

    std::map<int, Rmath::KinChain::Joint>::const_iterator joint_iter = kc.joints.begin();
    while (joint_iter != kc.joints.end())
    {
        Rmath::KinChain::Joint current_q((joint_iter->second).value,
                                         (joint_iter->second).max,
                                         (joint_iter->second).min);

        joints[joint_iter->first] = current_q;
        ++joint_iter;
    }

    for(int i=0; i<kc.transformations.size(); ++i)
        transformations.push_back( kc.transformations.at(i)->clone() );
}

Rmath::KinChain& Rmath::KinChain::operator+=(const Rmath::KinChain& kc)
{
    std::map<int, Rmath::KinChain::Joint>::const_iterator joint_iter = kc.joints.begin();
    while (joint_iter != kc.joints.end())
    {
        Rmath::KinChain::Joint current_q((joint_iter->second).value,
                                         (joint_iter->second).max,
                                         (joint_iter->second).min);

        joints[ transformations.size() + joint_iter->first] = current_q;
        ++joint_iter;
    }

    for(int i=0; i<kc.transformations.size(); ++i)
        transformations.push_back( kc.transformations.at(i)->clone() );

    return *this;
}

Rmath::KinChain Rmath::KinChain::operator+(const Rmath::KinChain& kc)
{
    std::vector<Rmath::Transform*> transf;
    for(int i=0; i< transformations.size(); ++i)
        transf.push_back( transformations.at(i)->clone() );

    for(int i=0; i <kc.transformations.size(); ++i)
        transf.push_back( kc.transformations.at(i)->clone() );

    return KinChain(transf);
}

std::ostream& Rmath::operator<<(std::ostream& out, const Rmath::KinChain& kc)
{
    for(unsigned int i = 0; i < kc.transformations.size(); ++i)
        kc.transformations.at(i)->print(out);
    return out;
}

