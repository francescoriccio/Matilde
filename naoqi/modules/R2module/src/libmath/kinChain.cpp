/**
* @class: KinChain
* @namespace: Rmath
* This class models the kinematic chain of a robotic manipulator.
*
* @file kinChain.h
* @author Claudio Delli Bovi, Chiara Picardi, Francesco Riccio
*/

#include <assert.h>

#include "libmath/kinChain.h"

#define INFO(x) std::cout << "[R2 module::KinChain] " << x << std::endl;


/*
 * Class constructor using a ConfigReader class object.
 * The chain of transformation is extracted from the .cfg file using indinces _i, _j and then
 * the private containers are initialized.
 */
Rmath::KinChain::KinChain(ConfigReader& theConfigReader, int _i, int _j)
{
    // Extract the chain of transformations
    transformations = theConfigReader.Transformations();

    // Extract from the general .cfg file all the information about joints
    std::vector<double> jointBounds_max, jointBounds_min, jointZeroPose;
    theConfigReader.extractJointParams(&jointBounds_min, &jointBounds_max, &jointZeroPose, _i, _j);
    // Check for dimensions consistency
    assert(jointBounds_max.size() == jointBounds_min.size());
    assert(jointBounds_max.size() <= transformations.size());
    assert(jointZeroPose.size() >= 0);

    // Detect the actual joints across the kinematic chain and fill the joints container
    int joint_count = 0;
    for(unsigned int i = 0; i < transformations.size(); ++i)
    {
        // A joints is identified by a DH-transform
        if ( transformations.at(i)->getType() == Rmath::dh_transform )
        {
            // Look for data about the current joint
            if(jointBounds_min.size() != 0 && jointBounds_max.size() != 0 && jointZeroPose.size() != 0)
            {
                // Set the zero-value of the joint (if present)
                if(jointZeroPose.at(joint_count) != 0.0)
                    transformations.at(i)->update(jointZeroPose.at(joint_count));
                // Generate a Joint class object and push it in the container
                Rmath::KinChain::Joint current_q(jointZeroPose.at(joint_count),
                                                 jointBounds_max.at(joint_count),
                                                 jointBounds_min.at(joint_count));
                joints[i] = current_q;
            }
            // Create a joint with no mechanical bounds and initial value = 0.0 otherwise
            else
            {
                Rmath::KinChain::Joint current_q (0.0);
                joints[i] = current_q;
            }
            ++joint_count;
        }
    }
}

/*
 * Class constructor overload using a vector of Rmath::Transform.
 */
Rmath::KinChain::KinChain(std::vector<Rmath::Transform*> _transf): transformations(_transf)
{
    // Detect the actual joints across the kinematic chain and fill the joints container
    int joint_count = 0;
    for(unsigned int i = 0; i < transformations.size(); ++i)
    {
        // A joints is identified by a DH-transform
        if ( transformations.at(i)->getType() == Rmath::dh_transform )
        {
            // Create a joint with no mechanical bounds and initial value = 0.0
            Rmath::KinChain::Joint current_q;
            joints[i] = current_q;

            ++joint_count;
        }
    }

}

/*
 * Class copy constructor (NOTE: shallow copy!).
 */
Rmath::KinChain::KinChain(const Rmath::KinChain& kc)
{
    *this = kc;
}

/*
 * Class destructor: deallocates every Transform pointers.
 */
Rmath::KinChain::~KinChain()
{
    for(unsigned int i = 0; i < transformations.size(); ++i)
        if(transformations.at(i)) delete transformations.at(i);
}

/*
 * Returns the joint bounds corresponding to the kinematic chain as a soth::VectorBound class object.
 */
const soth::VectorBound Rmath::KinChain::jointLimits()
{
    soth::VectorBound bounds(joints.size());

    // For each joint in the container, retrieve the lower and higher bound
    int i = 0;
    std::map<int, Rmath::KinChain::Joint>::iterator joint_reader = joints.begin();
    while (joint_reader != joints.end())
    {
        bounds[i] = soth::Bound((joint_reader->second).min, (joint_reader->second).max);
        ++i;
        ++joint_reader;
    }

    return bounds;
}

/*
 * Get the current joint configuration (or any subset of it).
 */
Eigen::VectorXd Rmath::KinChain::jointConfiguration(int base, int ee)
{
    // Check indices consistency
    assert((base >= 0) && (ee < joints.size()));

    // Store the configuration in a vector
    Eigen::VectorXd currentConfiguration(ee-base+1);
    int i = 0, j=0;
    // Iterate over the joint container
    std::map<int, Rmath::KinChain::Joint>::iterator joint_reader = joints.begin();
    while (joint_reader != joints.end())
    {
        // Consider only the joint in the desired range
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
    // Store the configuration in a vector
    Eigen::VectorXd currentConfiguration(joints.size());
    int i = 0;
    // Iterate over the joint container
    std::map<int, Rmath::KinChain::Joint>::iterator joint_reader = joints.begin();
    while (joint_reader != joints.end())
    {
        currentConfiguration(i) = (joint_reader->second).value;
        ++joint_reader; ++i;
    }

    return currentConfiguration;
}

/*
 * Update the current joint configuration (or any subset of it).
 */
void Rmath::KinChain::update(int first, int last, Eigen::VectorXd& q)
{
    // Check for indices consistency
    assert(q.size() == (last-first+1) );
    assert(q.size() <= joints.size());

    int i = 0, j = 0;
    // Iterate over the joint container
    std::map<int, Rmath::KinChain::Joint>::iterator joint_updater = joints.begin();
    while (joint_updater != joints.end())
    {
        // Skip joints out of the desired range
        if ((j >= first) && (j <= last) )
        {
            // Update using the Transform base class method
            joint_updater->second = q(i);
            transformations.at(joint_updater->first)->update(q(i));
            ++i;
        }
        ++joint_updater; ++j;
    }
}

void Rmath::KinChain::update(const Eigen::VectorXd &q)
{
    // Check for dimensions consistency
    assert(q.size() == joints.size());

    int i = 0;
    // Iterate over the joint container
    std::map<int, Rmath::KinChain::Joint>::iterator joint_updater = joints.begin();
    while (joint_updater != joints.end())
    {
        // Update using the Transform base class method
        joint_updater->second = q(i);
        transformations.at(joint_updater->first)->update(q(i));

        ++joint_updater; ++i;
    }
}


/*
 * Compute the forward kinematics of the chain (or any subchain of it) as a homogeneous transformation matrix.
 */
void Rmath::KinChain::forward(int base, int ee, Eigen::Matrix4d* H)
{
    // Check for indices consistency
    assert((base >= 0) && (ee <= transformations.size()));
    // Extract the desired subchain
    std::vector<Rmath::Transform*> subchain(transformations.begin()+base, transformations.begin()+ee);
    // Compute the transformation matrix
    Rmath::DirectKin(subchain, H);
}

void Rmath::KinChain::forward(Eigen::Matrix4d* H)
{
    // Compute the transformation matrix relative to the whole chain
    Rmath::DirectKin(transformations, H);
}

/*
 * Compute the forward differential kinematics of the chain (or any subchain of it).
 */
void Rmath::KinChain::differential(int base, int ee, Eigen::MatrixXd* J, int taskDim)
{
    // Check for indices consistency
     assert((base >= 0) && (ee <= transformations.size()));
     // Extract the desired subchain
     std::vector<Rmath::Transform*> subchain(transformations.begin()+base, transformations.begin()+ee);

     // Collect all joint positions within the chain
     std::vector<int> q_indices;
     std::map<int, Rmath::KinChain::Joint>::iterator selector = joints.begin();

     // Iterate over the joint container
     while(selector != joints.end())
     {
         // Skip joints out of the desired range
         if ( selector->first >= base && selector->first <= ee )
             q_indices.push_back(selector->first -base);

         ++selector;
     }

     // Compute the Jacobian matrix
     Rmath::geomJacobian(subchain, q_indices, J, taskDim);
}

void Rmath::KinChain::differential(Eigen::MatrixXd* J, int taskDim)
{
    // Collect all joint positions within the chain
    std::vector<int> q_indices;

    std::map<int, Rmath::KinChain::Joint>::iterator selector = joints.begin();
    // Iterate over the joint container
    while( selector != joints.end() )
    {
        q_indices.push_back(selector->first);
        ++selector;
    }

    // Compute the Jacobian matrix
    Rmath::geomJacobian(transformations, q_indices, J, taskDim);
}

/*
 * Overload of operator equal.
 */
Rmath::KinChain& Rmath::KinChain::operator=(const Rmath::KinChain& kc)
{
    // Reset the containers
    transformations.clear();
    joints.clear();

    // Re-fill the joint container using the input KinChain object
    std::map<int, Rmath::KinChain::Joint>::const_iterator joint_iter = kc.joints.begin();
    while (joint_iter != kc.joints.end())
    {
        Rmath::KinChain::Joint current_q((joint_iter->second).value,
                                         (joint_iter->second).max,
                                         (joint_iter->second).min);

        joints[joint_iter->first] = current_q;
        ++joint_iter;
    }

    // Re-fill the vector of Transform* using the input KinChain object (NOTE: shallow copy!)
    for(int i=0; i<kc.transformations.size(); ++i)
        transformations.push_back( kc.transformations.at(i) );
}

/*
 * Overload of operator +=: append the given KinChan object to *this (NOTE: deep copy!).
 */
Rmath::KinChain& Rmath::KinChain::operator+=(const Rmath::KinChain& kc)
{
    // Add the new set of joints to the container
    std::map<int, Rmath::KinChain::Joint>::const_iterator joint_iter = kc.joints.begin();
    while (joint_iter != kc.joints.end())
    {
        Rmath::KinChain::Joint current_q((joint_iter->second).value,
                                         (joint_iter->second).max,
                                         (joint_iter->second).min);
        // Indices are shifted by the size of the precedent chain
        joints[ transformations.size() + joint_iter->first] = current_q;
        ++joint_iter;
    }

    // Add a copy of each new Transform to the chain
    for(int i=0; i<kc.transformations.size(); ++i)
        transformations.push_back( kc.transformations.at(i)->clone() );

    return *this;
}

/*
 * Overload of operator +: create a new KinChain object by appending kc to *this (NOTE: deep copy!).
 */
Rmath::KinChain Rmath::KinChain::operator+(const Rmath::KinChain& kc)
{
    // First insert copies of the Transform objects of *this
    std::vector<Rmath::Transform*> transf;
    for(int i=0; i< transformations.size(); ++i)
        transf.push_back( transformations.at(i)->clone() );
    // Then add the remaining copies from the given KinChain object
    for(int i=0; i <kc.transformations.size(); ++i)
        transf.push_back( kc.transformations.at(i)->clone() );

    // NOTE: any information on the joint bounds is lost (TO FIX.
    return KinChain(transf);
}

/*
 * Overload of operator <<: print every Transform of the chain in sequence.
 */
std::ostream& Rmath::operator<<(std::ostream& out, const Rmath::KinChain& kc)
{
    for(unsigned int i = 0; i < kc.transformations.size(); ++i)
        kc.transformations.at(i)->print(out);
    return out;
}

