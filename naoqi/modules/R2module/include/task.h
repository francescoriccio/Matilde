/**
* @class: Task
* This class (and related hierarchy) defines a generic task in the HQP framework.
*
* @file task.h
* @author Claudio Delli Bovi, Chiara Picardi, Francesco Riccio
*/

#ifndef TASK
#define TASK

#include <string>
#include <soth/HCOD.hpp>

#include "libmath/kinChain.h"
#include "configReader.h"
#include "configParams.h"

/** ------------------- TaskBase class declaration ------------------- */
/**
 * TaskBase encodes the task description as matrix equation A * x = b, where:
 * - 'A' is a matrix representing the task constraint in terms of an equation in x;
 * - 'b' is the vector of task bounds and defines the nature of the task (higher bound, lower bound, acceptable range of values).
 * Each task has a priority identified by a numeric value: the lower the value, the higher the priority (max priority = 0).
 * Such value also defines an order among TaskBase class objects and it can be incremented or decremented with the usual postfix/prefix operators.
 */

class TaskBase
{
protected:

    // Task description
    Eigen::MatrixXd constraint_matrix;
    soth::VectorBound bounds;
    // Bound type
    soth::Bound::bound_t boundType;

    // Priority value
    int priority;

public:

    // Constructors
    TaskBase(int _priority, const Eigen::MatrixXd& A, const soth::VectorBound& b);
    TaskBase(int _priority, const Eigen::MatrixXd& A, const Eigen::MatrixXd& b);
    TaskBase(const TaskBase& tb);
    TaskBase(int _priority, const Eigen::MatrixXd& A, const Eigen::VectorXd& b, soth::Bound::bound_t bt);
    // Destructor
    ~TaskBase(){}

    // Priority setter&getter
    inline void setPriority(int _priority){ priority = _priority; }
    inline const int getPriority() const { return priority; }
    // Task getters
    inline const soth::VectorBound& vectorBounds() const { return bounds; }
    inline const Eigen::MatrixXd& constraintMatrix() const { return constraint_matrix; }
    inline const soth::Bound::bound_t getType() const { return boundType; }
    // Task setters
    void setVectorBounds(const Eigen::VectorXd& b, soth::Bound::bound_t type);
    void setVectorBounds( const Eigen::MatrixXd& b );
    void setVectorBounds( const soth::VectorBound& b );
    void setConstraintMatrix( const Eigen::MatrixXd& A );

    // Increment/decrement operators overload
    TaskBase& operator++();
    TaskBase& operator--();
    TaskBase operator++(int);
    TaskBase operator--(int);

    // Comparison operator overload
    friend bool operator<(const TaskBase& l_tb, const TaskBase& r_tb);
    // Stream operator overload
    friend std::ostream& operator<<(std::ostream& out, const TaskBase& tb);
};

// Struct compare for the TaskBase comparison operator overload: useful with ordered STL containers
struct taskCmp
{
    bool operator()( const TaskBase* l_tb, const TaskBase* r_tb ) const
    {
        // Invokes the comparison operator for TaskBase
        return (*l_tb) < (*r_tb);
    }
};


/** ------------------- Task class declaration ------------------- */
/**
 * Task represents a robot pose task (i.e. positioning and orienting the end effector in the task space) within the HQP framework.
 * Along with the task description A * x = b encoded in TaskBase, this class contains the information about the kinematic chain involved in the task.
 * Moreover, Task can only be defined as a constraint of the equality type: it imposes a target pose for the kinematic chain in the form J(q) * q_dot = r
 * where
 * - J(q) is the Jacobian matrix of the kinematic chain;
 * - q_dot is the vector of joint velocities;
 * - r is the task generalized velocity vector.
 * Since Task represents a pose task, r is defined in terms of desired pose as r = K * (CURRENT_POSE - DESIRED_POSE) + DESIRED_VELOCITY.
 * Since the constraint matrix A = J(q) is not static an update function is defined in Task to change A according to the joint configuration q and to update
 * the desired pose and/or desired velocity.
 */

enum status{
    active = 1,
    inactive,
    inactive2active,
    active2inactive
};

class Task : public TaskBase
{
private:

    // The task kinematic chain
    Rmath::KinChain* theKinChain;
    int base_end;

    class Parameters
    {
    public:

        status taskStatus;
        float activationValue;
        float activationStep;

        // Partial solution of the HQP problem without the task
        Eigen::VectorXd qd_n;

        // Toggle positioning/velocity task
        bool positioningActive;
        // Toggle joint space/cartesian space control
        bool jointControlActive;

        // Discrete path to desired pose
        std::vector<Eigen::VectorXd> path;
        // Current progress in the path
        int path_currentStep;
        // Current target velocity
        Eigen::VectorXd targetVelocity;
        // Fixed base transform for the current task
        Eigen::MatrixXd baseT;

        Parameters( status _s, float h, float ts, Eigen::VectorXd tv, Eigen::VectorXd _qd, Eigen::Matrix4d _baseT ):
            taskStatus(_s), activationValue(h), activationStep(ts), qd_n(_qd),
            positioningActive(false), jointControlActive(false),
            path_currentStep(0), targetVelocity(tv), baseT(_baseT){}

        inline void increaseActivationValue()
        {
            activationValue += activationStep;
            if(activationValue >= 1.0)
            {
                activationValue = 1.0;
                taskStatus = active;
            }
        }

        inline void decreaseActivationValue()
        {
            activationValue -= activationStep;
            if(activationValue <= 0.0)
            {
                activationValue = 0.0;
                taskStatus = inactive;
            }
        }

    };

    Parameters parameters;

public:
    // Constructor
    Task(int m, int n,  int _priority, ConfigReader theConfigReader, soth::Bound::bound_t boundType = soth::Bound::BOUND_TWIN );
    Task(int m, int n,  int _priority, const Rmath::KinChain& _kc, int _base = 0, soth::Bound::bound_t boundType = soth::Bound::BOUND_TWIN );
    // Destructor
    ~Task();

    inline Rmath::KinChain* kinChain(){ return theKinChain; }

    inline status taskStatus(){ return parameters.taskStatus; }
    void activate(float activationStep = ACTIVATION_STEP);
    void stop( float decayStep = ACTIVATION_STEP);

    inline float activationValue(){ return parameters.activationValue; }

    void set_qd(Eigen::VectorXd new_qd)
    {
        assert(new_qd.size() == parameters.qd_n.size());
        parameters.qd_n = new_qd;
    }
    void set_baseTransform(const Eigen::Matrix4d& _baseT)
    {
        parameters.baseT = _baseT;
    }

#ifdef TASK_MANAGER_DEBUG
    inline float activationValue() const
    {
        return parameters.activationValue;
    }
#endif

    // Retrieve the current end-effector pose
    Eigen::VectorXd getCurrentPose() const;
    // Retrieve the current target velocity
    inline const Eigen::VectorXd& getTargetVelocity() const
    {
        return parameters.targetVelocity;
    }
    // Retrieve the current target pose
    const Eigen::VectorXd getTargetPose() const;
    // Retrieve the kinchain joints information
    void getJointsIDs(std::map<std::string, int>* jointsIDs) const
    {
        theKinChain->getJointsIDs(jointsIDs);
    }
    // Check if the task is done
    inline bool done() const
    {
        if ((parameters.taskStatus == inactive))
            return false;
        else
            return (parameters.path_currentStep == parameters.path.size()-1);
    }

    // Set a desired velocity in the task space
    inline void setTargetVelocity(const Eigen::VectorXd& r)
    {
        assert(r.size() == parameters.targetVelocity.size());
        parameters.targetVelocity = r;
        parameters.positioningActive = false;
    }
    // Set a desired pose in the task space
    void setDesiredPose( const Eigen::VectorXd& dp, int n_controlPoints = 1 );
    void setDesiredPose( const Eigen::VectorXd& idp, const Eigen::VectorXd& dp, int n_controlPoints = 1 );
    // Set a desired configuration in the joint space
    void setDesiredConfiguration(const Eigen::VectorXd& desiredConf, int n_controlPoints);
    // Reset the current target pose (either in joint or task space)
    inline void resetDesiredPose()
    {
        parameters.positioningActive = false;
        parameters.path.clear();
    }

    // design a circular path
    void circularPathGenerator( const Eigen::VectorXd& dp, float z_shift = 0.0, int n_controlPoints = 1, float radius = 0.0, int n = 1 );

    // Update function
    void update(const Eigen::VectorXd& _q, const Eigen::VectorXd& desiredVel, double K = 1.0 );
};

#endif
