/**
* @class: Task
* This class (and related hierarchy) defines a generic task in the HQP framework.
*
* @file transform.h
* @author Claudio Delli Bovi, Chiara Picardi, Francesco Riccio
*/

#ifndef TASK
#define TASK

#include <string>
#include <soth/HCOD.hpp>

#include "libmath/kinChain.h"
#include "configReader.h"

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
    // Priority value
    int priority;
    // Task status
    bool active;

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
    inline bool isActive() const { return active; }
    // Status togglers
    inline void activate() { active = true; }
    inline void stop() { active = false; }
    // Task setters
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


class Task : public TaskBase
{

private:

    // The task kinematic chain
    Rmath::KinChain* theKinChain;

public:
    // Constructor
    Task(int m, int n,  int _priority, ConfigReader theConfigReader, int _base, int _ee);
    // Destructor
    ~Task();

    // Update function
    void update(const Eigen::VectorXd& q, double K, const Eigen::VectorXd& desiredPose, const Eigen::VectorXd& desiredVel);
};

#endif
