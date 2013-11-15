/**
* @class: Task
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

class TaskBase
{
protected:
    Eigen::MatrixXd constraint_matrix;
    soth::VectorBound bounds;

    int priority;

public:
    TaskBase(int _priority, const Eigen::MatrixXd& A, const soth::VectorBound& b);
    TaskBase(int _priority, const Eigen::MatrixXd& A, const Eigen::MatrixXd& b);
    TaskBase(const TaskBase& tb);
    TaskBase(int _priority, const Eigen::MatrixXd& A, const Eigen::VectorXd& b, soth::Bound::bound_t bt);
    ~TaskBase(){}

    inline void setPriority(int _priority){ priority = _priority; }

    inline const int getPriority() const { return priority; }
    inline const soth::VectorBound& vectorBounds() const { return bounds; }
    inline const Eigen::MatrixXd& constraintMatrix() const { return constraint_matrix; }

    void setVectorBounds( const Eigen::MatrixXd& b );
    void setVectorBounds( const soth::VectorBound& b );
    void setConstraintMatrix( const Eigen::MatrixXd& A );

    TaskBase& operator++();
    TaskBase& operator--();
    TaskBase operator++(int);
    TaskBase operator--(int);
    friend bool operator<(const TaskBase& l_tb, const TaskBase& r_tb);
    friend std::ostream& operator<<(std::ostream& out, const TaskBase& tb);
};

struct taskCmp{
    bool operator()( const TaskBase* l_tb, const TaskBase* r_tb ) const{
        return (*l_tb) < (*r_tb);
    }
};

class Task : public TaskBase
{
private:
    int base;
    int ee;
    Rmath::KinChain* theKinChain;

public:
    Task(std::string cfg_path_file, int n, int m, int _base, int _ee, int _priority = 0);
    ~Task(){}

    soth::VectorBound& chainJointLimits();

    void update(const Eigen::VectorXd& q, double K, const Eigen::VectorXd& desiredPose, const Eigen::VectorXd& desiredVel);
    void update(const Eigen::VectorXd& q, double K, const Eigen::VectorXd& desiredPose, const Eigen::VectorXd& desiredVel,
                const Eigen::MatrixXd& J_base, const Eigen::MatrixXd& H_base);
};

#endif
