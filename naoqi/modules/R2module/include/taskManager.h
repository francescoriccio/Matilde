/**
* @class: TaskManager
* This class defines a scheduler for tasks in the HQP framework.
*
* @file taskManager.h
* @author Claudio Delli Bovi, Chiara Picardi, Francesco Riccio
*/

#ifndef TASK_MANAGER
#define TASK_MANAGER

#include <string>
#include <map>
#include <set>
#include "Eigen/Core"

#include "task.h"
#include "configParams.h"


class R2Module;

class TaskManager
{

private:

    // Pointer to the robot module
    R2Module* module_ptr;

    // Joint limits primary task
    TaskBase* jointLimits;
    // All initialized tasks
    std::map <std::string, Task*> taskMap;
    // Stack of tasks currently active
    std::set<Task*, taskCmp> taskSet;

    // Update a single task
    void taskUpdate( const std::string& taskName, const Eigen::VectorXd& q );
    // Update all tasks fully enabled
    void updateActiveTasks( const Eigen::VectorXd& q );
    // Update all tasks in a transition phase
    void updateIntermediateTasks( const Eigen::VectorXd& q, const Eigen::VectorXd& partial_qdot );

    // Compute a partial HQP solution involving only fully enabled tasks
    void computePartialSolution(const Eigen::VectorXd& q, Eigen::VectorXd* partial_qdot);
    // Compute the final HQP involving all enabled tasks
    void computeCompleteSolution(const Eigen::VectorXd& q, const Eigen::VectorXd& partial_qdot, Eigen::VectorXd* qdot);

public:

    // Constructor
    TaskManager( R2Module* m_ptr ): module_ptr(m_ptr){}
    // Destructor
    ~TaskManager();

    // Set joint limits
    void setJointLimits(const Eigen::MatrixXd& velBounds);
    // Declare a new task
    inline void createTask(const std::string& taskName, Task* taskPtr)
    {
        taskMap.insert( std::pair<std::string, Task*>(taskName,taskPtr) );
    }
    // Retrieve a reference to specific task
    inline Task& task(const std::string& taskName)
    {
        assert(taskMap.find(taskName) != taskMap.end());
        return *(taskMap[taskName]);
    }

    // Solve the HQP problem with the current task set
    void exec(const Eigen::VectorXd& q, Eigen::VectorXd* qdot);
};

#endif
