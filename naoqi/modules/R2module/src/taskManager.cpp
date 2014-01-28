/**
* @class: TaskManager
* This class implements a scheduler for tasks in the HQP framework.
*
* @file taskManager.h
* @author Claudio Delli Bovi, Chiara Picardi, Francesco Riccio
*/

#include <iostream>
#include <soth/HCOD.hpp>

#include "r2Module.h"

//#define INFO(x) std::cerr << "\033[22;34;1m" << "[TaskManager] " << x << "\033[0m" << std::endl;
#define INFO(x) std::cout << x << std::endl;
using namespace AL;

void R2Module::TaskManager::taskUpdate(const std::string& taskName, const Eigen::VectorXd& q)
{
    if( taskName == HEAD_TASK)
    {
        // Head task update
        Eigen::VectorXd q_H (LLEG_CHAIN_SIZE+HEAD_CHAIN_SIZE);
        q_H << q.segment<LLEG_CHAIN_SIZE>(LLEG_CHAIN_BEGIN),
               q.segment<HEAD_CHAIN_SIZE>(HEAD_CHAIN_BEGIN);
        ( taskMap[taskName] )->update( q_H, Eigen::VectorXd::Zero(HEAD_TASK_DIM), K_HEAD );

#ifdef DEBUG_MODE
        INFO("Head task constraint equation: ");
        INFO(std::endl << *( taskMap[taskName] ) );
#endif
    }

    else if(taskName == LEFT_ARM)
    {
        // Left arm task update
        Eigen::VectorXd q_LA (LLEG_CHAIN_SIZE+RARM_CHAIN_SIZE);
        q_LA << q.segment<LLEG_CHAIN_SIZE>(LLEG_CHAIN_BEGIN),
                q.segment<LARM_CHAIN_SIZE>(LARM_CHAIN_BEGIN);
        ( taskMap[taskName] )->update( q_LA, Eigen::VectorXd::Zero(LARM_TASK_DIM), K_LARM );

    }
//    else if( (taskName == "LHolySpirit") || (taskName == "LJesus") || (taskName == "LMary") )
//    {
//        // Left arm task update
//        Eigen::VectorXd q_LA (LARM_CHAIN_SIZE);
//        q_LA << q.segment<LARM_CHAIN_SIZE>(LARM_CHAIN_BEGIN);
//        ( taskMap[taskName] )->update( q_LA, Eigen::VectorXd::Zero(LARM_TASK_DIM), K_LARM );

//#ifdef DEBUG_MODE
//        INFO("Left arm task constraint equation: ");
//        INFO(std::endl << *( taskMap[taskName] ) );
//#endif
//    }
//    else if( (taskName == "RHolySpirit") || (taskName == "RJesus") || (taskName == "RMary") )
//    {
//        // Left arm task update
//        Eigen::VectorXd q_RA (RARM_CHAIN_SIZE);
//        q_RA << q.segment<RARM_CHAIN_SIZE>(RARM_CHAIN_BEGIN);
//        ( taskMap[taskName] )->update( q_RA, Eigen::VectorXd::Zero(RARM_TASK_DIM), K_RARM );

//#ifdef DEBUG_MODE
//        INFO("Right arm task constraint equation: ");
//        INFO(std::endl << *( taskMap[taskName] ) );
//#endif
//    }

    else if( (taskName == RIGHT_ARM) || (taskName == "Rbound") )
    {
        // Right arm task update
        Eigen::VectorXd q_RA (LLEG_CHAIN_SIZE+RARM_CHAIN_SIZE);
        q_RA << q.segment<LLEG_CHAIN_SIZE>(LLEG_CHAIN_BEGIN),
                q.segment<RARM_CHAIN_SIZE>(RARM_CHAIN_BEGIN);

#ifdef RARM_LARM_JOINT_TASK
        if (taskName == "Rbound")
            ( taskMap[taskName] )->update( q_RA, Eigen::VectorXd::Zero(RARM_TASK_DIM), K_RARM );
        // Right arm mimic task
        else if ( ARMS_TASK == MIMIC_TASK )
        {
            Eigen::VectorXd desiredRHandPose = ( taskMap[LEFT_ARM] )->getTargetPose();
            assert(RARM_TASK_DIM == LARM_TASK_DIM);
            assert(LARM_TASK_DIM >= 2);
            desiredRHandPose(1) -= MINIMUM_HANDS_DISTANCE;

            if (LARM_TASK_DIM > 3)
                for (int i=3; i < LARM_TASK_DIM; ++i)
                    desiredRHandPose(i) = -desiredRHandPose(i);

            ( taskMap[taskName] )->setDesiredPose(desiredRHandPose, 1 );
            ( taskMap[taskName] )->update( q_RA, Eigen::VectorXd::Zero(RARM_TASK_DIM), K_RARM );

        }
        // Right arm mirror task
        else if ( ARMS_TASK == MIRROR_TASK )
        {
            Eigen::VectorXd desiredRHandVel = ( taskMap[LEFT_ARM] )->getTargetVelocity();

            assert(RARM_TASK_DIM == LARM_TASK_DIM);
            assert(LARM_TASK_DIM >= 2);
            desiredRHandVel(1) = -desiredRHandVel(1);

            if (LARM_TASK_DIM > 3)
                for (int i=3; i < LARM_TASK_DIM; ++i)
                    desiredRHandVel(i) = -desiredRHandVel(i);

            ( taskMap[taskName] )->setTargetVelocity(desiredRHandVel);
            ( taskMap[taskName] )->update( q_RA, desiredRHandVel, K_RARM );
        }
#else
        ( taskMap[taskName] )->update( q_RA, Eigen::VectorXd::Zero(RARM_TASK_DIM), K_RARM );
#endif

#ifdef DEBUG_MODE
        INFO("Right arm task constraint equation: ");
        INFO(std::endl << *( taskMap[taskName] ) );
#endif
    }
//    else if ( (taskName == "rleg1") || (taskName == "rleg2") )
//        ( taskMap[taskName] )->update( q.segment<RLEG_CHAIN_SIZE>(RLEG_CHAIN_BEGIN), Eigen::VectorXd::Zero(RLEG_TASK_DIM), K_RLEG );
    else if( (taskName == RIGHT_LEG) )
    {
        // Right leg task update
#ifndef UP_DOWN_TASK
        ( taskMap[taskName] )->update( q.segment<RLEG_CHAIN_SIZE>(RLEG_CHAIN_BEGIN), Eigen::VectorXd::Zero(RLEG_TASK_DIM), K_RLEG );
#else
        ( taskMap[taskName] )->update( q.segment<RLEG_CHAIN_SIZE>(RLEG_CHAIN_BEGIN), Eigen::VectorXd::Zero(RLEG_TASK_DIM), 0.4 );
#endif

#ifdef DEBUG_MODE
        INFO("Right leg task constraint equation: ");
        INFO(std::endl << *( taskMap[taskName] ) );
#endif
    }
//    else if ( (taskName == "lleg1") || (taskName == "lleg2") )
//        ( taskMap[taskName] )->update( q.segment<LLEG_CHAIN_SIZE>(LLEG_CHAIN_BEGIN), Eigen::VectorXd::Zero(LLEG_TASK_DIM), K_LLEG );
    else if( (taskName == LEFT_LEG) )
    {
#ifndef UP_DOWN_TASK
        // Left leg task update
        ( taskMap[taskName] )->update( q.segment<LLEG_CHAIN_SIZE>(LLEG_CHAIN_BEGIN), Eigen::VectorXd::Zero(LLEG_TASK_DIM), K_LLEG );
#else
        ( taskMap[taskName] )->update( q.segment<RLEG_CHAIN_SIZE>(RLEG_CHAIN_BEGIN), Eigen::VectorXd::Zero(LLEG_TASK_DIM), 0.4 );
#endif
#ifdef DEBUG_MODE
        INFO("Left leg task constraint equation: ");
        INFO(std::endl << *( taskMap[taskName] ) );
#endif
    }
}

void R2Module::TaskManager::updateActiveTasks(const Eigen::VectorXd& q)
{
    std::map<std::string, Task*>::iterator updater=taskMap.begin();
    while(updater != taskMap.end())
    {
        if( (updater->second)->taskStatus() == active )
        {
            taskUpdate(updater->first, q );
            taskSet.insert(updater->second);
        }

        ++updater;
    }
}

void R2Module::TaskManager::updateIntermediateTasks(const Eigen::VectorXd& q, const Eigen::VectorXd& partial_qdot )
{
    std::map<std::string, Task*>::iterator updater=taskMap.begin();
    while(updater != taskMap.end())
    {
        if( (updater->second)->taskStatus() == inactive2active || (updater->second)->taskStatus() == active2inactive )
        {
            // Retrieve current kinchain joints information
            std::map<std::string, int> jointsIndicesMap;
            (updater->second)->getJointsIDs(&jointsIndicesMap);
            Eigen::VectorXd current_qd((updater->second)->constraintMatrix().cols());

            for (unsigned int i = 0; i < module_ptr->jointID.size(); ++i)
                if (jointsIndicesMap.find(module_ptr->jointID.at(i)) != jointsIndicesMap.end())
                    current_qd(jointsIndicesMap[module_ptr->jointID.at(i)]) = partial_qdot(i);

            // qd_n <- cropped qdot
            (updater->second)->set_qd(current_qd);

            taskUpdate(updater->first, q );
            taskSet.insert(updater->second);
        }

        ++updater;
    }
}

void R2Module::TaskManager::computePartialSolution(const Eigen::VectorXd& q, Eigen::VectorXd* partial_qdot)
{
    assert(partial_qdot->size() == q.size());

    updateActiveTasks(q);

    // Initialize the HQP solver
    soth::HCOD hsolver( q.size(), taskSet.size()+1 );

    // Set up the hierarchy
    std::vector<Eigen::MatrixXd> A;
    std::vector<soth::VectorBound> b;
    // Joint limits first (top priority task)
    A.push_back(jointLimits->constraintMatrix());
    b.push_back(jointLimits->vectorBounds());
    // All the remaining tasks (in descending order of priority)
    std::set<Task*>::iterator updater = taskSet.begin();
    while( updater != taskSet.end() )
    {
        // Retrieve current kinchain joints information
        std::map<std::string, int> jointsIndicesMap;
        (*updater)->getJointsIDs(&jointsIndicesMap);
        // Current task constraint matrix (restricted to the joint involved)
        Eigen::MatrixXd currentA_task = (*updater)->constraintMatrix();

        // Rewrite the task constraint matrix with the complete joint configuration
        Eigen::MatrixXd currentA = Eigen::MatrixXd::Zero( currentA_task.rows(), JOINTS_NUM );
        // The task constraint matrix is the only non-zero block in A
        for (unsigned int i = 0; i <module_ptr->jointID.size(); ++i)
        {
            if (jointsIndicesMap.find(module_ptr->jointID.at(i)) != jointsIndicesMap.end())
                currentA.col(i) = currentA_task.col( jointsIndicesMap[module_ptr->jointID.at(i)] );
            else
                currentA.col(i) = Eigen::VectorXd::Zero(currentA_task.rows());
        }

        A.push_back(currentA);
        b.push_back((*updater)->vectorBounds());
        ++updater;
    }

    hsolver.pushBackStages(A, b);

    // Configure the solver
    hsolver.setDamping(0.0);
    hsolver.setInitialActiveSet();

    // Run the solver
    hsolver.activeSearch(*partial_qdot);
    // Trim solution
    Rmath::trim(partial_qdot);
}

void R2Module::TaskManager::computeCompleteSolution(const Eigen::VectorXd& q, const Eigen::VectorXd& partial_qdot, Eigen::VectorXd* qdot)
{
    assert(partial_qdot.size() == q.size());
    assert(qdot->size() == q.size());

    updateIntermediateTasks(q, partial_qdot);

    // Initialize the HQP solver
    soth::HCOD hsolver( q.size(), taskSet.size()+1 );
#ifdef DEBUG_MODE
        INFO("Initializing HQP solver...");
        INFO( "Active tasks: " << (taskSet.size()+1) << " out of " << (taskMap.size()+1) );
        INFO("Generating stack...");

        int task_i = 2;
#endif
    // Set up the hierarchy
    std::vector<Eigen::MatrixXd> A;
    std::vector<soth::VectorBound> b;
    // Joint limits first (top priority task)
    A.push_back(jointLimits->constraintMatrix());
    b.push_back(jointLimits->vectorBounds());
    // All the remaining tasks (in descending order of priority)
    std::set<Task*>::iterator updater = taskSet.begin();
    while( updater != taskSet.end() )
    {
        // Retrieve current kinchain joints information
        std::map<std::string, int> jointsIndicesMap;
        (*updater)->getJointsIDs(&jointsIndicesMap);
        // Current task constraint matrix (restricted to the joint involved)
        Eigen::MatrixXd currentA_task = (*updater)->constraintMatrix();

        // Rewrite the task constraint matrix with the complete joint configuration
        Eigen::MatrixXd currentA = Eigen::MatrixXd::Zero( currentA_task.rows(), JOINTS_NUM );
        // The task constraint matrix is the only non-zero block in A
        for (unsigned int i = 0; i <module_ptr->jointID.size(); ++i)
        {
            if (jointsIndicesMap.find(module_ptr->jointID.at(i)) != jointsIndicesMap.end())
                currentA.col(i) = currentA_task.col( jointsIndicesMap[module_ptr->jointID.at(i)] );
            else
                currentA.col(i) = Eigen::VectorXd::Zero(currentA_task.rows());
        }

        A.push_back(currentA);
        b.push_back((*updater)->vectorBounds());
        ++updater;

#ifdef DEBUG_MODE
            INFO("Task n. " << task_i << ", constraint matrix (enlarged):\n" << currentA);
            ++task_i;
#endif
    }

    hsolver.pushBackStages(A, b);

#ifdef DEBUG_MODE
        INFO("Configuring HQP solver...");
        INFO("Start active search... ");
#endif

    // Configure the solver
    hsolver.setDamping(0.0);
    hsolver.setInitialActiveSet();

    // Run the solver
    hsolver.activeSearch(*qdot);
    // Trim solution
    Rmath::trim(qdot);

#ifdef DEBUG_MODE
        INFO("Done.");
        INFO("Active set: ");
        hsolver.showActiveSet(std::cerr);
        INFO("Solution: [\n" << *qdot << "]");
#endif

}


R2Module::TaskManager::~TaskManager()
{
    // Deallocate memory
    if (jointLimits) delete jointLimits;

    // Cleaning up containers
    std::map<std::string, Task*>::iterator destroyer = taskMap.begin();
    while (destroyer != taskMap.end())
    {
        if(destroyer->second) delete destroyer->second;
        ++destroyer;
    }
    taskMap.clear();
    taskSet.clear();
}

void R2Module::TaskManager::setJointLimits(const Eigen::MatrixXd& velBounds)
{
    // Dimensions check
    assert(velBounds.rows() == JOINTS_NUM);
    assert(velBounds.cols() == 2);

    // Replace current TaskBase*
    if (jointLimits) delete jointLimits;
    jointLimits = new TaskBase(0, Eigen::MatrixXd::Identity( JOINTS_NUM, JOINTS_NUM ), velBounds);
}

void R2Module::TaskManager::exec(const Eigen::VectorXd& q, Eigen::VectorXd* qdot)
{
    // Dimension check
    assert(qdot->size() == JOINTS_NUM);

#ifdef TASK_MANAGER_DEBUG
    INFO("-----------------------");
    std::map<std::string,Task*>::const_iterator watcher = taskMap.begin();
    while(watcher != taskMap.end())
    {
//        if ((watcher->first == "Rbound") || (watcher->first == RIGHT_ARM) )
        {
            INFO(watcher->first << ": ");
            INFO("Status = " << (watcher->second)->taskStatus());
            INFO("Priority = " << (watcher->second)->getPriority());
            INFO("Activation value = " << (watcher->second)->activationValue());
            if ((watcher->second)->done())
            {
                INFO("Task done.");
            }
            else
            {
                INFO("Task in progress...");
            }
        }
        ++watcher;
    }
#endif

    // Handle tasks waiting for a priority change
    std::map<std::string,int>::iterator priorityHandler = changingTaskMap.begin();
    while(priorityHandler != changingTaskMap.end())
    {
        Task* currentTask = taskMap[priorityHandler->first];

        // The task finally stopped and has to be re-activated with the new priority value
        if ( (currentTask->getPriority() != priorityHandler->second) && (currentTask->taskStatus() == inactive) )
        {
            // Set the task priority to the new value
            currentTask->setPriority( priorityHandler->second );
            // Insert the task back
            currentTask->activate();
            // Remove the task from the queue
            changingTaskMap.erase(priorityHandler);
        }
        // The task is still active with the old priority
        else if ( (currentTask->getPriority() != priorityHandler->second) && (currentTask->taskStatus() != active2inactive) )
            currentTask->stop();

        ++priorityHandler;
    }

    // Update joint limits task
    Eigen::MatrixXd velBounds(q.size(), 2);
    module_ptr->posBound2velBound(module_ptr->jointBounds, q, &velBounds);
    jointLimits->setVectorBounds(velBounds);

#ifdef DEBUG_MODE
    INFO("Updating joint bounds: ");
    INFO(std::endl << *jointLimits);
#endif

    taskSet.clear();
    // Compute the partial solution first
    Eigen::VectorXd partialSolution(q.size());
    computePartialSolution(q, &partialSolution);

    // Compute the final solution
    computeCompleteSolution(q, partialSolution, qdot);

#ifdef LOG
    Eigen::VectorXd qdot_LOG(JOINTS_NUM);
    qdot_LOG = *qdot;
    std::map<std::string,Task*>::const_iterator logger = taskMap.begin();
    while(logger != taskMap.end())
    {
        if((logger->first == LEFT_ARM))
        {
            INFO("-------- task name: "<< logger->first <<" --------");

            INFO("task status: " << (logger->second)->taskStatus() );
            INFO("priority: " << (logger->second)->getPriority());
            INFO("activation function: " << (logger->second)->activationValue());

            Eigen::Vector3d ee_position_LOG;
            Eigen::Matrix4d h_LOG;
            ((logger->second)->kinChain())->forward(&h_LOG);
            ee_position_LOG = h_LOG.topRightCorner(3,1);

            Eigen::Vector3d velocity;
            Eigen::Vector3d velocityError_LOG;
            Eigen::Vector3d velocityTarget_LOG;
            velocityTarget_LOG = (logger->second)->getTargetVelocity();
            Eigen::MatrixXd j_LOG;
            ((logger->second)->kinChain())->differential(&j_LOG);

            Eigen::VectorXd qdot_LA (LLEG_CHAIN_SIZE+LARM_CHAIN_SIZE);
            qdot_LA << qdot_LOG.segment<LLEG_CHAIN_SIZE>(LLEG_CHAIN_BEGIN),
                    qdot_LOG.segment<LARM_CHAIN_SIZE>(LARM_CHAIN_BEGIN);
            velocity = (j_LOG*qdot_LA).head(3);
            velocityError_LOG = velocityTarget_LOG - velocity;

            INFO("end effector position [" << ee_position_LOG(0) <<", "<<ee_position_LOG(1)<<", "<<ee_position_LOG(2)<<"]");
            INFO("velocity [" << velocity(0) <<", "<<velocity(1)<<", "<<velocity(2)<<"]");
            INFO("desired velocity [" << velocityTarget_LOG(0) <<", "<<velocityTarget_LOG(1)<<", "<<velocityTarget_LOG(2)<<"]");
            INFO("velocity error [" << velocityError_LOG(0) <<", "<<velocityError_LOG(1) <<", "<<velocityError_LOG(2)<<"]");
            INFO("velocity error norm: "<< velocityError_LOG.norm());
        }
        if( (logger->first == RIGHT_ARM) || (logger->first == "Rbound") )
        {
            INFO("-------- task name: "<< logger->first <<" --------");

            INFO("task status: " << (logger->second)->taskStatus() );
            INFO("priority: " << (logger->second)->getPriority());
            INFO("activation function: " << (logger->second)->activationValue());

            Eigen::Vector3d ee_position_LOG;
            Eigen::Matrix4d h_LOG;
            ((logger->second)->kinChain())->forward(&h_LOG);
            ee_position_LOG = h_LOG.topRightCorner(3,1);
            Eigen::Vector3d velocity;
            Eigen::Vector3d velocityError_LOG;
            Eigen::Vector3d velocityTarget_LOG;
            velocityTarget_LOG = (logger->second)->getTargetVelocity();
            Eigen::MatrixXd j_LOG;
            ((logger->second)->kinChain())->differential(&j_LOG);
            Eigen::VectorXd qdot_RA (LLEG_CHAIN_SIZE+RARM_CHAIN_SIZE);
            qdot_RA << qdot_LOG.segment<LLEG_CHAIN_SIZE>(LLEG_CHAIN_BEGIN),
                    qdot_LOG.segment<RARM_CHAIN_SIZE>(RARM_CHAIN_BEGIN);
            velocity = (j_LOG*qdot_RA).head(3);
            velocityError_LOG = velocityTarget_LOG - velocity;

            INFO("end effector position [" << ee_position_LOG(0) <<", "<<ee_position_LOG(1)<<", "<<ee_position_LOG(2)<<"]");
            INFO("velocity [" << velocity(0) <<", "<<velocity(1)<<", "<<velocity(2)<<"]");
            INFO("desired velocity [" << velocityTarget_LOG(0) <<", "<<velocityTarget_LOG(1)<<", "<<velocityTarget_LOG(2)<<"]");
            INFO("velocity error [" << velocityError_LOG(0) <<", "<<velocityError_LOG(1) <<", "<<velocityError_LOG(2)<<"]");
            INFO("velocity error norm: "<< velocityError_LOG.norm());
        }

        ++logger;
    }
#endif
}
