/**
* @class: Task
* This class (and related hierarchy) defines a generic task in the HQP framework.
*
* @file transform.h
* @author Claudio Delli Bovi, Chiara Picardi, Francesco Riccio
*/

#include "task.h"

#define INFO(x) std::cerr << "\033[22;34;1m" << "[Task] " << x << "\033[0m" << std::endl;
//#define TASK_DEBUG

#define POSITION_TASK_DIM 3

#define POSE_ERROR_TOLERANCE 50 /* [mm] */


/** ---------- TaskBase ---------- */


/*
 * Class constructor: given a general task in the form A*x=b and the correspondent priority, directly initializes
 * the class private members using their copy constructors.
 */
TaskBase::TaskBase(int _priority, const Eigen::MatrixXd& A, const soth::VectorBound& b):
    boundType(b[0].getType()), priority(_priority)
{
    // Matrices dimensions have to be consistent
    assert( A.rows() == b.rows() );

    // Initialize private members
    constraint_matrix = A;
    bounds = b;

#ifdef TASK_DEBUG
    INFO("Task base class initalized.");
    INFO("Constraint matrix:");
    INFO(constraint_matrix);
    INFO("Bound vector:");
    INFO(bounds);
#endif
}

/*
 * Overloaded version of the constructor. Takes as input the bound vector as a general Eigen::MatrixXd.
 */
TaskBase::TaskBase(int _priority, const Eigen::MatrixXd& A, const Eigen::MatrixXd& b):
    boundType(soth::Bound::BOUND_DOUBLE), priority(_priority)
{
    // Matrices dimensions have to be consistent
    assert( A.rows() == b.rows() );

    // Initialize constraint matrix
    constraint_matrix = A;
    // Initialize bound vector
    bounds.setZero(b.rows(),1);
    for(int i=0; i<b.rows(); ++i)
        bounds[i] = soth::Bound(b(i,0), b(i,1));

#ifdef TASK_DEBUG
    INFO("Task base class initalized.");
    INFO("Constraint matrix:");
    INFO(constraint_matrix);
    INFO("Bound vector:");
    INFO(bounds);
#endif
}

/*
 * Overloaded version of the constructor. Takes as input the bound vector as a Eigen::VectorXd and the bound type.
 */
TaskBase::TaskBase(int _priority, const Eigen::MatrixXd& A, const Eigen::VectorXd& b, soth::Bound::bound_t bt):
    boundType(bt), priority(_priority)
{
    // Matrices dimensions have to be consistent
    assert( A.rows() == b.rows() );

    // Initialize constraint matrix
    constraint_matrix = A;
    // Initialize bound vector
    bounds.setZero(b.rows(),1);
    for(int i=0; i<b.rows(); ++i)
        bounds[i] = soth::Bound(b(i), bt);

#ifdef TASK_DEBUG
    INFO("Task base class initalized.");
    INFO("Constraint matrix:");
    INFO(constraint_matrix);
    INFO("Bound vector:");
    INFO(bounds);
#endif
}

/*
 * Class copy constructor.
 */
TaskBase::TaskBase(const TaskBase& tb)
{
    constraint_matrix = tb.constraint_matrix;
    bounds = tb.bounds;
    boundType = tb.boundType;
    priority = tb.priority;

#ifdef TASK_DEBUG
    INFO("Task base class initalized.");
    INFO("Constraint matrix:");
    INFO(constraint_matrix);
    INFO("Bound vector:");
    INFO(bounds);
#endif
}

/*
 * Setter that replaces the bound vector with another bound vector (given as a vector + bound type).
 */
void TaskBase::setVectorBounds(const Eigen::VectorXd& b, soth::Bound::bound_t type)
{
    // Dimensions must be consistent
    assert( bounds.rows() == b.size() );

    // Replacing the old bound vector with the new one
    bounds.setZero(b.size(), 1);
    boundType = type;
    for(int i = 0; i < b.size(); ++i)
        bounds[i] = soth::Bound(b(i), type);

#ifdef TASK_DEBUG
    INFO("New bound vector:");
    INFO(bounds);
#endif
}

/*
 * Setter that replaces the bound vector with another bound vector (given as a general Eigen::MatrixXd).
 * NOTE: in this case, the new bound is assumed to be of type soth::Bound::BOUND_DOUBLE.
 */
void TaskBase::setVectorBounds( const Eigen::MatrixXd& b )
{
    // Dimensions must be consistent
    assert( bounds.rows() == b.rows() );
    assert( b.cols() == 2 );

    // Replacing the old bounds with the new ones
    bounds.setZero(b.rows(),1);
    boundType = soth::Bound::BOUND_DOUBLE;
    for(int i = 0; i < b.rows(); ++i)
        bounds[i] = soth::Bound(b(i,0), b(i,1));

#ifdef TASK_DEBUG
    INFO("New bound vector:");
    INFO(bounds);
#endif
}

/*
 * Setter that replaces the bound vector, directly taking as input a soth::VectorBound.
 */
void TaskBase::setVectorBounds( const soth::VectorBound& b )
{
    // Dimensions must be consistent
    assert( bounds.rows() == b.rows() );

    bounds = b;
    boundType = b[0].getType();

#ifdef TASK_DEBUG
    INFO("New bound vector:");
    INFO(bounds);
#endif
}

/*
 * Setter that replaces the constraint matrix.
 */
void TaskBase::setConstraintMatrix( const Eigen::MatrixXd& A )
{
    // Dimensions must be consistent
    assert( A.rows() == constraint_matrix.rows() );

    constraint_matrix = A;

#ifdef TASK_DEBUG
    INFO("New constraint matrix:");
    INFO(constraint_matrix);
#endif
}

/*
 * Less-than operator overload: tasks are ordered by their priority value.
 */
bool operator<(const TaskBase& l_tb, const TaskBase& r_tb)
{
    if(l_tb.priority <= r_tb.priority) return true;
    else return false;
}

/*
 * Stream operator overload: print the task on the output stream, in the form A*x=b.
 */
std::ostream& operator<<(std::ostream& out, const TaskBase& tb)
{
    // Devise the task type (equality, inequality, 'double' inequality)
    std::string op = "";
    if(tb.bounds[0].getType() == soth::Bound::BOUND_INF) op = "<";
    else if(tb.bounds[0].getType() == soth::Bound::BOUND_SUP) op = ">";
    else if(tb.bounds[0].getType() == soth::Bound::BOUND_TWIN) op = "=";
    else if(tb.bounds[0].getType() == soth::Bound::BOUND_DOUBLE) op = "in";

    // Print the task equation row-by-row
    for(unsigned int i = 0; i < tb.bounds.rows(); ++i)
    {
        if(i == tb.bounds.rows()/2 && op == "in")
            out<<"  | "<<tb.constraint_matrix.row(i)<<" |  "<< op <<"  "<< tb.bounds[i]<<std::endl;
        else if(i == tb.bounds.rows()/2)
            out<<"  | "<<tb.constraint_matrix.row(i)<<" |  "<< op <<"   "<< tb.bounds[i]<<std::endl;
        else
            out<<"  | "<<tb.constraint_matrix.row(i)<<" |      "<< tb.bounds[i]<<std::endl;
    }
    return out;
}

/*
 * Pre/postfix increment/decrement operators overload: the task priority is modified accordingly.
 */
TaskBase& TaskBase::operator++()
{
    ++priority;
    return *this;
}

TaskBase& TaskBase::operator--()
{
    --priority;
    return *this;
}

TaskBase TaskBase::operator++(int)
{
    TaskBase tmp_tb(*this);
    ++(*this);
    return tmp_tb;
}

TaskBase TaskBase::operator--(int)
{
    TaskBase tmp_tb(*this);
    --(*this);
    return tmp_tb;
}

/** ---------- Task ---------- */

/*
 * Class constructor. Input arguments are:
 * - The task dimension m,n where A*x=b being 'A' m x n and 'b' m x 1;
 * - The task priority value (low value => higher priority and high value => lower priority);
 * - A ConfigReader class object to initialize the task kinematic chain;
 * - Two indices _base, _ee identifying the task kinematic chain in the ConfigReader object file.
 */
Task::Task(int m, int n, int _priority, ConfigReader theConfigReader, soth::Bound::bound_t boundType):
    // Base class initalization
    TaskBase(_priority, Eigen::MatrixXd::Identity(m, n), Eigen::VectorXd::Zero(m), boundType),
    // Kinematic chain initialization
    theKinChain( new Rmath::KinChain(theConfigReader) ), base_end(0),
    parameters( inactive, 0, ACTIVATION_STEP, Eigen::VectorXd::Zero(m),
        Eigen::VectorXd::Zero(n), Eigen::Matrix4d::Identity())
{
#ifdef TASK_DEBUG
    INFO("Task kinematic chain:");
    INFO(theKinChain);
#endif
}

Task::Task(int m, int n,  int _priority, const Rmath::KinChain& _kc, int _base, soth::Bound::bound_t boundType):
    // Base class initalization
    TaskBase(_priority, Eigen::MatrixXd::Identity(m, n), Eigen::VectorXd::Zero(m), boundType),
    // Kinematic chain initialization
    theKinChain( new Rmath::KinChain(_kc) ), base_end(_base),
    parameters( inactive, 0, ACTIVATION_STEP, Eigen::VectorXd::Zero(m),
        Eigen::VectorXd::Zero(n), Eigen::Matrix4d::Identity())
{
#ifdef TASK_DEBUG
    INFO("Task kinematic chain:");
    INFO(theKinChain);
#endif
}

/*
 * Class destructor: deallocates the kinematic chain from the heap.
 */
Task::~Task()
{
    if(theKinChain) delete theKinChain;
}

void Task::activate(float activationStep)
{
    if (parameters.taskStatus == inactive)
    {
        // Recover information about the target
        if (parameters.positioningActive && !parameters.path.empty())
            // Rebuild the path from scratch
            setDesiredPose( parameters.path.at(parameters.path.size()-1), parameters.path.size());
    }

    if(parameters.taskStatus != active && parameters.taskStatus != inactive2active )
    {
        parameters.activationStep = activationStep;
        if(parameters.activationValue == 1.0)
            parameters.taskStatus = active;
        else
            parameters.taskStatus = inactive2active;
    }
}

void Task::stop(float decayStep)
{
    if(parameters.taskStatus != inactive && parameters.taskStatus != active2inactive )
    {
        parameters.activationStep = decayStep;
        if(parameters.activationValue == 0.0)
            parameters.taskStatus = inactive;
        else
            parameters.taskStatus = active2inactive;
    }
}


Eigen::VectorXd Task::getCurrentPose() const
{
    Eigen::Matrix4d H_chain;
    theKinChain->forward((&H_chain));
    H_chain = parameters.baseT * H_chain;
    Eigen::VectorXd currentPose (constraint_matrix.rows());
    if(constraint_matrix.rows() > POSITION_TASK_DIM)
        // Retrieving the position from the translation vector of the forward kinematics
        // Retrieving the orientation from the Euler fixed frame x-y-z angles
        currentPose << H_chain.topRightCorner(POSITION_TASK_DIM,1),
                       Rmath::xyzEulerAngles( H_chain.topLeftCorner(3,3) ).head(constraint_matrix.rows()-POSITION_TASK_DIM);
    // If the task target is a position vector (A row size <= 3) just the translation vector of the forward kinematics is needed
    else
        currentPose << H_chain.col(POSITION_TASK_DIM).head(constraint_matrix.rows());

    return currentPose;
}

/* TOCOMMENT */
const Eigen::VectorXd Task::getTargetPose() const
{
    if(parameters.positioningActive)
        return parameters.path.at(parameters.path_currentStep);
    else
        return getCurrentPose();
}


void Task::setDesiredPose(const Eigen::VectorXd& dp, int n_controlPoints)
{
    // Re-computing direct kinematics
    Eigen::Matrix4d H_chain;
    theKinChain->forward((&H_chain));
    H_chain = parameters.baseT * H_chain;

    Eigen::VectorXd initialPose (constraint_matrix.rows());
    if(constraint_matrix.rows() > POSITION_TASK_DIM)
        // Retrieving the position from the translation vector of the forward kinematics
        // Retrieving the orientation from the Euler fixed frame x-y-z angles
        initialPose << H_chain.topRightCorner(POSITION_TASK_DIM,1),
                       Rmath::xyzEulerAngles( H_chain.topLeftCorner(3,3) ).head(constraint_matrix.rows()-POSITION_TASK_DIM);
    // If the task target is a position vector (A row size <= 3) just the translation vector of the forward kinematics is needed
    else
        initialPose << H_chain.col(POSITION_TASK_DIM).head(constraint_matrix.rows());

    assert(dp.size() == initialPose.size());

    parameters.path.clear();
    for (float i = 1.0; i <= n_controlPoints; ++i)
        parameters.path.push_back(initialPose + (i/static_cast<float>(n_controlPoints)) * (dp - initialPose));

    parameters.path_currentStep = 0;
    parameters.positioningActive = true;
    if (parameters.jointControlActive) parameters.jointControlActive = false;
}

void Task::setDesiredPose(const Eigen::VectorXd& idp, const Eigen::VectorXd& dp, int n_controlPoints)
{
    // Re-computing direct kinematics
    Eigen::Matrix4d H_chain;
    theKinChain->forward((&H_chain));
    H_chain = parameters.baseT * H_chain;

    Eigen::VectorXd initialPose (constraint_matrix.rows());
    if(constraint_matrix.rows() > POSITION_TASK_DIM)
        // Retrieving the position from the translation vector of the forward kinematics
        // Retrieving the orientation from the Euler fixed frame x-y-z angles
        initialPose << H_chain.topRightCorner(POSITION_TASK_DIM,1),
                       Rmath::xyzEulerAngles( H_chain.topLeftCorner(3,3) ).head(constraint_matrix.rows()-POSITION_TASK_DIM);
    // If the task target is a position vector (A row size <= 3) just the translation vector of the forward kinematics is needed
    else
        initialPose << H_chain.col(POSITION_TASK_DIM).head(constraint_matrix.rows());

    assert(idp.size() == initialPose.size());
    assert(dp.size() == initialPose.size());

    parameters.path.clear();
    for (float i = 1.0; i <= n_controlPoints; ++i)
        parameters.path.push_back(initialPose + (i/static_cast<float>(n_controlPoints)) * (idp - initialPose));

    for (float i = 1.0; i <= n_controlPoints; ++i)
       parameters.path.push_back(idp + (i/static_cast<float>(n_controlPoints)) * (dp - idp));

    parameters.path_currentStep = 0;
    parameters.positioningActive = true;
    if (parameters.jointControlActive) parameters.jointControlActive = false;
}

void Task::setDesiredConfiguration(const Eigen::VectorXd& desiredConf, int n_controlPoints)
{

    Eigen::VectorXd initialConf = theKinChain->jointConfiguration();
    assert(initialConf.size() == desiredConf.size());

    constraint_matrix = Eigen::MatrixXd::Identity(initialConf.size(), initialConf.size());
    bounds.setZero(initialConf.size(),1);

    parameters.path.clear();
    for (float i = 1.0; i <= n_controlPoints; ++i)
        parameters.path.push_back(initialConf + (i/static_cast<float>(n_controlPoints)) * (desiredConf - initialConf));

    parameters.path_currentStep = 0;
    parameters.positioningActive = true;
    if (!parameters.jointControlActive) parameters.jointControlActive = true;
}

void Task::circularPathGenerator( const Eigen::VectorXd& dp, float z_shift, int n_controlPoints, float radius, int n )
{

    // Re-computing direct kinematics
    Eigen::Matrix4d H_chain;
    theKinChain->forward((&H_chain));
    H_chain = parameters.baseT * H_chain;

    parameters.path.clear();
    for (float i = 0.0; i < n*2*M_PI; i+= 2*M_PI/n_controlPoints)
    {
        Eigen::VectorXd ee_desiredPose_handframe(4);
        ee_desiredPose_handframe << radius*cos(i), radius*sin(i), z_shift, 1.0;
        Rmath::trim(&ee_desiredPose_handframe);

        Eigen::VectorXd ee_desiredPose_CoMframe(dp.size());
        if(dp.size() > 3)
            ee_desiredPose_CoMframe <<  (H_chain * ee_desiredPose_handframe).head(3), dp(3), dp(4), dp(5);
        else
            ee_desiredPose_CoMframe <<  (H_chain * ee_desiredPose_handframe).head(3);

        Rmath::trim(&ee_desiredPose_CoMframe);

        parameters.path.push_back( ee_desiredPose_CoMframe );
    }

    parameters.path_currentStep = 0;
    parameters.positioningActive = true;
    if (parameters.jointControlActive) parameters.jointControlActive = false;
}

/*
 * Task update function: update the constraint matrix A=A(q) with a new value of q and replace the target vector
 * with a proportional/derivative control law of the type K*POSE_ERROR + DESIRED_VELOCITY.
 */
void Task::update( const Eigen::VectorXd& _q, const Eigen::VectorXd& desiredVel, double K )
{
    // Dimensions must be consistent
    assert(constraint_matrix.rows() == desiredVel.size());
    assert(constraint_matrix.cols() == _q.size());

    Eigen::VectorXd q(_q.size());
    q << -_q.head(base_end).reverse(), _q.tail(_q.size()-base_end);

    // Updating the task kinematic chain with the new joint values
    theKinChain->update(q);

#ifdef TASK_DEBUG
        INFO("Updating task...");
        INFO("Current joint configuration: \n" << q);
        INFO("Kinematic chain: \n" << (*theKinChain));
#endif

    // Cartesian space task
    if (!parameters.jointControlActive)
    {

        // Replacing the task constraint matrix with the updated version
        if (constraint_matrix.rows() > POSITION_TASK_DIM)
        {
            // Re-computing differential kinematics
            Eigen::MatrixXd J_chain(constraint_matrix.rows(), constraint_matrix.cols());
            theKinChain->differential(&J_chain);

            // Computing base transform
            Eigen::MatrixXd baseT = Eigen::MatrixXd::Zero(6,6);
            baseT.topLeftCorner(3,3) = parameters.baseT.topLeftCorner(3,3);
            baseT.bottomRightCorner(3,3) = parameters.baseT.topLeftCorner(3,3);
            // Pre-multipling base transform
            constraint_matrix << (baseT * J_chain).topRows(constraint_matrix.rows());
        }
        else
        {
            // Re-computing differential kinematics
            Eigen::MatrixXd J_chain(constraint_matrix.rows(), constraint_matrix.cols());
            theKinChain->differential(&J_chain, POSITION_TASK_DIM);

            // Pre-multipling base transform
            constraint_matrix << (parameters.baseT.topLeftCorner(3,3) * J_chain).topRows(constraint_matrix.rows());
        }

#ifdef TASK_DEBUG
        INFO("New constraint matrix:");
        INFO(std::endl << constraint_matrix);
#endif
        Eigen::VectorXd transitionVelocity(constraint_matrix.rows());
        transitionVelocity = constraint_matrix * parameters.qd_n;

        // Equality task with position control in the Cartesian space
        if ( (parameters.positioningActive) && (bounds[0].getType() == soth::Bound::BOUND_TWIN) )
        {
            // Re-computing direct kinematics
            Eigen::Matrix4d H_chain;
            theKinChain->forward((&H_chain));
            H_chain = parameters.baseT * H_chain;

            // Computing the current pose in the task space
            Eigen::VectorXd currentPose(constraint_matrix.rows());
            // If the task target is a pose (position+orientation) vector, a minimal description of the orientation has to be computed
            if(constraint_matrix.rows() > POSITION_TASK_DIM)
                // Retrieving the position from the translation vector of the forward kinematics
                // Retrieving the orientation from the Euler fixed frame x-y-z angles
                currentPose << H_chain.topRightCorner(POSITION_TASK_DIM,1),
                               Rmath::xyzEulerAngles( H_chain.topLeftCorner(3,3) ).head(constraint_matrix.rows()-POSITION_TASK_DIM);
            // If the task target is a position vector (A row size <= 3) just the translation vector of the forward kinematics is needed
            else
                currentPose << H_chain.col(POSITION_TASK_DIM).head(constraint_matrix.rows());

            Eigen::VectorXd pose_error;
            if(parameters.path_currentStep < parameters.path.size())
                 pose_error = parameters.path.at(parameters.path_currentStep) - currentPose;
            else
                pose_error = parameters.path.at(parameters.path.size()-1) - currentPose;

            parameters.targetVelocity = K * pose_error + desiredVel;

            // Updating the bound vector with the task error + a feedforward term
            for(int i=0; i<bounds.rows(); ++i)
            {
                bounds[i] = parameters.targetVelocity(i) * parameters.activationValue +
                        (1- parameters.activationValue)* transitionVelocity(i);
            }

            if ( (pose_error.norm() < POSE_ERROR_TOLERANCE) && (parameters.path_currentStep < parameters.path.size()-1) )
                ++parameters.path_currentStep;

        }
        // Equality task with velocity control in the Cartesian space
        else if ( bounds[0].getType() == soth::Bound::BOUND_TWIN )
        {
            // Updating the bound vector with the task error + a feedforward term
            for(int i=0; i<bounds.rows(); ++i)
                bounds[i] = parameters.targetVelocity(i) * parameters.activationValue +
                        (1- parameters.activationValue)* transitionVelocity(i);
        }
        // Inequality task in the Cartesian space
        else
        {
            // Re-computing direct kinematics
            Eigen::Matrix4d H_chain;
            theKinChain->forward((&H_chain));
            H_chain = parameters.baseT * H_chain;

            // Computing the current pose in the task space
            Eigen::VectorXd currentPose(constraint_matrix.rows());
            // If the task target is a pose (position+orientation) vector, a minimal description of the orientation has to be computed
            if(constraint_matrix.rows() > POSITION_TASK_DIM)
                // Retrieving the position from the translation vector of the forward kinematics
                // Retrieving the orientation from the Euler fixed frame x-y-z angles
                currentPose << H_chain.topRightCorner(POSITION_TASK_DIM,1),
                               Rmath::xyzEulerAngles( H_chain.topLeftCorner(3,3) ).head(constraint_matrix.rows()-POSITION_TASK_DIM);
            // If the task target is a position vector (A row size <= 3) just the translation vector of the forward kinematics is needed
            else
                currentPose << H_chain.col(POSITION_TASK_DIM).head(constraint_matrix.rows());

            Eigen::VectorXd new_b (bounds.rows());
            new_b = ( parameters.path.at(parameters.path.size()-1) - currentPose ) / TIME_STEP;
            for(int i=0; i<bounds.rows(); ++i)
                bounds[i] = soth::Bound(new_b(i) * parameters.activationValue + (1- parameters.activationValue)* transitionVelocity(i),
                                        boundType);
        }
    }
    // Joint space task
    else
    {
        // Equality task with position control in the joint space
        if ( (parameters.positioningActive) && (bounds[0].getType() == soth::Bound::BOUND_TWIN) )
        {
            Eigen::VectorXd joint_error;
            if(parameters.path_currentStep < parameters.path.size())
                 joint_error = parameters.path.at(parameters.path_currentStep) - theKinChain->jointConfiguration();
            else
                joint_error = parameters.path.at(parameters.path.size()-1) - theKinChain->jointConfiguration();

            parameters.targetVelocity = K * joint_error + desiredVel;
            // Updating the bound vector with the task error + a feedforward term
            for(int i=0; i<bounds.rows(); ++i)
            {
                bounds[i] = parameters.targetVelocity(i) * parameters.activationValue +
                        (1- parameters.activationValue)* parameters.qd_n(i);
            }

            if ( (joint_error.norm() < POSE_ERROR_TOLERANCE) && (parameters.path_currentStep < parameters.path.size()-1) )
                ++parameters.path_currentStep;
        }
        // Equality task with velocity control in the joint space
        else if ( bounds[0].getType() == soth::Bound::BOUND_TWIN )
        {
            // Updating the bound vector with the task error + a feedforward term
            for(int i=0; i<bounds.rows(); ++i)
            {
                bounds[i] = parameters.targetVelocity(i) * parameters.activationValue +
                        (1- parameters.activationValue)* parameters.qd_n(i);
            }
        }
        // Inequality task in the joint space
        else
        {
            Eigen::VectorXd new_b (bounds.rows());
            new_b = (parameters.path.at(parameters.path.size()-1) - q) / TIME_STEP;
            for(int i=0; i<bounds.rows(); ++i)
                bounds[i] = soth::Bound (new_b(i) * parameters.activationValue + (1- parameters.activationValue)* parameters.qd_n(i),
                                         boundType);
        }
    }

    if(parameters.taskStatus == inactive2active)
        parameters.increaseActivationValue();
    else if(parameters.taskStatus == active2inactive)
        parameters.decreaseActivationValue();

#ifdef TASK_DEBUG
    INFO("New bound vector:");
    INFO(bounds);
#endif
}
