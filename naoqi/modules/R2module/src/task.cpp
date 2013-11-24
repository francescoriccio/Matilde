/**
* @class: Task
* This class (and related hierarchy) defines a generic task in the HQP framework.
*
* @file transform.h
* @author Claudio Delli Bovi, Chiara Picardi, Francesco Riccio
*/

#include "task.h"

#define INFO(x) std::cerr << "\033[22;34;1m" << "[Task] " << x << "\033[0m" << std::endl;
//#define DEBUG_MODE

#define POSITION_TASK_DIM 3


/** ---------- TaskBase ---------- */


/*
 * Class constructor: given a general task in the form A*x=b and the correspondent priority, directly initializes
 * the class private members using their copy constructors.
 */
TaskBase::TaskBase(int _priority, const Eigen::MatrixXd& A, const soth::VectorBound& b):
    priority(_priority), active(false)
{
    // Matrices dimensions have to be consistent
    assert( A.rows() == b.rows() );

    // Initialize private members
    constraint_matrix = A;
    bounds = b;

#ifdef DEBUG_MODE
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
TaskBase::TaskBase(int _priority, const Eigen::MatrixXd& A, const Eigen::MatrixXd& b): priority(_priority)
{
    // Matrices dimensions have to be consistent
    assert( A.rows() == b.rows() );

    // Initialize constraint matrix
    constraint_matrix = A;
    // Initialize bound vector
    bounds.setZero(b.rows(),1);
    for(int i=0; i<b.rows(); ++i)
        bounds[i] = soth::Bound(b(i,0), b(i,1));

#ifdef DEBUG_MODE
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
TaskBase::TaskBase(int _priority, const Eigen::MatrixXd& A, const Eigen::VectorXd& b, soth::Bound::bound_t bt): priority(_priority)
{
    // Matrices dimensions have to be consistent
    assert( A.rows() == b.rows() );

    // Initialize constraint matrix
    constraint_matrix = A;
    // Initialize bound vector
    bounds.setZero(b.rows(),1);
    for(int i=0; i<b.rows(); ++i)
        bounds[i] = soth::Bound(b(i), bt);

#ifdef DEBUG_MODE
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
    priority = tb.priority;

#ifdef DEBUG_MODE
    INFO("Task base class initalized.");
    INFO("Constraint matrix:");
    INFO(constraint_matrix);
    INFO("Bound vector:");
    INFO(bounds);
#endif
}

/*
 * Setter that replaces the bound vector with another bound vector (given as a general Eigen::MatrixXd).
 */
void TaskBase::setVectorBounds( const Eigen::MatrixXd& b )
{
    // Dimensions must be consistent
    assert( bounds.rows() == b.rows() );
    assert( b.cols() == 2 );

    // Replacing the old bound vector with the new one
    bounds.setZero(b.rows(),1);
    for(int i=0; i<b.rows(); ++i)
        bounds[i] = soth::Bound(b(i,0), b(i,1));

#ifdef DEBUG_MODE
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

#ifdef DEBUG_MODE
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

#ifdef DEBUG_MODE
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


/** ---------- TaskBase ---------- */


/*
 * Class constructor. Input arguments are:
 * - The task dimension m,n where A*x=b being 'A' m x n and 'b' m x 1;
 * - The task priority value (low value => higher priority and high value => lower priority);
 * - A ConfigReader class object to initialize the task kinematic chain;
 * - Two indices _base, _ee identifying the task kinematic chain in the ConfigReader object file.
 */
Task::Task(int m, int n, int _priority, ConfigReader theConfigReader, int _base, int _ee):
    // Base class initalization
    TaskBase(_priority, Eigen::MatrixXd::Identity(m, n), Eigen::VectorXd::Zero(m), soth::Bound::BOUND_TWIN ),
    // Kinematic chain initialization
    theKinChain( new Rmath::KinChain( theConfigReader, _base, _ee) )
{
#ifdef DEBUG_MODE
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

/*
 * Task update function: update the constraint matrix A=A(q) with a new value of q and replace the target vector
 * with a proportional/derivative control law of the type K*POSE_ERROR + DESIRED_VELOCITY.
 */
void Task::update(const Eigen::VectorXd& q, double K, const Eigen::VectorXd& desiredPose, const Eigen::VectorXd& desiredVel)
{
    // Dimensions must be consistent
    assert(constraint_matrix.rows() == desiredPose.size());
    assert(constraint_matrix.rows() == desiredVel.size());
    assert(constraint_matrix.cols() == q.size());

#ifdef DEBUG_MODE
    INFO("Updating task...");
#endif

    // Updating the task kinematic chain with the new joint values
    theKinChain->update(q);

    // Re-computing direct kinematics
    Eigen::Matrix4d H_chain;
    theKinChain->forward((&H_chain));
    // Re-computing differential kinematics
    Eigen::MatrixXd J_chain(constraint_matrix.rows(), constraint_matrix.cols());
    theKinChain->differential(&J_chain,constraint_matrix.rows());

    // Replacing the task constraint matrix with the updated version
    constraint_matrix << J_chain;

#ifdef DEBUG_MODE
    INFO("New constraint matrix:");
    INFO(constraint_matrix);
#endif

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

    // Updating the bound vector with the task error + a feedforward term
    for(int i=0; i<bounds.rows(); ++i)
        bounds[i] = K * (desiredPose(i)-currentPose(i)) + desiredVel(i);

#ifdef DEBUG_MODE
    INFO("New bound vector:");
    INFO(bounds);
#endif
}
