/**
* @class: Task
*
* @file task.h
* @author Claudio Delli Bovi, Chiara Picardi, Francesco Riccio
*/

#include "task.h"

/**----------TaskBase----------*/
TaskBase::TaskBase(int _priority, const Eigen::MatrixXd& A, const soth::VectorBound& b): priority(_priority)
{
    assert( A.rows() == b.rows() );

    constraint_matrix = A;
    bounds = b;
}

TaskBase::TaskBase(int _priority, const Eigen::MatrixXd& A, const Eigen::MatrixXd& b): priority(_priority)
{
    assert( A.rows() == b.rows() );

    constraint_matrix = A;
    bounds.setZero(b.rows(),1);
    for(int i=0; i<b.rows(); ++i)
        bounds[i] = soth::Bound(b(i,0), b(i,1));
}

TaskBase::TaskBase(int _priority, const Eigen::MatrixXd& A, const Eigen::VectorXd& b, soth::Bound::bound_t bt): priority(_priority)
{
    assert( A.rows() == b.rows() );

    constraint_matrix = A;

    bounds.setZero(b.rows(),1);
    for(int i=0; i<b.rows(); ++i)
        bounds[i] = soth::Bound(b(i), bt);
}

TaskBase::TaskBase(const TaskBase& tb)
{
    constraint_matrix = tb.constraint_matrix;
    bounds = tb.bounds;

    priority = tb.priority;
}

void TaskBase::setVectorBounds( const Eigen::MatrixXd& b )
{
    assert( bounds.rows() == b.rows() );
    assert( b.cols() == 2 );

    bounds.setZero(b.rows(),1);
    for(int i=0; i<b.rows(); ++i)
        bounds[i] = soth::Bound(b(i,0), b(i,1));
}

void TaskBase::setVectorBounds( const soth::VectorBound& b )
{
    assert( bounds.rows() == b.rows() );
    bounds = b;
}

void TaskBase::setConstraintMatrix( const Eigen::MatrixXd& A )
{
    assert( A.rows() == constraint_matrix.rows() );

    constraint_matrix = A;
}

bool /*TaskBase::*/operator<(const TaskBase& l_tb, const TaskBase& r_tb)
{
    if(l_tb.priority <= r_tb.priority) return true;
    else return false;
}

std::ostream& operator<<(std::ostream& out, const TaskBase& tb)
{
    std::string op = "";
    if(tb.bounds[0].getType() == soth::Bound::BOUND_INF) op = "<";
    else if(tb.bounds[0].getType() == soth::Bound::BOUND_SUP) op = ">";
    else if(tb.bounds[0].getType() == soth::Bound::BOUND_TWIN) op = "=";
    else if(tb.bounds[0].getType() == soth::Bound::BOUND_DOUBLE) op = "in";

    for(unsigned int i = 0; i < tb.bounds.rows(); ++i)
    {
        if(i == tb.bounds.rows()/2 && op == "in")
            std::cout<<"  | "<<tb.constraint_matrix.row(i)<<" |  "<< op <<"  "<< tb.bounds[i]<<std::endl;
        else if(i == tb.bounds.rows()/2)
            std::cout<<"  | "<<tb.constraint_matrix.row(i)<<" |  "<< op <<"   "<< tb.bounds[i]<<std::endl;
        else
            std::cout<<"  | "<<tb.constraint_matrix.row(i)<<" |      "<< tb.bounds[i]<<std::endl;
    }
    return out;
}

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

/**----------Task----------*/
Task::Task(std::string cfg_path_file, int n, int m, int _base, int _ee, int _priority):
    TaskBase(_priority, Eigen::MatrixXd::Identity(n, m), Eigen::VectorXd::Zero(n), soth::Bound::BOUND_TWIN ),
    base(_base), ee(_ee)
{
    ConfigReader theConfigReader( cfg_path_file.c_str(),
                                  "../config/joints_params.cfg" );

    theKinChain = new Rmath::KinChain( theConfigReader, _base, _ee);
//    (*baseKinChain) += (*theKinChain);
//    theKinChain= baseKinChain;
}

soth::VectorBound& Task::chainJointLimits()
{
    return theKinChain->jointLimits();
}

void Task::update(const Eigen::VectorXd& q, double K, const Eigen::VectorXd& desiredPose, const Eigen::VectorXd& desiredVel)
{
    assert(constraint_matrix.rows() == desiredPose.size());
    assert(constraint_matrix.rows() == desiredVel.size());
    assert(constraint_matrix.cols() == q.size());

    Eigen::VectorXd qtmp(q.size()-base);
    qtmp << q.tail(q.size()-base);

    Eigen::Matrix4d H_chain;
    Eigen::MatrixXd J_chain(constraint_matrix.rows(), ee-base);
    Eigen::VectorXd q_chain(ee-base);
    q_chain << qtmp.head(ee-base);

    theKinChain->update(q_chain);
    theKinChain->forward((&H_chain));
    theKinChain->differential(&J_chain,constraint_matrix.rows());

    if(base == 0)
        constraint_matrix << J_chain,
                             Eigen::MatrixXd::Zero(constraint_matrix.rows(), q.size()-ee);
    else
        constraint_matrix << Eigen::MatrixXd::Zero(constraint_matrix.rows(),base),
                             J_chain,
                             Eigen::MatrixXd::Zero(constraint_matrix.rows(), q.size()-ee);

    Eigen::VectorXd currentPose(constraint_matrix.rows());
    if(constraint_matrix.rows() > 3)
        currentPose << H_chain.topRightCorner(3,1),
                       Rmath::xyzEulerAngles( H_chain.topLeftCorner(3,3) ).head(constraint_matrix.rows()-3);
    else
        currentPose << H_chain.col(3).head(constraint_matrix.rows());

    for(int i=0; i<bounds.rows(); ++i)
        bounds[i] = K * (desiredPose(i)-currentPose(i)) + desiredVel(i);
}

void Task::update(const Eigen::VectorXd& q, double K, const Eigen::VectorXd& desiredPose, const Eigen::VectorXd& desiredVel,
                  const Eigen::MatrixXd& J_base, const Eigen::MatrixXd& H_base)
{
    assert(constraint_matrix.rows() == desiredPose.size());
    assert(constraint_matrix.rows() == desiredVel.size());
    assert(constraint_matrix.cols() == q.size());
    assert(J_base.rows() == constraint_matrix.rows());

    Eigen::VectorXd qtmp(q.size()-base);
    qtmp << q.tail(q.size()-base);

    Eigen::Matrix4d H_chain;
    Eigen::MatrixXd J_chain(constraint_matrix.rows(), ee-base);
    Eigen::VectorXd q_chain(ee-base);
    q_chain << qtmp.head(ee-base);

    theKinChain->update(q_chain);
    theKinChain->forward((&H_chain));
    theKinChain->differential(&J_chain, constraint_matrix.rows());

    if(base == 0)
        constraint_matrix << J_chain,
                             Eigen::MatrixXd::Zero(constraint_matrix.rows(), q.size()-(ee+J_base.cols())),
                             J_base;
    else
        constraint_matrix << Eigen::MatrixXd::Zero(constraint_matrix.rows(),base),
                             J_chain,
                             Eigen::MatrixXd::Zero(constraint_matrix.rows(), q.size()-(ee+J_base.cols())),
                             J_base;

    H_chain = H_base * H_chain;
    Eigen::VectorXd currentPose(constraint_matrix.rows());
    if(constraint_matrix.rows() > 3)
        currentPose << H_chain.topRightCorner(3,1),
                       Rmath::xyzEulerAngles( H_chain.topLeftCorner(3,3) ).head(constraint_matrix.rows()-3);
    else
        currentPose << H_chain.col(3).head(constraint_matrix.rows());

    for(int i=0; i<bounds.rows(); ++i)
        bounds[i] = K * (desiredPose(i)-currentPose(i)) + desiredVel(i);
}
