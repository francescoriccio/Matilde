/**
* @file DiveHandler.cpp
*
*	This header file contains the implementation of a module working as a dive handler for the goalie.
*   Such handler is activated when the ball gets in the own field side, and it computes an estimate of its projection toward the goal
*   with respect to the goalie reference frame. It also provides estimates for the amount of time needed to dive, save the ball and
*   then get back to the goalie position. This measure is compared against the estimated time the ball needs to reach the goal.
*   With the use of reinforcement learning techniques (Policy Gradient, Genetic Algorithms) this module seeks the optimal diving strategy
*   by minimizing a cost function defined with the above mentioned parameters.
*   Output of this module is the representation DiveHandle, comprising a timer to trigger the dive action and the type of dive to be
*   performed (long dive, close dive, no dive at all).
*
* @author Claudio Delli Bovi, Francesco Riccio
*
*/

#include <stdlib.h>
#include <time.h>

#include "DiveHandler.h"

// Uncomment to have debug information
#define DEBUG_MODE
// Debug messages template
#define SPQR_ERR(x) std::cerr << "\033[22;31;1m" <<"[DiveHandler] " << x << "\033[0m"<< std::endl;
#define SPQR_INFO(x) std::cerr << "\033[22;34;1m" <<"[DiveHandler] " << x << "\033[0m" << std::endl;

MAKE_MODULE(DiveHandler, SPQR-Modules)


/** --------------------- CoeffsLearner: base class --------------------- */


/*
 * Simple setters for the learner's parameters and coefficients.
 */
void DiveHandler::CoeffsLearner::setCoeffs(const std::vector<float>& _coeffs)
{
    coeffs = _coeffs;
}

void DiveHandler::CoeffsLearner::setParam(const std::string& _key, float _value)
{
    // "Smart" insertion procedure using iterators (C++ 11)
    std::map<std::string, float>::iterator iter = params.begin();
    params.insert( iter, std::pair< std::string, float >(_key, _value) );
}



/** --------------------- CoeffsLearner: Policy Gradient --------------------- */


/*
 * Default constructor. Initializes the algorithm parameters and coefficients.
 * Input arguments are:
 * - The number of coefficients involved in the learning process (mandatory);
 * - The step size for the policy perturbation phase;
 * - The number of perturbations to be considered simultaneously;
 * - An initial value for the learning coefficients (or an upper bound for the random initialization of those);
 * - A flag indicating whether a fixed or random initialization has to be performed.
 */
DiveHandler::PGLearner::PGLearner( DiveHandler* _dhPtr, int _nCoeffs, float _epsilon, int _T, float _initValue, bool randomize ):
    // Initialize the base class
    CoeffsLearner(_nCoeffs, _initValue, _dhPtr)
{
    // Initializing coefficients
    if(randomize)
    {
        // Random initialization in [0, INIT_VALUE]
        srand(time(NULL));
        for( int i=0; i<_nCoeffs; ++i)
            coeffs.at(i) = (rand()/RAND_MAX)*_initValue;
    }

    // Initializing parameters
    setParam("epsilon", _epsilon);
    setParam("T", _T);
}

/* TODO */
bool DiveHandler::PGLearner::converged()
{
    // alpha1 alpha2 converge

    // if yes, store values in a file

    return false;
}

/* TODO */
void DiveHandler::PGLearner::generatePerturbations()
{
    // store perturbations
}

/*
 * TOCOMMENT
 */
float DiveHandler::PGLearner::evaluatePerturbation( std::vector<float> R )
{
    // Dimensions check
    assert(R.size() == coeffs.size());
    // Generate perturbated policy and call the DiveHandler object for evaluation
    return diveHandler_ptr->computeDiveAndRecoverTime(coeffs.at(0) + R.at(0), coeffs.at(1) + R.at(1));
}

/* TODO */
bool DiveHandler::PGLearner::updateCoeffs()
{
    // while stop criterion: MAX_ITER || converged()
    // generate the set of perturbation and store it in the private member
    // for each perturbation, evaluate it with the objective function and store the result in a temporary container
    // for each parameter, compute the 3 Avg values and determine An
    // update the coeffs with -(A/abs(A))*ETA where A is the 2D vector of Ans
    return false;
}



/** --------------------------- Dive Handler ---------------------------- */


/*
 * Default class constructor: initializes all parameters and generates the learning agent.
 */
DiveHandler::DiveHandler():
    diveType(none), learner(new PGLearner(this, 2, EPSILON, T, 1.0)),
    tBall2Goal(SPQR::FIELD_DIMENSION_Y), tDive(0.0), tBackInPose(0.0)
{
#ifdef DEBUG_MODE
    SPQR_INFO("Initializing PGlearner...");
    std::vector<float> coeffs = learner->getCoeffs();
    SPQR_INFO("Coefficients: alpha 1 = " << coeffs.at(0) << ", alpha 2 = " << coeffs.at(1));
    SPQR_INFO("Parameters: epsilon = " << learner->getParam("epsilon") << ", T = " << learner->getParam("T"));
#endif
}

/*
 * Default class destructor: destroys the learning agent and deallocates memory.
 */
DiveHandler::~DiveHandler()
{
    if (learner) delete learner;
}

/*
 * Computation of the ball projection on the goal line from its estimated position and velocity.
 * Intersecting such projection with the goal line itself yields the shift along the goalie Y-axis
 * at which the ball is expected to reach the goal.
 * Then, the diveTime and the diveType parameters are defined accordingly.
 */
void DiveHandler::estimateBallProjection()
{
    // Ball path line
    float A1 = (theBallModel.estimate.position.y - theBallModel.estimate.velocity.y) - theBallModel.estimate.position.y;
    float B1 = theBallModel.estimate.position.x - (theBallModel.estimate.position.x - theBallModel.estimate.velocity.x);
    float C1 = A1*theBallModel.estimate.position.x + B1*theBallModel.estimate.position.y;

    // Goal line
    float A2 = SPQR::GOALIE_FAR_LIMIT_Y - -SPQR::GOALIE_FAR_LIMIT_Y;

    // Cross product/determinant
    float det = - A2*B1;

    // Y-intercept initialized with the maximum value possible
    float yIntercept = SPQR::FIELD_DIMENSION_Y;

    // Non-singular case
    if( fabs(det) > SPQR::GOALIE_EPSILON_COLLINEAR )
    {
        // Computing Y-intercept
        yIntercept = (- A2*C1) / det;

        // Devising the type of dive to be performed
        if( yIntercept > ( SPQR::GOALIE_CLOSE_LIMIT_Y/2) && yIntercept < SPQR::GOALIE_FAR_LIMIT_Y )
            // Close intercept on the left
            diveType = lcloseDive;
        else if( yIntercept > SPQR::GOALIE_FAR_LIMIT_Y )
            // Far intercept on the left
            diveType = lDive;
        else if( yIntercept < (-SPQR::GOALIE_CLOSE_LIMIT_Y/2) && yIntercept > -SPQR::GOALIE_FAR_LIMIT_Y )
            // Close intercept on the right
            diveType = rcloseDive;
        else if( yIntercept < -SPQR::GOALIE_FAR_LIMIT_Y )
            // Far intercept on the right
            diveType = rDive;
        else
            // Any other case: no dive at all
            diveType = none;
        }

    // Using the appropriate estimate for the dive time
    if (diveType == lDive || diveType == rDive )
        tDive = SPQR::GOALIE_DIVE_TIME;
    else if (diveType == lcloseDive || diveType == rcloseDive )
        tDive = SPQR::GOALIE_CLOSE_DIVE_TIME;
    else
        tDive = 0.0;

    // Updating the class parameters with the obtained value
    ballProjectionIntercept = yIntercept;

}

/*
 * Estimation of the time needed for the ball to reach the goal line, from its its estimated position and velocity.
 * If the ball is either too slow or completely off target, such time is set to -1.0 by default.
 * The estimated time for the goalie to recover its position is defined accordingly.
 */
void DiveHandler::estimateDiveTimes()
{
    // Computing the distance vector from the ball to the goal
    float delta_x = -SPQR::FIELD_DIMENSION_X - theGlobalBallEstimation.singleRobotX;
    float delta_y = ballProjectionIntercept - theGlobalBallEstimation.singleRobotY;
    // Estimated distance from the ball
    float distanceBall2Goal = sqrt( delta_x*delta_x + delta_y*delta_y);

    // Check whether the ball is actually moving toward the goal
    if ( (theBallModel.estimate.velocity.abs() != 0.0)
         && (theBallModel.estimate.velocity.x < 0.0) )
        // Use a constant velocity approximation to the estimate the time interval
        tBall2Goal = 1000.0 * ( distanceBall2Goal / theBallModel.estimate.velocity.abs() );
    else
        // Otherwise, set the parameter to a meaningless value
        tBall2Goal = -1.0;

    // Using the appropriate estimates for recover and reposition times
    float tRecover = 0.0;
    float tReposition = 0.0;
    if( diveType == rcloseDive || diveType == lcloseDive )
        // Close dive: no need to back up to the original position
        tRecover = SPQR::GOALIE_CLOSE_DIVE_RECOVER_TIME;
    else if( diveType == rDive || diveType == lDive )
    {
        // Long dive: the robot has to stand up and reposition
        tRecover = SPQR::GOALIE_DIVE_RECOVER_TIME;
        tReposition = SPQR::GOALIE_DIVE_REPOSITION_TIME;
    }

    // Total time needed to recover the original position
    tBackInPose = tRecover + tReposition;

}

/*
 * TOCOMMENT
 */
inline float DiveHandler::computeDiveAndRecoverTime(float alpha1, float alpha2)
{
    return alpha2*( alpha1*tBall2Goal - tDive ) + tBackInPose;
}

/* TODO */
/*
 * The module update function.
 * At each time step, the goalie enables the DiveHandler if the ball is close enough.
 * Using the ball percept and the robot position, all DiveHandler parameters are estimated: then, if the module
 * is in the learning state, performs a single iteration of the algorithm and updates the policy coefficients.
 * Then, the learning process is suspended until a reward is obtained (in terms of ball saved or scored).
 * Such coefficients are used to provide the goalie with a DiveHandle, containing:
 * - A timer to trigger the dive action;
 * - The type of dive to be performed;
 * - The Y-intercept of the ball projection with the goal line.
 * If the module is not in the learning state, the coefficients current values are used without any further processing.
 */
void DiveHandler::update(DiveHandle& diveHandle)
{
    // Check you're actually the goalie...
    if (theRobotInfo.number == 1)
    {
        // Compute the ball projection estimate
        estimateBallProjection();
        // Update the DiveHandle
        diveHandle.ballProjectionEstimate = ballProjectionIntercept;

        // Check whether the ball is close enough
        if( theGlobalBallEstimation.singleRobotX < 0.0 && fabs(ballProjectionIntercept) < SPQR::FIELD_DIMENSION_Y )
        {
            // Estimate all temporal parameters
            estimateDiveTimes();

#ifdef DEBUG_MODE
            SPQR_INFO("Ball projection: " << ballProjectionIntercept);
            SPQR_INFO("PAPO time: " << tBall2Goal);
            SPQR_INFO("Dive time: " << tDive);
            SPQR_INFO("Back-in-position time: " << tBackInPose);
#endif
            // The module is in the learning state and a reward has been received
            if( state == learning )
            {
                // iterazione while PG

                // state = waitReward;
            }
            // The module is in the learning state, waiting for the next reward
            else if( state == waitReward )
            {
                // polling for reward

                // if reward -> state = learning
            }

            // Use the reward to adjust the algorithm parameters
            if( state == learning )
            {
                // adjust PG parameters wrt reward
            }

#ifdef DEBUG_MODE
            SPQR_INFO( "Estimated overall time to dive and recover position: " <<
                      computeDiveAndRecoverTime( (learner->getCoeffs()).at(0), (learner->getCoeffs()).at(1) ) );
#endif
            // Compute the dive time using the current coefficients as T = alpha2 * (alpha1*T_PAPO - T_dive)
            float diveTime = (learner->getCoeffs()).at(1) * ( (learner->getCoeffs()).at(0) * tBall2Goal - tDive );
            // Update the DiveHandle
            if (diveTime >= 0)
                diveHandle.diveTime = diveTime;
            else
                diveHandle.diveTime = -1.0;

#ifdef DEBUG_MODE
            if (diveHandle.diveTime > 0)
            {SPQR_INFO("Dive in " << diveHandle.diveTime << " ms! ");}
            else if (diveHandle.diveTime == 0)
            {SPQR_INFO("Dive now! ");}
            else
            {SPQR_INFO("Stay still... ");}
#endif

        }
        // If the ball is far away or completely off target, no dive has to performed
        else
        {
            diveHandle.diveTime = -1;
            diveHandle.diveType = diveType;
        }
    }

}
