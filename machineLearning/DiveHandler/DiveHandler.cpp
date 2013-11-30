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
#include <cmath>
#include <time.h>

#include "DiveHandler.h"

// Uncomment to have debug information
//#define DEBUG_MODE
//#define RAND_PERMUTATIONS

#define NEGATIVE_REWARD -1.0
#define POSITIVE_REWARD 1.0

// Debug messages template
#define SPQR_ERR(x) std::cerr << "\033[22;31;1m" <<"[DiveHandler] " << x << "\033[0m"<< std::endl;
#define SPQR_INFO(x) std::cerr << "\033[22;34;1m" <<"[DiveHandler] " << x << "\033[0m" << std::endl;
#define SPQR_SUCCESS(x) std::cerr << "\033[0;32;1m" <<"[DiveHandler] " << x << "\033[0m" << std::endl;

MAKE_MODULE(DiveHandler, SPQR-Modules)

// Shortcut to compute the magnitude of a vector
float magnitude(std::vector<float> v)
{
    float m = 0.0;
    for (unsigned int i = 0; i < v.size(); ++i)
        m += v.at(i) * v.at(i);

    return sqrt(m);
}


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
            coeffs.at(i) = (static_cast<float>(rand()%101)/100 ) *_initValue;
    }

    // Initializing parameters
    setParam("epsilon", _epsilon);

#ifdef RAND_PERMUTATIONS
    setParam("T", _T);
#else
    setParam("T", pow(3,coeffs.size()));
#endif
}

/* TOTEST&COMMENT */
bool DiveHandler::PGLearner::converged()
{
    // Skip convergence check if the buffer is not full
    if (coeffsBuffer.size() < BUFFER_DIM)
        return false;
    // Average every coefficients variation across the buffer
    else
    {
        // Compute variations mean
        // Delta previous to current step
        float avg_variation = (magnitude(coeffs) - magnitude(coeffsBuffer.front()))/coeffsBuffer.size() ;
        // Iterate over the whole buffer and compute deltas from step i-1 to i
        PGbuffer::const_iterator i = coeffsBuffer.begin();
        PGbuffer::const_iterator j = coeffsBuffer.begin(); ++j;
        while (j != coeffsBuffer.end())
        {
            avg_variation += ( magnitude(*i) - magnitude(*j) )/coeffsBuffer.size();
            ++i; ++j;
        }

        // Compute variations standard deviation
        // Delta previous to current step
        float std_variation = pow(magnitude(coeffs)-magnitude(coeffsBuffer.front()) - avg_variation, 2) / coeffsBuffer.size();
        // Iterate over the whole buffer and compute deltas from step i-1 to i
        PGbuffer::const_iterator k = coeffsBuffer.begin();
        PGbuffer::const_iterator t = coeffsBuffer.begin(); ++t;
        while (t != coeffsBuffer.end())
        {
            std_variation += (pow(magnitude(*k)-magnitude(*t) - avg_variation, 2)) / coeffsBuffer.size();
            ++k; ++t;
        }
        std_variation = sqrt(std_variation);

        // Check result against variation threshold
        if ((avg_variation < CONVERGENCE_THRESHOLD) && (std_variation < CONVERGENCE_THRESHOLD))
        {
    #ifdef DEBUG_MODE
            SPQR_SUCCESS("PGLearner converged!");
            SPQR_SUCCESS("Coefficients values:");
            for (unsigned int i = 0; i < coeffs.size(); ++i)
                SPQR_SUCCESS("\t" << coeffs.at(i));
    #endif
            return true;
        }
        else
            return false;
    }
}

/* TOTEST&COMMENT */
void DiveHandler::PGLearner::generatePerturbations()
{
    // Clean up the buffer
    if(! perturbationsBuffer.empty())
        perturbationsBuffer.clear();

#ifdef RAND_PERMUTATIONS
    srand(time(NULL));

    for(int i=0; i<params["T"]; ++i)
    {
        std::vector<float> perturbation(coeffs);

        for(unsigned int j=0; j<coeffs.size(); ++j)
            perturbation.at(j) += (rand()%3 -1)*params["epsilon"];

        perturbationsBuffer.push_back(perturbation);

#ifdef DEBUG_MODE
        SPQR_INFO("Generated perturbation: [" << perturbation.at(0) << ", " << perturbation.at(1) << "]");
#endif

    }
#else
    // Initialize a placeholder for perturbations
    std::vector<float> perturbation (coeffs.size(),0.0);

    // Generate all possible combinations recursively
    generatePerturbations(&perturbation, 0);

#ifdef DEBUG_MODE
    PGbuffer::const_iterator printer = perturbationsBuffer.begin();
    while(printer != perturbationsBuffer.end())
    {
        SPQR_INFO("Generated perturbation: [" << (*printer).at(0) << ", " << (*printer).at(1) << "]");
        ++printer;
    }
#endif

#endif

}

/* TOTEST&COMMENT */
void DiveHandler::PGLearner::generatePerturbations(std::vector<float>* partial_perturbation, unsigned int index)
{
    if (index == partial_perturbation->size()-1)
    {
        // Base case: generate all combinations of the last coefficient for the current partial vector
        for (int perturbation_type = -1; perturbation_type <= 1; ++perturbation_type)
        {
            // Compute last index and generate the final perturbation
            std::vector<float> perturbation (*partial_perturbation);
            perturbation.at(index) = coeffs.at(index) + perturbation_type * params["epsilon"];

            // Update the perturbations buffer
            perturbationsBuffer.push_back(perturbation);
        }
    }
    else
    {
        for (int perturbation_type = -1; perturbation_type <= 1; ++perturbation_type)
        {
            // Generate current perturbation
            partial_perturbation->at(index) = coeffs.at(index) + perturbation_type * params["epsilon"];

            // Generate all possible perturbations for the current index
            generatePerturbations(partial_perturbation, index+1);
        }
    }
}


/* TOCOMMENT */
float DiveHandler::PGLearner::evaluatePerturbation( std::vector<float> R )
{
    // Dimensions check
    assert(R.size() == coeffs.size());
    // Generate perturbated policy and call the DiveHandler object for evaluation
    return diveHandler_ptr->computeDiveAndRecoverTime(coeffs.at(0) + R.at(0), coeffs.at(1) + R.at(1));
}


/* TOTEST&COMMENT */
void DiveHandler::PGLearner::updateParams(const std::list<float>& rewards)
{
    float reward_score = 0.0;
    int discount_exp = 0;
    std::list<float>::const_iterator i = rewards.begin();
    while (i != rewards.end())
    {
        // Computing discounted rewards
        reward_score += (*i) * pow(GAMMA, discount_exp);
        ++i; ++discount_exp;
    }

    //Adjusting PG parameters according to the obtained score
    setParam("epsilon", exp( reward_score / rewards.size() ) * getParam("epsilon"));

#ifdef RAND_PERMUTATIONS
    setParam("T", exp( reward_score / rewards.size() ) * getParam("T"));
#endif
}


/* TOTEST&COMMENT */
bool DiveHandler::PGLearner::updateCoeffs()
{
    if( iter_count == MAX_ITER || converged() )
        return false;
    else
        {
            // First generate the set of random perturbation for the current coefficients
            generatePerturbations();

            // For each perturbation, evaluate with the objective function and store the result in a temporary container
            std::vector<float> evaluatedPerturbations (perturbationsBuffer.size());
            PGbuffer::const_iterator evaluator;
            for(evaluator = perturbationsBuffer.begin(); evaluator != perturbationsBuffer.end(); ++evaluator)
                evaluatedPerturbations.push_back( evaluatePerturbation(*evaluator) );

            // Compute the average 'gradient' for the current coefficients
            std::vector<float> coeffs_avgGradient(coeffs.size());

#ifdef RAND_PERMUTATIONS
            // For each coefficient, compute the average score to determine the correspondent 'gradient' entry
            PGbuffer::const_iterator current_perturbation = perturbationsBuffer.begin();
            for( unsigned int n = 0; n < coeffs.size(); ++n )
            {
                std::vector<float> score_plus, score_minus, score_zero;

                // Keep track of the perturbation type and store each score in a container
                for( unsigned int i = 0; i < evaluatedPerturbations.size(); ++i )
                {
                    if ( ((*current_perturbation).at(n) - coeffs.at(n)) > 0 )
                        score_plus.push_back(evaluatedPerturbations.at(i));
                    else if ( ((*current_perturbation).at(n) - coeffs.at(n)) < 0 )
                        score_minus.push_back(evaluatedPerturbations.at(i));
                    else
                        score_zero.push_back(evaluatedPerturbations.at(i));

                    ++current_perturbation;
                }

                // Sum up all positive perturbation scores
                float avg_plus = 0.0;
                for (unsigned int j = 0; j < score_plus.size(); ++j)
                    avg_plus += score_plus.at(j) / score_plus.size();

                // Sum up all negative perturbation scores
                float avg_minus = 0.0;
                for (unsigned int j = 0; j < score_minus.size(); ++j)
                    avg_minus += score_minus.at(j) / score_minus.size();

                // Sum up all null perturbation scores
                float avg_zero = 0.0;
                for (unsigned int j = 0; j < score_zero.size(); ++j)
                    avg_zero += score_zero.at(j) / score_zero.size();

                if( avg_zero <= avg_plus && avg_zero<= avg_minus )
                    coeffs_avgGradient.at(n) = 0.0;
                else
                    coeffs_avgGradient.at(n) = avg_plus - avg_minus;
            }
#else
            // For each coefficient, compute different averages to determine the correspondent 'gradient' entry
            for( unsigned int n = 0; n < coeffs.size(); ++n )
            {
                int avg_selector = 0;
                float avg_minus = 0.0 , avg_zero = 0.0, avg_plus = 0.0;
                for( unsigned int i = 0; i < evaluatedPerturbations.size(); i = i + pow(3,n) )
                {
                    for( unsigned int k = i; k < i + pow(3,n); ++k )
                    {
                        float evaluation = evaluatedPerturbations.at(k) / (evaluatedPerturbations.size()/3);

                        if( (avg_selector)%3 == 0 ) avg_minus += evaluation;
                        if( (avg_selector)%3 == 1 ) avg_zero += evaluation;
                        if( (avg_selector)%3 == 2 ) avg_plus += evaluation;
                    }
                    ++avg_selector;
                }
                // evaluate An
                if( avg_zero <= avg_plus && avg_zero<= avg_minus )
                    coeffs_avgGradient.at(coeffs.size() - (n +1)) = 0.0;
                else
                    coeffs_avgGradient.at(coeffs.size() - (n +1)) = avg_plus - avg_minus;
            }
#endif

#ifdef DEBUG_MODE
            SPQR_INFO("Computed policy gradient: [ " << coeffs_avgGradient.at(0)/magnitude(coeffs_avgGradient)
                      << ", " << coeffs_avgGradient.at(1)/magnitude(coeffs_avgGradient) << " ]");
#endif

            // Update coefficients history
            coeffsBuffer.push_front(coeffs);
            // Crop buffer
            if (coeffsBuffer.size() > BUFFER_DIM)
                coeffsBuffer.resize(BUFFER_DIM);

            // Update the coefficients following the gradient direction
            for( unsigned int i=0; i<coeffs_avgGradient.size(); ++i )
                coeffs.at(i) += - (coeffs_avgGradient.at(i)/magnitude(coeffs_avgGradient)) * ETA;

            ++iter_count;

            return true;
    }
}


/** --------------------------- Dive Handler ---------------------------- */


/*
 * Default class constructor: initializes all parameters and generates the learning agent.
 */
DiveHandler::DiveHandler():
    diveType(none), learner(new PGLearner(this, 2, EPSILON, T, 1.0, true)), opponentScore(0),
    tBall2Goal(SPQR::FIELD_DIMENSION_Y), tDive(0.0), tBackInPose(0.0)
{
#ifdef DEBUG_MODE
    SPQR_INFO("Initializing PGlearner...");
    std::vector<float> coeffs = learner->getCoeffs();
    SPQR_INFO("Coefficients: alpha 1 = " << coeffs.at(0) << ", alpha 2 = " << coeffs.at(1));
    SPQR_INFO("Parameters: epsilon = " << learner->getParam("epsilon") << ", T = " << learner->getParam("T"));

    learner->updateCoeffs();
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
        else if( fabs(yIntercept) < SPQR::GOALIE_CLOSE_LIMIT_Y/2)
            diveType = stopBall;
        else
            // Any other case: no dive at all
            diveType = none;
        }

    // Using the appropriate estimate for the dive time
    if (diveType == lDive || diveType == rDive )
        tDive = SPQR::GOALIE_DIVE_TIME;
    else if (diveType == lcloseDive || diveType == rcloseDive )
        tDive = SPQR::GOALIE_CLOSE_DIVE_TIME;
    else if (diveType == stopBall )
        tDive = SPQR::GOALIE_STOP_BALL_TIME;
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
    else if( diveType == stopBall )
    {
        // stop ball: the robot has to stand up and stop the ball
        tRecover = SPQR::GOALIE_STOP_BALL_RECOVER_TIME;
    }

    // Total time needed to recover the original position
    tBackInPose = tRecover + tReposition;
}

/* TOCOMMENT */
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
//    theOpponentTeamInfo.score;
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
                // Perform a single iteration of the learning algorithm
                if( learner->updateCoeffs() )
                    // Change the state in 'waiting for reward'
                    state = waitReward;
                else
                    // the algorithm converged
                    state = notLearning;

            }
            // The module is in the learning state, waiting for the next reward
            else if( state == waitReward )
            {
                if(diveHandle.diveTime < SPQR::GOALIE_DIVE_TIME_TOLERANCE) //TODO: check timing according to theOpponentTeamInfo.score
                {
                    // polling for reward
                    // if the opponent team scored
                    if(opponentScore != (int)theOpponentTeamInfo.score)
                    {
                        rewardHistory.push_front(NEGATIVE_REWARD);
                        if (rewardHistory.size() > REWARDS_HISTORY_SIZE)
                            rewardHistory.resize(REWARDS_HISTORY_SIZE);

                        //update opponent score
                        opponentScore = (int)theOpponentTeamInfo.score;
                    }
                    // if the goalie succeeds
                    else
                    {
                        rewardHistory.push_front(POSITIVE_REWARD);
                        if (rewardHistory.size() > REWARDS_HISTORY_SIZE)
                            rewardHistory.resize(REWARDS_HISTORY_SIZE);
                    }

                    state = learning;
                }
            }

            // Use the reward to adjust the algorithm parameters
            if( state == learning )
                learner->updateParams(rewardHistory);

#ifdef DEBUG_MODE
            SPQR_INFO( "Estimated overall time to dive and recover position: " <<
                      computeDiveAndRecoverTime( (learner->getCoeffs()).at(0), (learner->getCoeffs()).at(1) ) );
#endif
            // Compute the dive time using the current coefficients as T = alpha2 * (alpha1*T_PAPO - T_dive)
            float diveTime = (learner->getCoeffs()).at(1) * ( (learner->getCoeffs()).at(0) * tBall2Goal - tDive );
            // Update the DiveHandle
            if (diveTime > 0.0)
                diveHandle.diveTime = diveTime;
            else
                diveHandle.diveTime = -1.0;

#ifdef DEBUG_MODE
            if (diveHandle.diveTime > 0.0) SPQR_INFO("Dive in " << diveHandle.diveTime << " ms! ");
            if (diveHandle.diveTime < SPQR::GOALIE_DIVE_TIME_TOLERANCE) SPQR_INFO("Dive now! ");
            else SPQR_INFO("Stay still... ");
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
