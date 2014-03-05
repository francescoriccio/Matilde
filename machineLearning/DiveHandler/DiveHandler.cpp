/**
* @file DiveHandler.cpp
*
*	This source file contains the implementation of a module working as a dive handler for the goalie.
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
//#define DIVEHANDLER_DEBUG
#define DIVEHANDLER_TRAINING_DEBUG
#define DIVEHANDLER_TRAINING
//#define RAND_PERMUTATIONS

#define NEGATIVE_REWARD -1.0
#define POSITIVE_REWARD 1.5

#define REWARD_WORST 999999.9

// Debug messages template
#define SPQR_ERR(x) std::cerr << "\033[22;31;1m" <<"[DiveHandler] " << x << "\033[0m"<< std::endl;
#define SPQR_INFO(x) std::cerr << "\033[22;34;1m" <<"[DiveHandler] " << x << "\033[0m" << std::endl;
#define SPQR_SUCCESS(x) std::cerr << "\033[0;32;1m" <<"[DiveHandler] " << x << "\033[0m" << std::endl;
#define SPQR_FAILURE(x) std::cerr << "\033[22;31;1m" <<"[DiveHandler] " << x << "\033[0m"<< std::endl;

#define LEARNING_STATE(x) \
    if(x == 1) std::cerr << "\033[22;34;1m"<<"Learner state: disabled. "<<"\033[0m" << std::endl; \
    else if(x == 2) std::cerr << "\033[22;34;1m"<<"Learner state: paused (waiting for reward). "<<"\033[0m" << std::endl; \
    else if(x == 3) std::cerr << "\033[22;34;1m"<<"Learner state: enabled. "<<"\033[0m" << std::endl; \


MAKE_MODULE(DiveHandler, SPQR-Modules)

// Shortcut to compute the magnitude of a vector
double magnitude(std::vector<double> v)
{
    double m = 0.0;
    for (unsigned int i = 0; i < v.size(); ++i)
        m += v.at(i) * v.at(i);

    return sqrt(m);
}


/** --------------------- CoeffsLearner: base class --------------------- */


/*
 * Simple setters for the learner's parameters and coefficients.
 */
void DiveHandler::CoeffsLearner::setCoeffs(const std::vector<double>& _coeffs)
{
    coeffs = _coeffs;
}

void DiveHandler::CoeffsLearner::setParam(const std::string& _key, double _value)
{
    params[_key] = _value;
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
DiveHandler::PGLearner::PGLearner( DiveHandler* _dhPtr, int _nCoeffs, double _epsilon, int _T, double _initValue, bool randomize ):
    // Initialize the base class
    CoeffsLearner(_nCoeffs, _initValue, _dhPtr),
    // Initialize the gradient estimate
    coeffsGradient(_nCoeffs, 0.0), coeffsBest(_nCoeffs, 0.0)
{
    // Initializing reward scores
    reward_score = 0.0;
    reward_norm = 1.0;
    rewardBest = REWARD_WORST;

    // Initializing coefficients
    if(randomize)
    {
        // Random initialization in [0, INIT_VALUE]
        srand(time(NULL));
        for( int i=0; i<_nCoeffs; ++i)
            coeffs.at(i) = (static_cast<double>(rand()%101)/100 ) *_initValue;
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
        double avg_variation = (magnitude(coeffs) - magnitude(coeffsBuffer.front()))/coeffsBuffer.size() ;
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
        double std_variation = pow(magnitude(coeffs)-magnitude(coeffsBuffer.front()) - avg_variation, 2) / coeffsBuffer.size();
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
    #ifdef DIVEHANDLER_TRAINING
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
        std::vector<double> perturbation(coeffs);

        for(unsigned int j=0; j<coeffs.size(); ++j)
            perturbation.at(j) += (rand()%3 -1)*params["epsilon"];

        perturbationsBuffer.push_back(perturbation);

#ifdef DIVEHANDLER_DEBUG
        SPQR_INFO("Generated perturbation: [" << perturbation.at(0) << ", " << perturbation.at(1) << "]");
#endif

    }
#else
    // Initialize a placeholder for perturbations
    std::vector<double> perturbation (coeffs.size(),0.0);

    // Generate all possible combinations recursively
    generatePerturbations(&perturbation, 0);

#ifdef DIVEHANDLER_DEBUG
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
void DiveHandler::PGLearner::generatePerturbations(std::vector<double>* partial_perturbation, unsigned int index)
{
    if (index == partial_perturbation->size()-1)
    {
        // Base case: generate all combinations of the last coefficient for the current partial vector
        for (int perturbation_type = -1; perturbation_type <= 1; ++perturbation_type)
        {
            // Compute last index and generate the final perturbation
            std::vector<double> perturbation (*partial_perturbation);
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
double DiveHandler::PGLearner::evaluatePerturbation( std::vector<double> R )
{
    // Dimensions check
    assert(R.size() == coeffs.size());

    if (R.at(0) == 0.0 || R.at(1) == 0.0)
        return REWARD_WORST;

    // Generate perturbated policy and call the DiveHandler object for evaluation
    double tDiveAndRecover = diveHandler_ptr->computeDiveAndRecoverTime(R.at(0), R.at(1));

    // Attractor
    std::vector<double> distanceToBest(2);
    distanceToBest.at(0) = coeffsBest.at(0) - R.at(0);
    distanceToBest.at(1) = coeffsBest.at(1) - R.at(1);

#ifdef DIVEHANDLER_TRAINING_DEBUG
    SPQR_INFO("Perturbated policy: [" << R.at(0) << ", " << R.at(1)
              << "], Score: " << ((1.0-LAMBDA1)*fabs(diveHandler_ptr->tBall2Goal-tDiveAndRecover)+LAMBDA1*magnitude(distanceToBest)));
#endif

    return (1.0-LAMBDA1)*fabs(diveHandler_ptr->tBall2Goal - tDiveAndRecover) +
            LAMBDA1*magnitude(distanceToBest);

//    return (1.0-LAMBDA1-LAMBDA2)*fabs(tDiveAndRecover) +
//           LAMBDA1*fabs(diveHandler_ptr->tBall2Goal - tDiveAndRecover) +
//           LAMBDA2*fabs(1.0 - ((coeffs.at(0) + R.at(0))+(coeffs.at(1) + R.at(1))));

}


/* TOTEST&COMMENT */
void DiveHandler::PGLearner::updateParams(const std::list<double>& rewards)
{
    // Re-initialize reward scores
    reward_score = 0.0;
    if (!rewards.empty()) reward_norm = 0.0;
    int discount_exp = 0;
    int positives = 0;

    std::list<double>::const_iterator i = rewards.begin();
    while (i != rewards.end())
    {
        // Counting positives
        if (*i == POSITIVE_REWARD)
            ++positives;

        // Computing discounted rewards
        reward_score += (*i) * pow(GAMMA, discount_exp);
        reward_norm += fabs((*i) * pow(GAMMA, discount_exp));
        ++i; ++discount_exp;        
    }

    //Adjusting PG parameters according to the obtained score
    setParam("epsilon", exp( -reward_score / REWARDS_HISTORY_SIZE ) * getParam("epsilon"));

    // Update best performance
    if (rewardGradient < rewardBest)
    {
        rewardBest = rewardGradient;
        coeffsBest = coeffs;
    }
#ifdef DIVEHANDLER_TRAINING_DEBUG
    SPQR_INFO("Positive rewards: " << positives << " out of " << rewards.size());
    SPQR_INFO("Negative rewards: " << (rewards.size() - positives) << " out of " << rewards.size());
    SPQR_INFO("Reward total score: " << reward_score);
    SPQR_INFO("Best evaluation so far: [ " << coeffsBest.at(0) << ", " << coeffsBest.at(1) << " ] with score: " << rewardBest);
#endif

#ifdef DIVEHANDLER_TRAINING
    SPQR_INFO( "Epsilon value changed to: " << getParam("epsilon") << " according to the obtained rewards. ");
#endif

#ifdef RAND_PERMUTATIONS
    setParam("T", exp( -reward_score / REWARDS_HISTORY_SIZE ) * getParam("T"));
#endif
}


/* TOTEST&COMMENT */
bool DiveHandler::PGLearner::updateCoeffs()
{
    if( iter_count == MAX_ITER || converged() )
        return false;
    else
        {
#ifdef DIVEHANDLER_TRAINING
            SPQR_INFO( "PG algorithm, iteration " << iter_count << "... " );
#endif
            // First generate the set of random perturbation for the current coefficients
            generatePerturbations();

            // For each perturbation, evaluate with the objective function and store the result in a temporary container
            std::vector<double> evaluatedPerturbations (perturbationsBuffer.size());
            PGbuffer::const_iterator evaluator;
            for(evaluator = perturbationsBuffer.begin(); evaluator != perturbationsBuffer.end(); ++evaluator)
                evaluatedPerturbations.push_back( evaluatePerturbation(*evaluator) );

            // Compute the average 'gradient' for the current coefficients
            std::vector<double> coeffs_avgGradient(coeffs.size());

#ifdef RAND_PERMUTATIONS
            // For each coefficient, compute the average score to determine the correspondent 'gradient' entry
            PGbuffer::const_iterator current_perturbation = perturbationsBuffer.begin();
            for( unsigned int n = 0; n < coeffs.size(); ++n )
            {
                std::vector<double> score_plus, score_minus, score_zero;

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
                double avg_plus = 0.0;
                for (unsigned int j = 0; j < score_plus.size(); ++j)
                    avg_plus += score_plus.at(j) / score_plus.size();

                // Sum up all negative perturbation scores
                double avg_minus = 0.0;
                for (unsigned int j = 0; j < score_minus.size(); ++j)
                    avg_minus += score_minus.at(j) / score_minus.size();

                // Sum up all null perturbation scores
                double avg_zero = 0.0;
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
                double avg_minus = 0.0 , avg_zero = 0.0, avg_plus = 0.0;
                for( unsigned int i = 0; i < evaluatedPerturbations.size(); i = i + pow(3,n) )
                {
                    for( unsigned int k = i; k < i + pow(3,n); ++k )
                    {
                        double evaluation = evaluatedPerturbations.at(k) / (evaluatedPerturbations.size()/3);

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
            // Evaluate the gradient
            rewardGradient = evaluatePerturbation(coeffs_avgGradient);

            // Avoid 'nan' when the gradient is zeroed
            double normalization = 1.0;
            if (magnitude(coeffs_avgGradient) != 0)
                normalization = magnitude(coeffs_avgGradient);


#ifdef DIVEHANDLER_TRAINING
            SPQR_INFO("Computed policy gradient: [ " << coeffs_avgGradient.at(0)/normalization
                      << ", " << coeffs_avgGradient.at(1)/normalization << " ]");
            SPQR_INFO("Gradient score (before normalization): " << rewardGradient);
#endif
            // Weight new gradient estimate and previous one according to the reward score
            std::vector<double> newGradient (coeffsGradient.size());
            for( unsigned int j=0; j<newGradient.size(); ++j )
                newGradient.at(j) = coeffs_avgGradient.at(j)/normalization;

#ifdef DIVEHANDLER_TRAINING
            SPQR_INFO("New policy gradient: [ " << newGradient.at(0)
                      << ", " << newGradient.at(1) << " ]");
#endif

            // Update coefficients history
            coeffsBuffer.push_front(coeffs);
            // Crop buffer
            if (coeffsBuffer.size() > BUFFER_DIM)
                coeffsBuffer.resize(BUFFER_DIM);

            // Update the coefficients following the gradient direction
            for( unsigned int i=0; i<coeffs_avgGradient.size(); ++i )
            {
                // Coefficients
                coeffs.at(i) += - newGradient.at(i) * getParam("epsilon");
                // Gradient estimate
                coeffsGradient.at(i) = newGradient.at(i);

                // Crop negative coefficients
                if (coeffs.at(i) < 0) coeffs.at(i) = 0.0;
            }

#ifdef DIVEHANDLER_TRAINING
            SPQR_INFO("New coefficients: [ " << coeffs.at(0) << ", " << coeffs.at(1) << " ]");
#endif
            ++iter_count;

            return true;
    }
}


/** --------------------------- Dive Handler ---------------------------- */


/*
 * Default class constructor: initializes all parameters and generates the learning agent.
 */
DiveHandler::DiveHandler():
    diveType(none), state(static_cast<DiveHandler::LearningState>(SPQR::GOALIE_LEARNING_STATE)),
    learner(new PGLearner(this, 2, EPSILON, T)), opponentScore(0), tBall2Goal(SPQR::FIELD_DIMENSION_Y),
    tDive(0.0), tBackInPose(0.0), ballProjectionIntercept(SPQR::FIELD_DIMENSION_Y), distanceBall2Goal(SPQR::FIELD_DIMENSION_X)
{
#ifdef DIVEHANDLER_TRAINING
    SPQR_INFO("Initializing PGlearner...");
    std::vector<double> coeffs = learner->getCoeffs();
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
    double A1 = (theBallModel.estimate.position.y - theBallModel.estimate.velocity.y) - theBallModel.estimate.position.y;
    double B1 = theBallModel.estimate.position.x - (theBallModel.estimate.position.x - theBallModel.estimate.velocity.x);
    double C1 = A1*theBallModel.estimate.position.x + B1*theBallModel.estimate.position.y;

    // Goal line
    double A2 = SPQR::GOALIE_FAR_LIMIT_Y - -SPQR::GOALIE_FAR_LIMIT_Y;

    // Cross product/determinant
    double det = - A2*B1;

    // Y-intercept initialized with the maximum value possible
    double yIntercept = SPQR::FIELD_DIMENSION_Y;

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

    // Computing the distance vector from the ball to the goal
    double delta_x = -SPQR::FIELD_DIMENSION_X - theGlobalBallEstimation.singleRobotX;
    double delta_y = ballProjectionIntercept - theGlobalBallEstimation.singleRobotY;
    // Estimated distance from the ball
    distanceBall2Goal = sqrt( delta_x*delta_x + delta_y*delta_y);
}

/*
 * Estimation of the time needed for the ball to reach the goal line, from its its estimated position and velocity.
 * If the ball is either too slow or completely off target, such time is set to -1.0 by default.
 * The estimated time for the goalie to recover its position is defined accordingly.
 */
void DiveHandler::estimateDiveTimes()
{
    // Check whether the ball is actually moving toward the goal
    if ( (theBallModel.estimate.velocity.abs() != 0.0)
         && (theBallModel.estimate.velocity.x < 0.0) )
        // Use a constant velocity approximation to the estimate the time interval
        tBall2Goal = 1000.0 * ( distanceBall2Goal / theBallModel.estimate.velocity.abs() );
    else
        // Otherwise, set the parameter to a meaningless value
        tBall2Goal = -1.0;

    // Using the appropriate estimates for recover and reposition times
    double tRecover = 0.0;
    double tReposition = 0.0;
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
inline double DiveHandler::computeDiveAndRecoverTime(double alpha1, double alpha2)
{
    return alpha2*( alpha1*tBall2Goal - tDive );
}

/* TOTEST&COMMENT */
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
        if( (distanceBall2Goal < SPQR::FIELD_DIMENSION_X) && (fabs(ballProjectionIntercept) < SPQR::FIELD_DIMENSION_Y) )
        {
            // Estimate all temporal parameters
            estimateDiveTimes();

#ifdef DIVEHANDLER_DEBUG
            SPQR_INFO("Ball projection: " << ballProjectionIntercept);
            SPQR_INFO("PAPO time: " << tBall2Goal);
            SPQR_INFO("Dive time: " << tDive);
            SPQR_INFO("Back-in-position time: " << tBackInPose);
            LEARNING_STATE( (int)state );
#endif

            // The module is in the learning state and a reward has been received
            if( (state == learning) )
            {
                // Perform a single iteration of the learning algorithm
                if( learner->updateCoeffs() )
                {
                    // Change the state in 'waiting for reward'
                    state = waitReward;
                    // Flag a pending reward to the goalie behavior
                    diveHandle.rewardAck = false;
                }
                else
                    // The algorithm has converged: turning off learning
                    state = notLearning;

            }
            // The module is in the learning state, waiting for the next reward
            else if( state == waitReward )
            {
                // The opponent team scores: the goalie failed and gets a negative reward
                if(opponentScore != (int)theOpponentTeamInfo.score)
                {
                    // The learner obtains a negative reward
                    rewardHistory.push_front(NEGATIVE_REWARD);

                    // Crop the buffer
                    if (rewardHistory.size() > REWARDS_HISTORY_SIZE)
                        rewardHistory.resize(REWARDS_HISTORY_SIZE);
                    // Update opponent score
                    opponentScore = (int)theOpponentTeamInfo.score;

#ifdef DIVEHANDLER_TRAINING
                    SPQR_FAILURE("The opponent team scored! Negative reward for the learner. ");
#endif
                    // A reward has been received: re-enable learning
                    state = learning;
                    // Clear the pending reward
                    if(!diveHandle.rewardAck)
                        diveHandle.rewardAck = true;
                }
                // The own team scores: user-guided move to provide the goalie a positive reward
                else if(ownScore != (int)theOwnTeamInfo.score)
                {
                    // The learner obtains a positive reward
                    rewardHistory.push_front(POSITIVE_REWARD);

                    // Crop the buffer
                    if (rewardHistory.size() > REWARDS_HISTORY_SIZE)
                        rewardHistory.resize(REWARDS_HISTORY_SIZE);
                    // Update own score
                    ownScore = (int)theOwnTeamInfo.score;

#ifdef DIVEHANDLER_TRAINING
                    SPQR_SUCCESS("The goalie has succeeded! Positive reward for the learner.  ");
#endif
                    // A reward has been received: re-enable learning
                    state = learning;
                    // Clear the pending reward
                    if(!diveHandle.rewardAck)
                        diveHandle.rewardAck = true;
                }
            }

            // Use the reward to adjust the algorithm parameters
            if( state == learning )
                learner->updateParams(rewardHistory);

            // Compute the dive time using the current coefficients as T = alpha2 * (alpha1*T_PAPO - T_dive)
            double diveTime = (learner->getCoeffs()).at(1) * ( (learner->getCoeffs()).at(0) * tBall2Goal - tDive );

#ifdef DIVEHANDLER_DEBUG
            SPQR_INFO( "Estimated overall time to dive and recover position: " <<
                      computeDiveAndRecoverTime( (learner->getCoeffs()).at(0), (learner->getCoeffs()).at(1) ) );
            SPQR_INFO("Suggested dive in " << diveTime << " ms. ");
#endif

            // Update the DiveHandle
            if (diveTime > 0.0)
                diveHandle.diveTime = diveTime;
            else
                diveHandle.diveTime = -1.0;

#ifdef DIVEHANDLER_TRAINING
            if (diveTime > 0.0)
            {
                if(diveHandle.diveTime < SPQR::GOALIE_DIVE_TIME_TOLERANCE)
                    SPQR_INFO("Dive now! ");
            }
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
