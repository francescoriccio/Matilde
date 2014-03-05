/**
* @file DiveHandler.h
*
*	This header file contains the declaration of a module working as a dive handler for the goalie.
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

#pragma once

// Includes

#include <string>
#include <vector>
#include <list>
#include <map>

#include "Tools/Module/Module.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/SPQR-Representations/ConfigurationParameters.h"
#include "Representations/SPQR-Representations/RobotPoseSpqrFiltered.h"
#include "Representations/SPQR-Representations/GlobalBallEstimation.h"
#include "Representations/SPQR-Representations/DiveHandle.h"
#include "SPQR-Libraries/PTracking/src/Utils/AgentPacket.h"


// Module definition


MODULE(DiveHandler)
    REQUIRES(OpponentTeamInfo)
    REQUIRES(OwnTeamInfo)
    REQUIRES(FrameInfo)
    REQUIRES(RobotInfo)
    REQUIRES(RobotPoseSpqrFiltered)
    REQUIRES(BallModel)
    REQUIRES(GlobalBallEstimation)
    PROVIDES(DiveHandle)
END_MODULE


// Termination conditions
#define MAX_ITER 300
#define CONVERGENCE_THRESHOLD 0.01
// PG parameters
#define GAMMA 0.5
#define BUFFER_DIM 10
#define REWARDS_HISTORY_SIZE 15
#define EPSILON 0.10
#define T 15
// Evaluation weight
#define LAMBDA1 0.7
//#define LAMBDA2 0.3


// Module class declaration


class DiveHandler : public DiveHandlerBase
{
    // Learning state
    enum LearningState
    {
        // Learning disabled
        notLearning = 1,
        // Learning paused, expecting reward
        waitReward,
        // Learning active
        learning
    };

    // Dive type
    enum Dive
    {
        // No dive at all
        none = 1,
        // Long dive on the left
        lDive,
        // Long dive on the right
        rDive,
        // Close dive on the left
        lcloseDive,
        // Close dive on the right
        rcloseDive,
        // Stop the ball without diving
        stopBall
    };

    // Inner base class modeling the learning agent
    class CoeffsLearner
    {
        protected:
        // Set of coefficients representing the learning objective
        std::vector<double> coeffs;
        // Set of fixed parameters defining the cost funcion
        std::map<std::string, double> params;

        // Iteration counter
        int iter_count;

        // Pointer to the DiveHandler object whose coefficients are learned
        DiveHandler* diveHandler_ptr;
		
        public:
        // Default constructor
        CoeffsLearner(int _nCoeffs, double _initValue, DiveHandler* _dhPtr):
            coeffs(_nCoeffs, _initValue), iter_count(0), diveHandler_ptr(_dhPtr) { }

        // Setter/getter for the coefficients
        void setCoeffs(const std::vector<double>& _coeffs);
        inline std::vector<double> getCoeffs(){ return coeffs; }

        // Setter/getter for the parameters
        void setParam(const std::string& _key, double _value);
        inline double getParam(std::string _key){ return params[_key]; }

        // Update coefficients performing a step of the learning algorithm
        virtual bool updateCoeffs() = 0;

        // Use the obtained rewards to adjust the algorithm parameters
        virtual void updateParams(const std::list<double>& rewards) = 0;

    };

    // Inner class modeling a PolicyGradient-based learning agent
    class PGLearner : public CoeffsLearner
    {
        typedef std::list< std::vector<double> > PGbuffer;

        private:

        // Current estimate for the coefficients gradient
        std::vector<double> coeffsGradient;
        // Best individual performance achieved so far
        std::vector<double> coeffsBest;

        // Current reward score
        double reward_score;
        // Current reward normalization factor
        double reward_norm;
        // Score of the current gradient estimate
        double rewardGradient;
        // Best gradient score so far
        double rewardBest;

        // Memory buffer for the PG algorithm
        PGbuffer coeffsBuffer;
        // Set of perturbations to be performed
        PGbuffer perturbationsBuffer;

        // Check for convergence of the algorithm
        bool converged();

        // Recursive perturbation generator
        void generatePerturbations(std::vector<double>* partial_perturbation, unsigned int index);

        public:

        // Default constructor
        PGLearner(DiveHandler* _dhPtr, int _nCoeffs, double _epsilon = EPSILON,
                  int _T = T, double _initValue = 1.0, bool randomize = false);

        // Generate a set of perturbations for the current policy
        void generatePerturbations();

        // Evaluate a single policy perturbation with the cost function
        double evaluatePerturbation( std::vector<double> R );

        // Update the PG parameters according to the obtained rewards
        void updateParams(const std::list<double>& rewards);

        // Update coefficients performing a step of the learning algorithm
        virtual bool updateCoeffs();

        // Update the best coefficient setting so far
        inline void updateCoeffsBest()
        {
            coeffsBest = coeffs;
        }

    };
	
// 	class GALearner : public CoeffsLearner
// 	{};
	
private:

    // Dive type currently selected
    Dive diveType;

    // Current learning state
    LearningState state;  
    // Learning agent
    CoeffsLearner* learner;
    // Obtained rewards
    std::list<double> rewardHistory;

    // Current scores
    int opponentScore;
    int ownScore;

    // Estimated time the ball needs to reach the goal
    // a.k.a. Tpapo (historical reasons)
    double tBall2Goal;
    // Estimated time needed for the current dive action to be performed
    double tDive;
    // Estimated time the goalie needs to back up to its original position
    double tBackInPose;

    // Estimated intersection between the ball projection and the goal line
    double ballProjectionIntercept;
    // Estimated distance of the ball from the own goal
    double distanceBall2Goal;

    // Computes parameters using the ball estimated position and velocity
    void estimateDiveTimes();
    void estimateBallProjection();

    // Compute the overall time the goalie needs to dive and then recover its position
    inline double computeDiveAndRecoverTime(double alpha1, double alpha2);
	
public:

    // Default constructor
    DiveHandler();
    // Destructor
    ~DiveHandler();

    // Setter for the reward list
    inline const std::list<double>& getRewardList() const
    {
        return rewardHistory;
    }

    // Update the DiveHandle for the goalie behavior
    void update(DiveHandle& diveHandle);
	
};
