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
    REQUIRES(FrameInfo)
    REQUIRES(RobotInfo)
    REQUIRES(RobotPoseSpqrFiltered)
    REQUIRES(BallModel)
    REQUIRES(GlobalBallEstimation)
    PROVIDES(DiveHandle)
END_MODULE


// Termination conditions
#define MAX_ITER 300
#define CONVERGENCE_THRESHOLD 0.05

// PG parameters
#define GAMMA 0.5
#define BUFFER_DIM 10
#define REWARDS_HISTORY_SIZE 15
#define ETA 0.4
#define EPSILON 0.15
#define T 15


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
        std::vector<float> coeffs;
        // Set of fixed parameters defining the cost funcion
        std::map<std::string, float> params;

        // Iteration counter
        int iter_count;

        // Pointer to the DiveHandler object whose coefficients are learned
        DiveHandler* diveHandler_ptr;
		
        public:
        // Default constructor
        CoeffsLearner(int _nCoeffs, float _initValue, DiveHandler* _dhPtr):
            coeffs(_nCoeffs, _initValue), iter_count(0), diveHandler_ptr(_dhPtr) { }

        // Setter/getter for the coefficients
        void setCoeffs(const std::vector<float>& _coeffs);
        inline std::vector<float> getCoeffs(){ return coeffs; }

        // Setter/getter for the parameters
        void setParam(const std::string& _key, float _value);
        inline float getParam(std::string _key){ return params[_key]; }

        // Update coefficients performing a step of the learning algorithm
        virtual bool updateCoeffs() = 0;

        // Use the obtained rewards to adjust the algorithm parameters
        virtual void updateParams(const std::list<float>& rewards) = 0;

    };

    // Inner class modeling a PolicyGradient-based learning agent
    class PGLearner : public CoeffsLearner
    {
        typedef std::list< std::vector<float> > PGbuffer;

        private:

        // Memory buffer for the PG algorithm
        PGbuffer coeffsBuffer;
        // Set of perturbations to be performed
        PGbuffer perturbationsBuffer;

        // Check for convergence of the algorithm
        bool converged();

        // Recursive perturbation generator
        void generatePerturbations(std::vector<float>* partial_perturbation, unsigned int index);

        public:

        // Default constructor
        PGLearner(DiveHandler* _dhPtr, int _nCoeffs, float _epsilon = EPSILON,
                  int _T = T, float _initValue = 0.0, bool randomize = false);

        // Generate a set of perturbations for the current policy
        void generatePerturbations();

        // Evaluate a single policy perturbation with the cost function
        float evaluatePerturbation( std::vector<float> R );

        // Update the PG parameters according to the obtained rewards
        void updateParams(const std::list<float>& rewards);

        // Update coefficients performing a step of the learning algorithm
        virtual bool updateCoeffs();

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
    std::list<float> rewardHistory;

    // Opponent team current score
    int opponentScore;
    // Flag enabled when a dive is performed
    bool dived;

    // Estimated time the ball needs to reach the goal
    // a.k.a. Tpapo (historical reasons)
    float tBall2Goal;
    // Estimated time needed for the current dive action to be performed
    float tDive;
    // Estimated time the goalie needs to back up to its original position
    float tBackInPose;

    // Estimated intersection between the ball projection and the goal line
    float ballProjectionIntercept;
    // Estimated distance of the ball from the own goal
    float distanceBall2Goal;

    // Computes parameters using the ball estimated position and velocity
    void estimateDiveTimes();
    void estimateBallProjection();

    // Compute the overall time the goalie needs to dive and then recover its position
    inline float computeDiveAndRecoverTime(float alpha1, float alpha2);
	
public:

    // Default constructor
    DiveHandler();
    // Destructor
    ~DiveHandler();

    // Update the DiveHandle for the goalie behavior
    void update(DiveHandle& diveHandle);
	
};
