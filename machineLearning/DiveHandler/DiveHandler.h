#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/SPQR-Representations/RobotPoseSpqrFiltered.h"
#include "Representations/SPQR-Representations/DiveHandle.h"
#include "Representations/SPQR-Representations/GlobalBallEstimation.h"
#include <SPQR-Libraries/PTracking/src/Utils/AgentPacket.h>
#include <Representations/SPQR-Representations/ConfigurationParameters.h>
#include <Representations/Infrastructure/GameInfo.h>
#include <Representations/Infrastructure/RobotInfo.h>

#include<vector>

MODULE(DiveHandler)
	REQUIRES(GameInfo)
        REQUIRES(RobotInfo)
	REQUIRES(RobotPoseSpqrFiltered)
	REQUIRES(BallModel)
        REQUIRES(GlobalBallEstimation)
	PROVIDES(DiveHandle)
END_MODULE

#define EPSILON 0.5
#define T 5

#define MAX_ITER 30
#define BUFFER_DIM 10
#define CONVERGENCE_THRESHOLD 0.05
#define ETA 1

class DiveHandler : public DiveHandlerBase
{
	enum LearningState
	{ 
		notLearning = 1, 
		waitReward, 
		learning
	};
	
	enum Dive
	{ 
		none = 1, 
		lDive, 
		rDive, 
		lcloseDive, 
		rcloseDive
	};
	
	class CoeffsLearner
	{
	protected:
		std::vector<float> coeffs;
		std::map<std::string, float> params;
		
	public:
		CoeffsLearner(int _nCoeffs, float _initValue): coeffs(_nCoeffs, _initValue){}
		
		void setCoeffs(std::vector<float> _coeffs);
		inline std::vector<float> getCoeffs(){ return coeffs; }
		
		void setParam(std::string _key, float _value);
		inline float getParam(std::string _key){ return params[_key]; }
		
		virtual bool updateCoeffs() = 0;
	};
	
	class PGLearner : public CoeffsLearner
	{
		typedef std::list< std::vector<float> > PGbuffer;
		
	private:
		PGbuffer coeffsBuffer;
		std::vector<float> perturbations; 
		
		bool converged(){ return false; }
	public:
		
		PGLearner(int _nCoeffs, float _epsilon = EPSILON, int _T = T, float _initValue = 0.0, bool randomize = false);
		
		void generatePerturbations();
		float evaluatePerturbation( std::vector<float> R );
		
		virtual bool updateCoeffs();
	};
	
// 	class GALearner : public CoeffsLearner
// 	{};
	
private:
	Dive diveType;
	LearningState state;
	
	float tBall2Goal; // so called Tpapo for historical reasons
	float tDive;
	float tBackInPose;
	
	float ballProjectionIntercept;

        CoeffsLearner* learner;
	
	void estimateDiveTimes();
	void estimateBallProjection();
	
public:
	DiveHandler();
        ~DiveHandler();
	void update(DiveHandle& diveHandle);
	
};
