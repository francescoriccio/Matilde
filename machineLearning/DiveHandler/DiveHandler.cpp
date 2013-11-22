

#include "DiveHandler.h"
#include <Utils/Utils.h>
#include <stdlib.h>
#include <time.h>

#define DEBUG_MODE

#define SPQR_ERR(x) std::cerr << "\033[22;31;1m" <<"[DiveHandler] " << x << "\033[0m"<< std::endl;
#define SPQR_INFO(x) std::cerr << "\033[22;34;1m" <<"[DiveHandler] " << x << "\033[0m" << std::endl;

MAKE_MODULE(DiveHandler, SPQR-Modules)

void DiveHandler::CoeffsLearner::setCoeffs(std::vector< float > _coeffs)
{
    coeffs = _coeffs;
}

void DiveHandler::CoeffsLearner::setParam(std::string _key, float _value)
{
    std::map<std::string, float>::iterator iter = params.begin();
    params.insert( iter, std::pair< std::string, float >(_key, _value) );
}

DiveHandler::PGLearner::PGLearner( int _nCoeffs, float _epsilon, int _T, float _initValue, bool randomize ):CoeffsLearner(_nCoeffs, _initValue)
{
    if(randomize)
    {
        srand(time(NULL));
        for( int i=0; i<_nCoeffs; ++i) coeffs.at(i) = (rand()/RAND_MAX)*_initValue;
    }

    setParam("epsilon", _epsilon);
    setParam("T", _T);
}

void DiveHandler::PGLearner::generatePerturbations()
{
    // store perturbations
}

float DiveHandler::PGLearner::evaluatePerturbation( std::vector<float> R )
{
    // evaluate perturbations
    return 0.0;
}

bool DiveHandler::PGLearner::updateCoeffs()
{
    // while stop criterion: MAX_ITER || alpha1 alpha2 converge
    return false;
}

DiveHandler::DiveHandler():
    diveType(none), tBall2Goal(SPQR::FIELD_DIMENSION_Y), tDive(0.0), tBackInPose(0.0), learner(new PGLearner(2, EPSILON, T, 1.0)){}

DiveHandler::~DiveHandler()
{
    delete learner;
}


void DiveHandler::estimateBallProjection()
{
    //if(((theBallModel.estimate.velocity.angle() >  M_PI/2) && (theBallModel.estimate.velocity.angle() <  M_PI)) || //ball direction toward goal line
    //((theBallModel.estimate.velocity.angle() > -M_PI) && (theBallModel.estimate.velocity.angle() < -M_PI/2)))
    {
        //ball path line
        float A1 = (theBallModel.estimate.position.y - theBallModel.estimate.velocity.y) - theBallModel.estimate.position.y;
        float B1 = theBallModel.estimate.position.x - (theBallModel.estimate.position.x - theBallModel.estimate.velocity.x);
        float C1 = A1*theBallModel.estimate.position.x + B1*theBallModel.estimate.position.y;

        //goal line
        float A2 = SPQR::GOALIE_FAR_LIMIT_Y - -SPQR::GOALIE_FAR_LIMIT_Y;

        //cross product or determinant
        float det = - A2*B1;

        float yIntercept = SPQR::FIELD_DIMENSION_Y;

        //lines are not parallel
        if( det > SPQR::GOALIE_EPSILON_COLLINEAR || det < -SPQR::GOALIE_EPSILON_COLLINEAR ) // TODO ask Gull
        {
            yIntercept = (- A2*C1) / det;

            if( yIntercept > ( SPQR::GOALIE_CLOSE_LIMIT_Y/2) && yIntercept < SPQR::GOALIE_FAR_LIMIT_Y ) diveType = lcloseDive;
            else if( yIntercept > SPQR::GOALIE_FAR_LIMIT_Y ) diveType = lDive;

            else if( yIntercept < (-SPQR::GOALIE_CLOSE_LIMIT_Y/2) && yIntercept > -SPQR::GOALIE_FAR_LIMIT_Y ) diveType = rcloseDive;
            else if( yIntercept < -SPQR::GOALIE_FAR_LIMIT_Y ) diveType = rDive;

            else diveType = none;
        }

        if (diveType == lDive || diveType == rDive ) tDive = SPQR::GOALIE_DIVE_TIME;
        else if (diveType == lcloseDive || diveType == rcloseDive ) tDive = SPQR::GOALIE_CLOSE_DIVE_TIME;
        else tDive = 0.0;

        ballProjectionIntercept = yIntercept;
    }
}

void DiveHandler::estimateDiveTimes()
{
    float delta_x = -SPQR::FIELD_DIMENSION_X - theGlobalBallEstimation.singleRobotX;
    float delta_y = ballProjectionIntercept - theGlobalBallEstimation.singleRobotY;

    float distanceBall2Goal = sqrt( delta_x*delta_x + delta_y*delta_y);

    if ( (theBallModel.estimate.velocity.abs() != 0.0)  && (theBallModel.estimate.velocity.x < 0.0) )
        tBall2Goal = 1000.0 * ( distanceBall2Goal / theBallModel.estimate.velocity.abs() );
    else
        tBall2Goal = -1.0;

    float tRecover = 0.0;
    float tReposition = 0.0;
    if( diveType == rcloseDive || diveType == lcloseDive ) tRecover = SPQR::GOALIE_CLOSE_DIVE_RECOVER_TIME;
    else if( diveType == rDive || diveType == lDive )
    {
        tRecover = SPQR::GOALIE_DIVE_RECOVER_TIME;
        tReposition = SPQR::GOALIE_DIVE_REPOSITION_TIME;
    }

    tBackInPose = tRecover + tReposition;
}

void DiveHandler::update(DiveHandle& diveHandle)
{
    if (theRobotInfo.number == 1)
    {
        estimateBallProjection();
        diveHandle.ballProjectionEstimate = ballProjectionIntercept;

        if( theGlobalBallEstimation.singleRobotX < 0.0 && fabs(ballProjectionIntercept) < SPQR::FIELD_DIMENSION_Y )
        {
            estimateDiveTimes();

#ifdef DEBUG_MODE
            SPQR_INFO("Ball projection: " << ballProjectionIntercept);
            SPQR_INFO("PAPO time: " << tBall2Goal);
            SPQR_INFO("Dive time: " << tDive);
            SPQR_INFO("Back-in-pose time: " << tBackInPose);
#endif

            if( state == learning )
            {
                // iterazione while PG

                // state = waitReward;
            }
            else if( state == waitReward )
            {
                // polling for reward

                // if reward -> state = learning
            }

            if( state == learning )
            {
                // adjust PG parameters wrt reward
            }

            // compute dive Time using current coefficients
            float diveTime = (learner->getCoeffs()).at(1) * ( (learner->getCoeffs()).at(0) * tBall2Goal - tDive );
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
        else
        {
            diveHandle.diveTime = -1;
            diveHandle.diveType = diveType;
        }
    }

}
