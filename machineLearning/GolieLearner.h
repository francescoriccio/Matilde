#include <StateMachineBehavior>
#include <cmath>

// Uncomment if you want to have debug information
#define GOALIE_DEBUG_MODE

enum Dive
{
    none = 1,
    lDive,
    rDive,
    lcloseDive,
    rcloseDive
};

option GoalieLearner
{
private:

    inline Vector2<float> convertRel2Glob(float x, float y)
    {
        Vector2 <float> result;
        float rho = sqrt((x * x) + (y * y));

        result.x = theRobotPoseSpqrFiltered.x + (rho * cos(theRobotPoseSpqrFiltered.theta + atan2(y,x)));
        result.y = theRobotPoseSpqrFiltered.y + (rho * sin(theRobotPoseSpqrFiltered.theta + atan2(y,x)));

        return result;
    }

    inline Vector2<float> convertGlob2Rel(float x, float y)
    {
        Vector2 <float> result;

        float tempX = x - theRobotPoseSpqrFiltered.x;
        float tempY = y - theRobotPoseSpqrFiltered.y;

        result.x = tempX * cos(theRobotPoseSpqrFiltered.theta) + tempY * sin(theRobotPoseSpqrFiltered.theta);
        result.y = -tempX * sin(theRobotPoseSpqrFiltered.theta) + tempY * cos(theRobotPoseSpqrFiltered.theta);

        return result;
    }

    inline bool ballIsInRange(bool withError = false)
    {
        float deltaX = convertRel2Glob(theBallModel.estimate.position.x, theBallModel.estimate.position.y).x - GOALIE_BASE_POSITION_X;
        float deltaY = convertRel2Glob(theBallModel.estimate.position.x, theBallModel.estimate.position.y).y - GOALIE_BASE_POSITION_Y;
        if (withError)
            return sqrt(deltaX * deltaX + deltaY * deltaY) < SPQR::GOALIE_MAX_DIST_BALL_IN_RANGE_ABS + SPQR::GOALIE_POSE_X_TOLLERANCE;
        else
            return sqrt(deltaX * deltaX + deltaY * deltaY) < SPQR::GOALIE_MAX_DIST_BALL_IN_RANGE_ABS;
    }

    inline bool ballIsSeen()
    {
        return (theFrameInfo.time - theBallModel.timeWhenLastSeen) < SPQR::GOALIE_MIN_TIME_WHEN_LAST_SEEN;
    }

    bool isGoalieReady()
    {
        if( abs(theRobotPoseSpqrFiltered.theta) < PTracking::Utils::deg2rad(SPQR::GOALIE_POSE_ANGLE_TOLLERANCE) &&
                abs(theRobotPoseSpqrFiltered.x - SPQR::GOALIE_BASE_POSITION_X) < SPQR::GOALIE_POSE_X_TOLLERANCE &&
                abs(theRobotPoseSpqrFiltered.y - SPQR::GOALIE_BASE_POSITION_Y) < SPQR::GOALIE_POSE_Y_TOLLERANCE )
            return true;
        else return false;
    }

    bool isPositioned()
    {
        if( abs(theRobotPoseSpqrFiltered.x - SPQR::GOALIE_BASE_POSITION_X) < SPQR::GOALIE_POSE_X_TOLLERANCE &&
                abs(theRobotPoseSpqrFiltered.y - SPQR::GOALIE_BASE_POSITION_Y) < SPQR::GOALIE_POSE_Y_TOLLERANCE )
            return true;
        else return false;
    }

#ifdef GOALIE_DEBUG_MODE
    Timestamp lastTimeRoleSent;
    std::string stateName;
#endif

    enum Dive
    {
        none = 1,
        lDive,
        rDive,
        lcloseDive,
        rcloseDive
    };

public:

    common()
    {
        action
        {
#ifdef GOALIE_DEBUG_MODE
            if ((Timestamp() - lastTimeRoleSent).getMs() > 1000.0)
            {
                cerr << "\033[22;34;1mGoalie State: " << stateName << " \033[0m" << endl;
                cerr << "\033[22;34;1mGoalie Positioned: " << isGoalieReady() << " \033[0m" << endl;
                cerr << "\033[22;34;1mGoalie Position: (" << theRobotPoseSpqrFiltered.x << ", " << theRobotPoseSpqrFiltered.y << ", " << PTracking::Utils::rad2deg(theRobotPoseSpqrFiltered.theta) << ")" << " \033[0m" << endl;
                lastTimeRoleSent.setToNow();
            }
#endif
        }
    }

    state start()
    {
        decision
        {
#ifdef GOALIE_DEBUG_MODE
            stateName = "start";
#endif
            if(stateTime > 100)
            {
                return main_loop;
            }
        }
        action
        {
            Stand();
            LookUp();
        }
    }

    state main_loop()
    {
        decision
        {
#ifdef GOALIE_DEBUG_MODE
            stateName = "main_loop";
#endif      

            if( ballIsInRange(false) && theBallModel.estimate.velocity.abs() < SPQR::GOALIE_MOVING_BALL_MIN_VELOCITY ) return  kick_ball_away;
            else if( !isGoalieReady() ) return goTo_golie_position;

            else if(ballIsSeen())
            {
                //if ball is moving fast enought
                if(theBallModel.estimate.velocity.abs() >= SPQR::GOALIE_MOVING_BALL_MIN_VELOCITY)
                {
                    if(((theBallModel.estimate.velocity.angle() >  M_PI/2) && (theBallModel.estimate.velocity.angle() <  M_PI)) || //ball direction toward goal line
                            ((theBallModel.estimate.velocity.angle() > -M_PI) && (theBallModel.estimate.velocity.angle() < -M_PI/2)))
                    {
                        if( theDiveHandle.ballProjectionEstimate < SPQR::GOALIE_FAR_LIMIT_Y &&
                                theDiveHandle.ballProjectionEstimate > -SPQR::GOALIE_FAR_LIMIT_Y && //if the ball is directed in the goal mirror
                                theBallModel.estimate.getEndPosition(theFieldDimensions.ballFriction).x < 0) //and his velocity is enought to overcome it
                        {
                            if( theDiveHandle.diveTime == 0.0 ) // best timing provided by the diveHandler
                            {
                                if(theDiveHandle.diveType == lDive) return dive_left;
                                else if(theDiveHandle.diveType == rDive) return dive_right;
//                                else if(theDiveHandle.diveType == lcloseDive) return close_dive_left;
//                                else if(theDiveHandle.diveType == rcloseDive) return close_dive_right;
                                else return stop_ball; // or nothing
                            }
                        }
                    }
                }
            }
        }

        action
        {
            GoalieSitDown();
            (theFrameInfo.time - theBallModel.timeWhenLastSeen) < 2000 ? LookAtBall() : GoalieSearchForLandmarks();
        }
    }

    state dive_left()
    {
        decision
        {
#ifdef GOALIE_DEBUG_MODE
            stateName = "dive_left";
#endif
            if( stateTime > SPQR::GOALIE_DIVE_TIME )return goTo_golie_position;
        }
        action
        {
            GoalieSaveLeft();
            LookAtBall();
        }
    }

    state dive_right()
    {
        decision
        {
#ifdef GOALIE_DEBUG_MODE
            stateName = "dive_right";
#endif
            if( stateTime > SPQR::GOALIE_DIVE_TIME ) return goTo_golie_position;
        }
        action
        {
            GoalieSaveRight();
            LookAtBall();
        }
    }

    state stop_ball()
    {
        decision
        {
#ifdef GOALIE_DEBUG_MODE
            stateName = "stop_ball";
#endif
            if( stateTime > SPQR::GOALIE_DIVE_TIME ) return main_loop;
        }
        action
        {
            StopBall();
            LookAtBall();
        }
    }

    state kick_ball_away(){

        decision
        {
#ifdef GOALIE_DEBUG_MODE
            stateName = "kick_ball_away";
#endif
            if( !ballIsInRange(true) || !ballIsSeen() ) return goTo_golie_position;
        }

        action
        {
            GoalieKickAway();
            LookAtBall();
        }
    }

    state goTo_golie_position()
    {
        decision
        {
#ifdef GOALIE_DEBUG_MODE
            stateName = "goTo_golie_position";
#endif
            if( ballIsInRange() ) return kick_ball_away;

            if( isGoalieReady() ) return main_loop;
            else if( isPositioned() ) return turnTo_opponent_goal;
        }
        action
        {
            Vector2 <float> goliePosition = convertGlob2Rel(SPQR::GOALIE_BASE_POSITION_X, SPQR::GOALIE_BASE_POSITION_Y);

            theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
            theMotionRequest.walkRequest.target.translation.x = goliePosition.x;
            theMotionRequest.walkRequest.target.translation.y = goliePosition.y;
            theMotionRequest.walkRequest.target.rotation = atan2(goliePosition.y, goliePosition.x);

            theMotionRequest.walkRequest.speed.translation.x = SPQR::SPEED_X;
            theMotionRequest.walkRequest.speed.translation.y = SPQR::SPEED_Y;
            theMotionRequest.walkRequest.speed.rotation = 1;
            theMotionRequest.motion = MotionRequest::walk;

            LookAtBall();
        }
    }

    state turnTo_opponent_goal()
    {
        decision
        {
#ifdef GOALIE_DEBUG_MODE
            stateName = "turnTo_opponent_goal";
#endif
            if( ballIsInRange() ) return kick_ball_away;

            if( isGoalieReady() ) return main_loop;
        }
        action
        {
            Vector2 <float> fieldCenter = convertGlob2Rel(0.0, 0.0);

            theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
            theMotionRequest.walkRequest.target.translation.x = 0;
            theMotionRequest.walkRequest.target.translation.y = 0;
            theMotionRequest.walkRequest.target.rotation = atan2(fieldCenter.y,fieldCenter.x);

            theMotionRequest.walkRequest.speed.translation.x = 0;
            theMotionRequest.walkRequest.speed.translation.y = 0;
            theMotionRequest.walkRequest.speed.rotation = 1;
            theMotionRequest.motion = MotionRequest::walk;

            LookUpAndDown();
        }
    }

};
