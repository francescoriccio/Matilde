#include <StateMachineBehavior>
#include <cmath>
#include <Utils/Utils.h>

// Uncomment if you want to have debug information.
#define DEBUG_BEHAVIOR

#define MIN_DIST_TO_BALL 200

using namespace PTracking;
using namespace SPQR;

option AttackerPNP
{
private:
	
	bool isLeftFoot()
	{
		if(theBallModel.estimate.position.y > 0) return true;
		else return false;
	}
	
	float norm(float _x, float _y)
	{
		return sqrt(_x*_x + _y*_y);
	}
	
	float deg2rad( float deg)
	{
		return (deg*3.14)/180;
	}
	
	bool alignedTowardGoal()
	{
		Vector2 <float> goal_position = (theFreePartOfOpponentGoalModel.leftEnd + theFreePartOfOpponentGoalModel.rightEnd) / 2;
		double goal_angle = atan2(goal_position.y,goal_position.x);
		
		if (fabs( goal_angle ) < deg2rad(3)) return true;
		else return false;
	}

#ifdef DEBUG_BEHAVIOR
	Timestamp lastTimeRoleSent;
	std::string stateName;
#endif
	
	Vector2 <float> position;
	double angle;

public:
	
	common()
	{
		action
		{
#ifdef DEBUG_BEHAVIOR 
		  if ((Timestamp() - lastTimeRoleSent).getMs() > 1000.0)
		  {
			  cerr << "\033[22;34;1mAttackerPNP State: " << stateName << " \033[0m" << endl;
				
			  lastTimeRoleSent.setToNow();
		  }
#endif
		}
	}

	state FindBall()
	{
		decision
		{
#ifdef DEBUG_BEHAVIOR
			stateName = "FindBall";
#endif
			
			if( theBallPercept.ballWasSeen ) return ApproachBall;

		}
		action
		{
			theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
			theMotionRequest.walkRequest.target.translation.x = 0;
			theMotionRequest.walkRequest.target.translation.y = 0;
                        theMotionRequest.walkRequest.target.rotation = theBallModel.estimate.velocity.y < 0 ? -M_PI : M_PI;
			theMotionRequest.walkRequest.speed.translation.x = 0;
			theMotionRequest.walkRequest.speed.translation.y = 0;
			theMotionRequest.walkRequest.speed.rotation = 1;
			theMotionRequest.motion = MotionRequest::walk;

			LookUpAndDown();
		}
	}

	state LookYourFeet()
	{
		decision
		{
#ifdef DEBUG_BEHAVIOR
			stateName = "LookYourFeet";
#endif
			
			if (theFrameInfo.time - theBallModel.timeWhenLastSeen > 2000) return FindBall;
			
			if( norm(theBallModel.estimate.position.x, theBallModel.estimate.position.y) > MIN_DIST_TO_BALL ) return ApproachBall;
			else if( !alignedTowardGoal() ) return MoveAroundBall;
			else return Kick;
			
		}
		action
		{
			Stand();
			LookAtBall();
		}
	}

	state AlignTowardGoal()
	{
		decision
		{
#ifdef DEBUG_BEHAVIOR
			stateName = "AlignTowardGoal";
#endif
			if (theFrameInfo.time - theBallModel.timeWhenLastSeen > 2000) return FindBall;
			
			if( norm(theBallModel.estimate.position.x, theBallModel.estimate.position.y) > MIN_DIST_TO_BALL ) return ApproachBall;
			else
			{
				if( alignedTowardGoal() && fabs(theBallModel.estimate.position.y) < 70) return Kick;
			}
		}
		action
		{
			position = (theFreePartOfOpponentGoalModel.leftEnd + theFreePartOfOpponentGoalModel.rightEnd) / 2;
			angle = atan2(position.y,position.x);
			
			theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
			theMotionRequest.walkRequest.target.translation.x = theBallModel.estimate.position.x - 150;
			theMotionRequest.walkRequest.target.translation.y = isLeftFoot() ? theBallModel.estimate.position.y - 50 : theBallModel.estimate.position.y + 50;
			theMotionRequest.walkRequest.target.rotation = angle;
			theMotionRequest.walkRequest.speed.translation.x = SPQR::SPEED_X / 2.5;
			theMotionRequest.walkRequest.speed.translation.y = SPQR::SPEED_Y / 2.5;
			theMotionRequest.walkRequest.speed.rotation = 0.5;
			theMotionRequest.motion = MotionRequest::walk;
			
			LookAtBall();
		}
	}

	state Kick()
	{
		decision
		{
#ifdef DEBUG_BEHAVIOR
			stateName = "Kick";
#endif
			
                        if (theFrameInfo.time - theBallModel.timeWhenLastSeen > 1000) return FindBall;

                        if (stateTime > 2000) return ApproachBall;
                        else if( norm(theBallModel.estimate.position.x, theBallModel.estimate.position.y) < MIN_DIST_TO_BALL ) return LookYourFeet;
		}
		action
		{
			if (isLeftFoot()) KickForwardLeft();
			else KickForwardRight();
		}
	}

	state ApproachBall()
	{
		decision
		{
#ifdef DEBUG_BEHAVIOR
			stateName = "ApproachBall";
#endif
			
                        if (theFrameInfo.time - theBallModel.timeWhenLastSeen > 1000) return FindBall;
			
			if( norm(theBallModel.estimate.position.x, theBallModel.estimate.position.y) < MIN_DIST_TO_BALL ) return LookYourFeet;

		}
		action
		{
                    theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
                    theMotionRequest.walkRequest.target.translation.x = theBallModel.estimate.position.x;

                    if( isLeftFoot() )
                    {
                        theMotionRequest.walkRequest.target.translation.y = theBallModel.estimate.position.y - 50.0;
                        theMotionRequest.walkRequest.target.rotation = atan2(theBallModel.estimate.position.y - 50.0,theBallModel.estimate.position.x);
                    }
                    else
                    {
                        theMotionRequest.walkRequest.target.translation.y = theBallModel.estimate.position.y + 50.0;
                        theMotionRequest.walkRequest.target.rotation = atan2(theBallModel.estimate.position.y + 50.0,theBallModel.estimate.position.x);
                    }

//                    theMotionRequest.walkRequest.target.translation.y = theBallModel.estimate.position.y;

                    theMotionRequest.walkRequest.target.rotation = atan2(theBallModel.estimate.position.y,theBallModel.estimate.position.x);
                    theMotionRequest.walkRequest.speed.translation.x = SPQR::SPEED_X;
                    theMotionRequest.walkRequest.speed.translation.y = SPQR::SPEED_Y;
                    theMotionRequest.walkRequest.speed.rotation = 1;
                    theMotionRequest.motion = MotionRequest::walk;

                    LookAtBall();
		}
	}

	state MoveAroundBall()
	{
		decision
		{
#ifdef DEBUG_BEHAVIOR
			stateName = "MoveAroundBall";
#endif
                        if (theFrameInfo.time - theBallModel.timeWhenLastSeen > 2000) return FindBall;
			
			if( norm(theBallModel.estimate.position.x, theBallModel.estimate.position.y) > MIN_DIST_TO_BALL ) return ApproachBall;
			else 
			{
				if( alignedTowardGoal() ) return Kick;
				else if(fabs(angle) < Utils::deg2rad(TURN_VALID_THS) &&
					fabs(theBallModel.estimate.position.y) < 100 && 
					fabs(theBallModel.estimate.position.x) < 350) return AlignTowardGoal;
			}
		}
		action
		{
			position = (theFreePartOfOpponentGoalModel.leftEnd + theFreePartOfOpponentGoalModel.rightEnd) / 2;
			angle = atan2(position.y,position.x);
			
			theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
			theMotionRequest.walkRequest.target.translation.x = theBallModel.estimate.position.x - MIN_DIST_TO_BALL;
			theMotionRequest.walkRequest.target.translation.y = theBallModel.estimate.position.y;
			theMotionRequest.walkRequest.target.rotation = ceil(angle/abs(angle))*(fabs(angle) > PTracking::Utils::deg2rad(15) ? PTracking::Utils::deg2rad(15) : fabs(angle));
			theMotionRequest.walkRequest.speed.translation.x = SPQR::SPEED_X;
			theMotionRequest.walkRequest.speed.translation.y = SPQR::SPEED_Y;
			theMotionRequest.walkRequest.speed.rotation = 0.5;
			theMotionRequest.motion = MotionRequest::walk;
					
			LookAtBall();
		}
	}

}; //end option AttackerPNP
