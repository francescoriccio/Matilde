#pragma once

#include "Tools/Math/Vector2.h"

class DiveHandle : public Streamable
{
private:
	/** Streaming function
		* @param in  streaming in ...
		* @param out ... streaming out.
	*/
	void serialize(In* in, Out* out)
	{
		STREAM_REGISTER_BEGIN;
                STREAM(diveTime);
		STREAM(ballProjectionEstimate);
		STREAM(diveType);
		STREAM_REGISTER_FINISH;
	}

public:
	enum Dive
	{ 
		none = 1, 
		lDive, 
		rDive, 
		lcloseDive, 
		rcloseDive
	};
	
	typedef int Dive;
	
        float diveTime;
		float ballProjectionEstimate;
		Dive diveType;
	
	/** Constructor */
        DiveHandle() : diveTime(-1.0), ballProjectionEstimate(0.0), diveType(none) {;}
};
