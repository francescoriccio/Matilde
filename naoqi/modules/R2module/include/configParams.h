#ifndef CONFIG_PARAMS
#define CONFIG_PARAMS

#include <Eigen/Core>

//#define DEBUG_MODE
//#define TASK_DEBUG
//#define TASK_MANAGER_DEBUG
#define LOG


// Discrete integration time step
static const double TIME_STEP = 1.0;


// Nao numerical values
#define FOOT_HEIGHT 45.19

#define HIP_HEIGHT 202.9
#define HIP_PELVIS_SHIFT 50.0
#define HIP_TORSO 85.0

#define TORSO_NECK 126.5
#define TORSO_SHOULDER_Z 100.0
#define TORSO_SHOULDER_Y 98.0

#define ARM_LENGTH 218.0
#define WRIST_SHIFT_Z 12.31
#define ELBOW_SHIFT_Y 15.0


/** ---------------------------- Paths ---------------------------- */


// Config (.cfg) files directory and paths
static const std::string CONFIG_PATH = "/home/francesco/naoqi/modules/R2module/config/";
//static const std::string CONFIG_PATH = "/home/claudio/Naoqi/NaoModules/r2module/config/";
// Joint parameters
static const std::string JOINT_BOUNDS_CFG = "joints_params.cfg";
// Base kinematic chains
static const std::string LEFT_LEG_BASE_CFG = "dh_LLbase.cfg";
static const std::string RIGHT_LEG_BASE_CFG = "dh_RLbase.cfg";
// Kinematic chains
static const std::string LEFT_LEG_CFG = "dh_leftLeg.cfg";
static const std::string LEFT_ARM_CFG = "dh_leftArm.cfg";
static const std::string RIGHT_LEG_CFG = "dh_rightLeg.cfg";
static const std::string RIGHT_ARM_CFG = "dh_rightArm.cfg";
static const std::string HEAD_CFG = "dh_head.cfg";
// Fixed transformations
static const std::string COM_HEAD_CFG = "CoM_head.cfg";
static const std::string COM_RIGHT_ARM_CFG = "CoM_rightArm.cfg";
static const std::string COM_LEFT_ARM_CFG = "CoM_leftArm.cfg";
static const std::string COM_RIGHT_LEG_CFG = "CoM_rightLeg.cfg";
static const std::string COM_LEFT_LEG_CFG = "CoM_leftLeg.cfg";



/** ---------------------------- Joint parameters ---------------------------- */



// Head
static const int HEAD_CHAIN_BEGIN = 0;
static const int HEAD_CHAIN_SIZE = 2;
static const int HEAD_CHAIN_END = HEAD_CHAIN_BEGIN + HEAD_CHAIN_SIZE;
// Right arm
static const int RARM_CHAIN_BEGIN = HEAD_CHAIN_END;
static const int RARM_CHAIN_SIZE = 5;
static const int RARM_CHAIN_END = RARM_CHAIN_BEGIN + RARM_CHAIN_SIZE;
// Left arm
static const int LARM_CHAIN_BEGIN = RARM_CHAIN_END;
static const int LARM_CHAIN_SIZE = 5;
static const int LARM_CHAIN_END = LARM_CHAIN_BEGIN + LARM_CHAIN_SIZE;
// Right leg
static const int RLEG_CHAIN_BEGIN = LARM_CHAIN_END;
static const int RLEG_CHAIN_SIZE = 6;
static const int RLEG_CHAIN_END = RLEG_CHAIN_BEGIN + RLEG_CHAIN_SIZE;
// Left leg
static const int LLEG_CHAIN_BEGIN = RLEG_CHAIN_END;
static const int LLEG_CHAIN_SIZE = 6;
static const int LLEG_CHAIN_END = LLEG_CHAIN_BEGIN + LLEG_CHAIN_SIZE;

// Nao joints number
static const int JOINTS_NUM = HEAD_CHAIN_SIZE +
                              LARM_CHAIN_SIZE + RARM_CHAIN_SIZE +
                              LLEG_CHAIN_SIZE + RLEG_CHAIN_SIZE;

// Fixed transformation base-ankle frame
static const double BASE_ANKLE_X = 0.0;
static const double BASE_ANKLE_Y = 0.0;
static const double BASE_ANKLE_Z = FOOT_HEIGHT;
static Eigen::Matrix4d base_ankle;



/** ---------------------------- Task parameters ---------------------------- */

static const float ACTIVATION_STEP = 0.1;

// Task names
static const std::string HEAD_TASK = "Head task";
static const std::string RIGHT_ARM = "Right arm task";
static const std::string LEFT_ARM = "Left arm task";
static const std::string RIGHT_LEG = "Right leg task";
static const std::string LEFT_LEG = "Left leg task";

// Task dimensions
static const int HEAD_TASK_DIM = 6;
static const int RARM_TASK_DIM = 3;
static const int LARM_TASK_DIM = 3;
static const int RLEG_TASK_DIM = 6;
static const int LLEG_TASK_DIM = 6;

// Task priorities
static const int HEAD_TASK_PRIORITY = 5;
static const int RARM_TASK_PRIORITY = 3;
static const int LARM_TASK_PRIORITY = 3;
static const int RLEG_TASK_PRIORITY = 1;
static const int LLEG_TASK_PRIORITY = 1;

// Gains
static const double K_HEAD = 1.0;
static const double K_RARM = 1.0;
static const double K_LARM = 1.0;
static const double K_RLEG = 0.6;
static const double K_LLEG = 0.6;

// Control points number
static const int HEAD_TASK_NSTEPS = 1;
static const int RARM_TASK_NSTEPS = 15;
static const int LARM_TASK_NSTEPS = 30;
static const int RLEG_TASK_NSTEPS = 30;
static const int LLEG_TASK_NSTEPS = 30;

// Smooth transition step
static const double HEAD_TRANSITION_STEP = 1.0;
static const double RARM_TRANSITION_STEP = 1.0;
static const double LARM_TRANSITION_STEP = 1.0;
static const double RLEG_TRANSITION_STEP = 1.0;
static const double LLEG_TRANSITION_STEP = 1.0;

#define RARM_LARM_JOINT_TASK
enum TaskType {
    MIRROR_TASK,
    MIMIC_TASK
};
static const TaskType ARMS_TASK = MIRROR_TASK;
static const double MINIMUM_HANDS_DISTANCE = 100.0;

// Circle trajectory task parameters
#define LARM_CIRCLE_TASK
//#define RARM_CIRCLE_TASK
static const double CIRCLE_Z_DEPTH = -70.0;         // wrt ee-frame
static const double CIRCLE_RADIUS = 80.0;
static const int CIRCLE_LAPS = 10;

// Up-down with legs task
//#define UP_DOWN_TASK
static int UP_DOWN = 80.0;


/** ------------------------------- Desired poses ------------------------------- */


static Eigen::VectorXd desiredHeadPose(6);
static Eigen::VectorXd desiredRHandPose(6);
static Eigen::VectorXd desiredLHandPose(6);
static Eigen::VectorXd desiredRLegPose(6);
static Eigen::VectorXd desiredLLegPose(6);

//Head
static const double HEAD_DESIRED_X = 0.0;   // [mm]
static const double HEAD_DESIRED_Y = 0.0;
static const double HEAD_DESIRED_Z = FOOT_HEIGHT+HIP_HEIGHT+HIP_TORSO+TORSO_NECK-9.9;
static const double HEAD_DESIRED_ROLL = 0.0;
static const double HEAD_DESIRED_PITCH = 0.0;
static const double HEAD_DESIRED_YAW = 0.0;
//Right arm
static const double RARM_DESIRED_X = ARM_LENGTH;
static const double RARM_DESIRED_Y = -HIP_PELVIS_SHIFT-TORSO_SHOULDER_Y-ELBOW_SHIFT_Y;
static const double RARM_DESIRED_Z = FOOT_HEIGHT+HIP_HEIGHT+HIP_TORSO+TORSO_SHOULDER_Z+WRIST_SHIFT_Z-9.9;
static const double RARM_DESIRED_ROLL = 0.0;
static const double RARM_DESIRED_PITCH = 0.0;
static const double RARM_DESIRED_YAW = 0.0;
// Left arm
static const double LARM_DESIRED_X = ARM_LENGTH;
static const double LARM_DESIRED_Y = -HIP_PELVIS_SHIFT+TORSO_SHOULDER_Y+ELBOW_SHIFT_Y;
static const double LARM_DESIRED_Z = FOOT_HEIGHT+HIP_HEIGHT+HIP_TORSO+TORSO_SHOULDER_Z+WRIST_SHIFT_Z-9.9;
static const double LARM_DESIRED_ROLL = 0.0;
static const double LARM_DESIRED_PITCH = 0.0;
static const double LARM_DESIRED_YAW = 0.0;
// Right leg
static const double RLEG_DESIRED_X = 20.0;
static const double RLEG_DESIRED_Y = -HIP_PELVIS_SHIFT;
#ifndef UP_DOWN_TASK
static const double RLEG_DESIRED_Z = -HIP_HEIGHT-FOOT_HEIGHT-HIP_TORSO+9.9;
#else
static const double RLEG_DESIRED_Z = -HIP_HEIGHT-FOOT_HEIGHT-HIP_TORSO+UP_DOWN+9.9;
#endif
static const double RLEG_DESIRED_ROLL = 0.0;
static const double RLEG_DESIRED_PITCH = 0.0;
static const double RLEG_DESIRED_YAW = 0.0;
// Left leg
static const double LLEG_DESIRED_X = 20.0;
static const double LLEG_DESIRED_Y = HIP_PELVIS_SHIFT;
#ifndef UP_DOWN_TASK
static const double LLEG_DESIRED_Z = -HIP_HEIGHT-FOOT_HEIGHT-HIP_TORSO+9.9;
#else
static const double LLEG_DESIRED_Z = -HIP_HEIGHT-FOOT_HEIGHT-HIP_TORSO+UP_DOWN+9.9;
#endif
static const double LLEG_DESIRED_ROLL = 0.0;
static const double LLEG_DESIRED_PITCH = 0.0;
static const double LLEG_DESIRED_YAW = 0.0;


static void initialization()
{
    // Initialize the fixed transformation base-ankle
    base_ankle << 0.0,      0.0,     1.0,    BASE_ANKLE_X,
                  0.0,     -1.0,     0.0,    BASE_ANKLE_Y,
                  1.0,      0.0,     0.0,    BASE_ANKLE_Z,
                  0.0,      0.0,     0.0,             1.0;

    desiredHeadPose << HEAD_DESIRED_X, HEAD_DESIRED_Y, HEAD_DESIRED_Z,
            HEAD_DESIRED_ROLL, HEAD_DESIRED_PITCH, HEAD_DESIRED_YAW;
    desiredRHandPose << RARM_DESIRED_X, RARM_DESIRED_Y, RARM_DESIRED_Z,
            RARM_DESIRED_ROLL, RARM_DESIRED_PITCH, RARM_DESIRED_YAW;
    desiredLHandPose << LARM_DESIRED_X, LARM_DESIRED_Y, LARM_DESIRED_Z,
            LARM_DESIRED_ROLL, LARM_DESIRED_PITCH, LARM_DESIRED_YAW;
    desiredRLegPose << RLEG_DESIRED_X, RLEG_DESIRED_Y, RLEG_DESIRED_Z,
            RLEG_DESIRED_ROLL, RLEG_DESIRED_PITCH, RLEG_DESIRED_YAW;
    desiredLLegPose << LLEG_DESIRED_X, LLEG_DESIRED_Y, LLEG_DESIRED_Z,
            LLEG_DESIRED_ROLL, LLEG_DESIRED_PITCH, LLEG_DESIRED_YAW;


    /*TOFIX*/
    assert(HEAD_TASK_DIM <=6);
    desiredHeadPose = desiredHeadPose.head(HEAD_TASK_DIM);
    assert(RARM_TASK_DIM <=6);
    desiredRHandPose = desiredRHandPose.head(RARM_TASK_DIM);
    assert(LARM_TASK_DIM <= 6);
    desiredLHandPose = desiredLHandPose.head(LARM_TASK_DIM);
    assert(RLEG_TASK_DIM <= 6);
//    desiredRLegPose = desiredRLegPose.head(RLEG_TASK_DIM);
    assert(LLEG_TASK_DIM <= 6);
//    desiredLLegPose = desiredLLegPose.head(LLEG_TASK_DIM);
}

#endif
