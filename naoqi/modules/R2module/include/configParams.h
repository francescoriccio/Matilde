#ifndef CONFIG_PARAMS
#define CONFIG_PARAMS

#include <Eigen/Core>

// CIRCLE:: n°circles, z_shift, radius

// Nao joints number
static const int JOINTS_NUM = 24;

// Discrete integration time step
static const double TIME_STEP = 1.0;

// Config (.cfg) files directory and paths
//static const std::string CONFIG_PATH = "/home/claudio/Naoqi/NaoModules/r2module/config/";
static const std::string CONFIG_PATH = "/home/francesco/naoqi/modules/R2module/config/";
static const std::string JOINT_BOUNDS_CFG = "joints_params.cfg";
static const std::string LEFT_LEG_CFG = "dh_leftLeg.cfg";
static const std::string LEFT_ARM_CFG = "dh_leftArm.cfg";
static const std::string RIGHT_LEG_CFG = "dh_rightLeg.cfg";
static const std::string RIGHT_ARM_CFG = "dh_rightArm.cfg";
static const std::string HEAD_CFG = "dh_head.cfg";


/** ---------------------- Head task parameters ---------------------- */


static const int HEAD_CHAIN_BEGIN = 0;
static const int HEAD_CHAIN_SIZE = 2;
static const int HEAD_CHAIN_END = HEAD_CHAIN_BEGIN + HEAD_CHAIN_SIZE;

static const int HEAD_TASK_DIM = 3;
static const int HEAD_TASK_PRIORITY = 3;
static const int K_HEAD = 1;
static const int HEAD_TASK_NSTEPS = 1;

static const double HEAD_DESIRED_X = 53.9;
static const double HEAD_DESIRED_Y = 0.0;
static const double HEAD_DESIRED_Z = 194.4;
static const double HEAD_DESIRED_ROLL = 0.0;
static const double HEAD_DESIRED_PITCH = 0.0;
static const double HEAD_DESIRED_YAW = 0.0;


/** -------------------- Right arm task parameters -------------------- */


static const int RARM_CHAIN_BEGIN = HEAD_CHAIN_END;
static const int RARM_CHAIN_SIZE = 5;
static const int RARM_CHAIN_END = RARM_CHAIN_BEGIN + RARM_CHAIN_SIZE;

static const double COM_RARM_SHIFT_X = 0.0;
static const double COM_RARM_SHIFT_Y = -98.0;
static const double COM_RARM_SHIFT_Z = 100.0;

static const int RARM_TASK_DIM = 3;
static const int RARM_TASK_PRIORITY = 1;
static const int K_RARM = 1;
static const int RARM_TASK_NSTEPS = 30;

static const double RARM_DESIRED_X = 250.0;
static const double RARM_DESIRED_Y = -113.0;
static const double RARM_DESIRED_Z = 120.0;
static const double RARM_DESIRED_ROLL = 0.0;
static const double RARM_DESIRED_PITCH = 0.0;
static const double RARM_DESIRED_YAW = 0.0;


#define RARM_LARM_JOINT_TASK
enum TaskType {
    MIRROR_TASK,
    MIMIC_TASK
};
static const TaskType ARMS_TASK = MIMIC_TASK;
static const double MINIMUM_HANDS_DISTANCE = 50.0;

/** -------------------- Left arm task parameters -------------------- */


static const int LARM_CHAIN_BEGIN = RARM_CHAIN_END;
static const int LARM_CHAIN_SIZE = 5;
static const int LARM_CHAIN_END = LARM_CHAIN_BEGIN + LARM_CHAIN_SIZE;

static const double COM_LARM_SHIFT_X = 0.0;
static const double COM_LARM_SHIFT_Y = 98.0;
static const double COM_LARM_SHIFT_Z = 100.0;

static const int LARM_TASK_DIM = 3;
static const int LARM_TASK_PRIORITY = 1;
static const int K_LARM = 1;
static const int LARM_TASK_NSTEPS = 30;

static const double LARM_DESIRED_X = 0.0;
static const double LARM_DESIRED_Y = 250 + 113.0;
static const double LARM_DESIRED_Z = 120.0;
static const double LARM_DESIRED_ROLL = 0.0;
static const double LARM_DESIRED_PITCH = 0.0;
static const double LARM_DESIRED_YAW = M_PI/2;


/** -------------------- Right leg task parameters -------------------- */


static const int RLEG_CHAIN_BEGIN = LARM_CHAIN_END;
static const int RLEG_CHAIN_SIZE = 6;
static const int RLEG_CHAIN_END = RLEG_CHAIN_BEGIN + RLEG_CHAIN_SIZE;


/** -------------------- Left leg task parameters -------------------- */


static const int LLEG_CHAIN_BEGIN = RLEG_CHAIN_END;
static const int LLEG_CHAIN_SIZE = 6;
static const int LLEG_CHAIN_END = LLEG_CHAIN_BEGIN + LLEG_CHAIN_SIZE;

static const int LLEG_TASK_DIM = 3;


#ifdef TEST_KINCHAIN
static Rmath::KinChain* theKinChain_TMP;
static Eigen::Matrix4d H_TMP;
static Eigen::MatrixXd J_TMP;
static Eigen::VectorXd q_TMP(LARM_CHAIN_SIZE);
static Eigen::Vector3d r;
#endif


#endif
