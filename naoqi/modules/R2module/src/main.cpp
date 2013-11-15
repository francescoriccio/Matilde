#include <iostream>
#include <sstream>

#include <boost/assign/std/vector.hpp>

#include <soth/debug.hpp>
#include <soth/HCOD.hpp>
#include <soth/Random.hpp>
#include <soth/DestructiveColPivQR.hpp>

#include "libmath/Rutils.h"
#include "libmath/transform.h"

#define LIBMATH_TEST
//#define SOTH_TEST

using namespace std;
using namespace boost::assign;
using namespace Rmath;
//using namespace soth;

#define NB_STAGE 1
#define NC 2

int main(){

#ifdef LIBMATH_TEST
    std::cout << "[HQP_example] Testing 'libmath' library... " << std::endl;

    /** --------- Testing homogeneous and rotation matrices --------- */

    Eigen::Matrix3d r1, r_aa;
    Rz(M_PI/2, &r1); R(M_PI/2, Eigen::Vector3d::UnitZ(), &r_aa);
    std::cout << "[HQP_example] Rotation around Z-axis: " << std::endl;
    std::cout << r1 << std::endl << std::endl;
    std::cout << r_aa << std::endl << std::endl;

    Eigen::Matrix3d r2, r_aa2;
    Ry(M_PI/2, &r2); R(M_PI/2, Eigen::Vector3d::UnitY(), &r_aa2);
    std::cout << "[HQP_example] Rotation around Y-axis: " << std::endl;
    std::cout << r2 << std::endl << std::endl;
    std::cout << r_aa2 << std::endl << std::endl;

    Eigen::Matrix3d r3, r_aa3;
    Rx(M_PI/2, &r3); R(M_PI/2, Eigen::Vector3d::UnitX(), &r_aa3);
    std::cout << "[HQP_example] Rotation around X-axis: " << std::endl;
    std::cout << r3 << std::endl << std::endl;
    std::cout << r_aa3 << std::endl << std::endl;

    Eigen::Matrix4d h3, hh3;
    homogenize(r3, &h3);
    std::cout << "[HQP_example] Homogenizing last matrix (no shift): " << std::endl;
    std::cout << h3 << std::endl;
    homogenize(r3, Eigen::Vector3d::Ones(), &hh3);
    std::cout << "[HQP_example] Homogenizing last matrix (shif = [ 1  1  1 ]''): " << std::endl;
    std::cout << hh3 << std::endl;


    /** --------- Testing forward kinematics and Jacobian --------- */

    Eigen::MatrixXd dh_leftleg (6,4);
        dh_leftleg << 45.19,        0,      0,       0,
                      0,        M_PI/2,     0,       0,
                      102.9,        0,      0,       0,
                      100,     -M_PI/2,     0,       0,
                      0,        M_PI/2,     0,       0,
                      0,            0,      0,       0;

        std::vector<Transform*> Nao_leftleg;
        Nao_leftleg.push_back(new DHtransform(45.19, 0.0, 0.0, 0.0));
        Nao_leftleg.push_back(new DHtransform(0.0, M_PI/2, 0.0, 0.0));
        Nao_leftleg.push_back(new DHtransform(102.9, 0.0, 0.0, 0.0));
        Nao_leftleg.push_back(new DHtransform(100, -M_PI/2, 0.0, 0.0));
        Nao_leftleg.push_back(new DHtransform(0.0, M_PI/2, 0.0, 0.0));
        Nao_leftleg.push_back(new DHtransform(0.0, 0.0, 0.0, 0.0));

        std::vector<int> joint_i;
        joint_i.push_back(0);
        joint_i.push_back(1);
        joint_i.push_back(2);
        joint_i.push_back(3);
        joint_i.push_back(4);
        joint_i.push_back(5);

        std::cout << "[HQP_example] NAO left leg D-H table: " << std::endl;
        std::cout << dh_leftleg << std::endl;

        Eigen::Matrix4d H_leftleg;
        DirectKin(dh_leftleg, &H_leftleg);
        std::cout << "[HQP_example] Left leg forward kinematics: " << std::endl;
        Eigen::Matrix4d Ty, Tz;
        Hy(M_PI/2, Eigen::Vector3d::Zero(), &Ty);
        Hz(M_PI, Eigen::Vector3d::Zero(), &Tz);
        H_leftleg = Ty*Tz*H_leftleg;
        trim(&H_leftleg);
        std::cout << H_leftleg << std::endl;
        //std::cout << (H_leftleg).inverse() << std::endl;

        Eigen::MatrixXd J_leftleg;
        geomJacobian(dh_leftleg, &J_leftleg);
        std::cout << "[HQP_example] Left leg geometric Jacobian #1: " << std::endl;
        std::cout << J_leftleg << std::endl;

        Eigen::MatrixXd J_leftleg2;
        geomJacobian(Nao_leftleg, joint_i, &J_leftleg2);
        std::cout << "[HQP_example] Left leg geometric Jacobian #2: " << std::endl;
        std::cout << J_leftleg2 << std::endl;

        Eigen::MatrixXd J_pinv;
        pseudoInverse(J_leftleg, &J_pinv);
        std::cout << "[HQP_example] Pseudo-inverse of J: " << std::endl;
        std::cout << J_pinv << std::endl;

        Eigen::MatrixXd J_dls;
        DLSInverse(J_leftleg, &J_dls, 0.5);
        std::cout << "[HQP_example] DLS inverse of J (lambda = 0.5): " << std::endl;
        std::cout << J_dls << std::endl;


        /** --------- Testing class hierarchy Transform --------- */


        DHtransform H1(0.0, M_PI/2, 0.5, M_PI/4);
        std::cout << "[HQP_example] DH transform: " << std::endl;
        std::cout << H1 << std::endl;
        std::cout << "[HQP_example] DH transform matrix: " << std::endl;
        std::cout << H1.transform() << std::endl;
        std::cout << "[HQP_example] Changing d=1, theta=M_PI: " << std::endl;
        H1.changeZshift(1.0); H1.changeZangle(M_PI);
        std::cout << H1.transform() << std::endl;

        std::vector<Transform*> Nao_rightleg;
        Nao_rightleg.push_back(new Translation(0.0, -50.0, -85));
        Nao_rightleg.push_back(new Rotation(1.0, 0.0, 0.0, -M_PI/4));
        Nao_rightleg.push_back(new DHtransform(0.0, 0.0, 0.0, 0.0));
        Nao_rightleg.push_back(new Rotation(1.0, 0.0, 0.0, -M_PI/4));
        Nao_rightleg.push_back(new DHtransform(0.0, -M_PI/2, 0.0, -M_PI/2));
        Nao_rightleg.push_back(new DHtransform(-100.0, M_PI/2, 0.0, 0.0));

        std::vector<int> joint_indices;
        joint_indices.push_back(2);
        joint_indices.push_back(4);
        joint_indices.push_back(5);

        std::cout << "[HQP_example] Right leg forward kinematics: " << std::endl;
        Eigen::Matrix4d H_rl;
        DirectKin(Nao_rightleg, &H_rl);
        std::cout << H_rl << std::endl;

        std::cout << "[HQP_example] Right leg Jacobian: " << std::endl;
        Eigen::MatrixXd J_rl;
        geomJacobian(Nao_rightleg, joint_indices, &J_rl);
        std::cout << J_rl << std::endl;


#endif
#ifdef SOTH_TEST
        /*
         * From 'simple.cpp' reminder:
         *  NB_STAGE = Number of tasks (i.e. levels in the hierarchy)
         *  NC = Total number of rows in the 9problem (sum over all single row sizes)
         *  NR = std::vector containing single row sizes of each problem
        */

        std::cout << "[HQP_example] Testing 'soth' libraries..." << std::endl;

        std::vector<Eigen::MatrixXd> J;
        std::vector<soth::VectorBound> b;

        Eigen::MatrixXd J1(2,2), J2(2,2);
        J1 << 0.1, -1.0,
              1.0, -1.0;

        J2 << 1.0, 0.0,
              1.0, 1.0;

        J.push_back(J1);
        J.push_back(J2);

        soth::VectorBound b0(2), b1(2);
        b0[0] = soth::Bound(-0.55, soth::Bound::BOUND_SUP);
        b0[1] = soth::Bound( 1.5, soth::Bound::BOUND_SUP);

        b1[0] = soth::Bound(2.5, soth::Bound::BOUND_INF);
        b1[1] = soth::Bound(2, soth::Bound::BOUND_INF);

        b.push_back(b0);
        b.push_back(b1);
        soth::HCOD hsolver(NC, NB_STAGE);
        Eigen::VectorXd solution(NC);

        hsolver.pushBackStages(J, b);

        hsolver.setDamping(0.0);
        hsolver.setInitialActiveSet();

        for(int i=0;i<1000;++i){
            hsolver.initialize();
            hsolver.Y.computeExplicitly();
            hsolver.computeSolution();
            hsolver.showActiveSet(std::cout);
        }

        hsolver.activeSearch(solution);

            std::cout << "actset = "; hsolver.showActiveSet(std::cout);
            std::cout << "res = \n" << solution << std::endl;
#endif

    return 0;
}

