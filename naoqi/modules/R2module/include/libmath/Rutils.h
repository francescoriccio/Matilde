/**
* @namespace: Rmath
* A bunch of mathematical functions and utilities to perform kinematics computation.
* Implementation based on Eigen 3.2.0 libraries.
*
* @file Rutils.cpp
* @author Claudio Delli Bovi, Chiara Picardi, Francesco Riccio
*/

#ifndef R_MATH
#define R_MATH

#include <vector>
#include <Eigen/Core>

// Numerical tolerance
#define TOLL 10e-6
#define CROP_TO_TOLERANCE

// Denavit-Hartenberg table row arrangement
#define A_POS 0
#define ALPHA_POS 1
#define D_POS 2
#define THETA_POS 3

#define TASK_SPACE_DIM 6

namespace Rmath{

class Transform;

/** -------- Core tools -------- */

/*
 * Trim matrix values according to a chosen tolerance.
*/
void trim(Eigen::VectorXd* v, double toll = TOLL);
inline void trim(Eigen::MatrixXd* m, double toll = TOLL);
inline void trim(Eigen::Matrix4d* m, double toll = TOLL);
inline void trim(Eigen::Matrix3d* m, double toll = TOLL);

/*
 * Generate a skew-matrix out of a given Vector3d.
*/
Eigen::Matrix3d Skew(Eigen::Vector3d v);

/*
 * Compute a rotation around the x-axis of 'a' radians.
 * The result is stored in 'mx'.
 */
void Rx(double a, Eigen::Matrix3d* mx);
Eigen::Matrix3d rotX(double a);

/*
 * Compute a rotation around the y-axis of 'a' radians.
 * The result is stored in 'my'.
 */
void Ry(double a, Eigen::Matrix3d* my);
Eigen::Matrix3d rotY(double a);

/*
 * Compute a rotation around the z-axis of 'a' radians.
 * The result is stored in 'mz'.
 */
void Rz(double a, Eigen::Matrix3d* mz);
Eigen::Matrix3d rotZ(double a);

/*
 * Compute a rotation matrix of an angle 'a' about an axis 'r'.
 * The result is stored in 'm'.
*/
void R(double a, const Eigen::Vector3d& r, Eigen::Matrix3d* m);
Eigen::Matrix3d rot(double a, const Eigen::Vector3d& r);

/*
 * Given a rotation matrix 'R' compute a minimal description of 'R' according to Euler angles formalism.
 * The set of angles is returned as a column vector [ PHI THETA PSI ]', i.e. the sequence of fixed-frame X-Y-Z rotations
 * corresponding to 'R'.
 * Out of singularities the inverse Euler problems yields two solutions of opposite sign for THETA: only the positive
 * one is returned by convention. In case of singularities a zero-vector is returned.
 */
Eigen::Vector3d xyzEulerAngles(const Eigen::Matrix3d& R);

/*
 * 'Homogenize' a 3x3 matrix by:
 *  - adding 1 in the last diagonal entry and 0 elsewhere;
 *  - adding 1 in the last diagonal entry, a translation vector and 0 elsewhere.
 * The result is stored in 'hm'.
*/
void homogenize(const Eigen::Matrix3d& m, Eigen::Matrix4d* hm);
void homogenize(const Eigen::Matrix3d& m, const Eigen::Vector3d& v, Eigen::Matrix4d* hm);

/*
 * Compute the 4x4 homogeneous transformation composed of:
 *  - a rotation around the x-axis of 'a' radians;
 *  - a translation along the vector 'shift'.
 * The result is stored in 'mx'.
 */
void Hx(double a, const Eigen::Vector3d& shift, Eigen::Matrix4d* mx);
Eigen::Matrix4d homX(double a, const Eigen::Vector3d& shift);

/*
 * Compute the 4x4 homogeneous transformation composed of:
 *  - a rotation around the y-axis of 'a' radians;
 *  - a translation along the vector 'shift'.
 * The result is stored in 'mx'.
 */
void Hy(double a, const Eigen::Vector3d& shift, Eigen::Matrix4d* my);
Eigen::Matrix4d homY(double a, const Eigen::Vector3d& shift);

/*
 * Compute the 4x4 homogeneous transformation composed of:
 *  - a rotation around the z-axis of 'a' radians;
 *  - a translation along the vector 'shift'.
 * The result is stored in 'mx'.
 */
void Hz(double a, const Eigen::Vector3d& shift, Eigen::Matrix4d* mz);
Eigen::Matrix4d homZ(double a, const Eigen::Vector3d& shift);

Eigen::Matrix4d hTranslation(const Eigen::Vector3d& shift);

/*
 * Compute the Moore-Penrose pseudo-inversion of matrix 'm' using SVD decomposition.
 * Optional value is the default tolerance for the singular values of 'm'.
 * The result is stored in 'm_pinv'.
 */
void pseudoInverse(const Eigen::MatrixXd& m, Eigen::MatrixXd* m_pinv, double toll = TOLL);

/*
 * Compute a Damped Least Squares (DLS) inversion of matrix 'm'.
 * Optional value is the damping factor 'lambda' (default is 0.0, i.e. far from singular conditions).
 * The result is stored in 'm_dls'.
 */
void DLSInverse(const Eigen::MatrixXd& m, Eigen::MatrixXd* m_dls, double lambda = 0.0);

/** -------- Kinematics -------- */

/*
 * Given the manipulator structure via Denavit-Hartenberg table of parameters 'dh'
 * (with the following order: a-alpha-d-theta), compute the kinematic chain and build up
 * the related homogeneous matrices from the base frame to each joint.
 * The resulting homogeneous transformation for the whole chain is stored in 'H'.
 * NOTE: in the following implementation only revolute joints are considered!
 */
void DirectKin(const Eigen::MatrixXd& dh, Eigen::Matrix4d* H);
void DirectKin(const std::vector<Rmath::Transform*>& chain, Eigen::Matrix4d* H);

/*
* Given the manipulator structure via Denavit-Hartenberg table of parameters 'dh'
* (with the following order: a-alpha-d-theta), compute the geometric Jacobian matrix
* for the whole chain. The result is stored in 'J'.
* Optional argument is the task space dimension (i.e. J's row size).
* NOTE: in the following implementation only revolute joints are considered!
*/
void geomJacobian(const Eigen::MatrixXd& dh, Eigen::MatrixXd* J, int taskSpaceDim = TASK_SPACE_DIM);
void geomJacobian(const std::vector<Rmath::Transform*>& chain, const std::vector<int>& i_joints, Eigen::MatrixXd* J, int taskSpaceDim = TASK_SPACE_DIM);


}

#endif
