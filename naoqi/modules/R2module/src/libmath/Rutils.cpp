/**
* @namespace: Rmath
* A bunch of mathematical functions and utilities to perform kinematics computation.
* Implementation based on Eigen 3.2.0 libraries.
*
* @file libmath.cpp
* @author Claudio Delli Bovi, Chiara Picardi, Francesco Riccio
*/

#include <cmath>
#include <vector>
#include <assert.h>
#include <iostream>

#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigen>

#include "libmath/Rutils.h"
#include "libmath/transform.h"

#define INFO(x) std::cout << "[Rmath] " << x << std::endl;

/** -------- Core tools -------- */


/*
 * Trim matrix values according to a chosen tolerance.
*/
void Rmath::trim(Eigen::VectorXd* v, double toll)
{
    for(int j=0; j < v->size(); ++j){
        // Zeroing any value smaller than the predefined tolerance
        if( std::abs((*v)(j)) <= toll ) (*v)(j) = 0.0;
    }
}

void Rmath::trim(Eigen::MatrixXd* m, double toll)
{
    for(int i=0; i < m->rows(); ++i){
        for(int j=0; j < m->cols(); ++j){
            // Zeroing any value smaller than the predefined tolerance
            if( std::abs((*m)(i,j)) <= toll ) (*m)(i,j) = 0.0;
        }
    }

}

void Rmath::trim(Eigen::Matrix4d* m, double toll)
{
    for(int i=0; i < 4; ++i){
        for(int j=0; j < 4; ++j){
            // Zeroing any value smaller than the predefined tolerance
            if( std::abs((*m)(i,j)) <= toll ) (*m)(i,j) = 0.0;
        }
    }

}

void Rmath::trim(Eigen::Matrix3d* m, double toll)
{
    for(int i=0; i < 3; ++i){
        for(int j=0; j < 3; ++j){
            // Zeroing any value smaller than the predefined tolerance
            if( std::abs((*m)(i,j)) <= toll ) (*m)(i,j) = 0.0;
        }
    }

}

/*
 * Generate a skew-matrix out of a given Vector3d.
*/
Eigen::Matrix3d Rmath::Skew(Eigen::Vector3d v)
{
    Eigen::Matrix3d sk;
    // Using Eigen comma initializer
    sk <<       0,    -v(2),    v(1),
             v(2),        0,   -v(0),
            -v(1),     v(0),       0;

  return sk;
}

/*
 * Compute a rotation around the x-axis of 'a' radians.
 * The result is stored in 'mx'.
 */
void Rmath::Rx(double a, Eigen::Matrix3d* mx)
{
    assert(mx != NULL);
    // Using Eigen comma initializer
    (*mx) << 1,          0,         0,
             0,     cos(a),   -sin(a),
             0,     sin(a),    cos(a);

#ifdef CROP_TO_TOLERANCE
    // Trim according to default numerical tolerance
    trim(mx);
#endif
}

Eigen::Matrix3d Rmath::rotX(double a)
{
    Eigen::Matrix3d rx;
    // Using Eigen comma initializer
    rx  <<   1,          0,         0,
             0,     cos(a),   -sin(a),
             0,     sin(a),    cos(a);

#ifdef CROP_TO_TOLERANCE
    // Trim according to default numerical tolerance
    trim(&rx);
#endif

    return rx;
}

/*
 * Compute a rotation around the y-axis of 'a' radians.
 * The result is stored in 'my'.
 */
void Rmath::Ry(double a, Eigen::Matrix3d* my)
{
    assert(my != NULL);
    // Using Eigen comma initializer
    (*my) << cos(a),     0,    sin(a),
                  0,     1,         0,
            -sin(a),     0,    cos(a);

#ifdef CROP_TO_TOLERANCE
    // Trim according to default numerical tolerance
    trim(my);
#endif
}

Eigen::Matrix3d Rmath::rotY(double a)
{
    Eigen::Matrix3d ry;
    // Using Eigen comma initializer
    ry  <<   cos(a),     0,    sin(a),
                  0,     1,         0,
            -sin(a),     0,    cos(a);

#ifdef CROP_TO_TOLERANCE
    // Trim according to default numerical tolerance
    trim(&ry);
#endif

    return ry;
}

/*
 * Compute a rotation around the z-axis of 'a' radians.
 * The result is stored in 'mz'.
 */
void Rmath::Rz(double a, Eigen::Matrix3d* mz)
{
    assert(mz != NULL);
    // Using Eigen comma initializer
    (*mz) << cos(a), -sin(a),       0,
             sin(a),  cos(a),       0,
             0,            0,       1;

#ifdef CROP_TO_TOLERANCE
    // Trim according to default numerical tolerance
    trim(mz);
#endif
}

Eigen::Matrix3d Rmath::rotZ(double a)
{
    Eigen::Matrix3d rz;
    // Using Eigen comma initializer
    rz  <<   cos(a), -sin(a),       0,
             sin(a),  cos(a),       0,
             0,            0,       1;

#ifdef CROP_TO_TOLERANCE
    // Trim according to default numerical tolerance
    trim(&rz);
#endif

    return rz;
}

/*
 * Compute a rotation matrix of an angle 'a' about an axis 'r'.
 * The result is stored in 'm'.
*/
void Rmath::R(double a, const Eigen::Vector3d& r, Eigen::Matrix3d* m)
{
    assert(m != NULL);
    // Using Eigen comma initializer
    (*m) << ((1-cos(a))*r(0)*r(0)+cos(a)),          ((1-cos(a))*r(0)*r(1)-r(2)*sin(a)),    ((1-cos(a))*r(0)*r(2)+r(1)*sin(a)),
            ((1-cos(a))*r(0)*r(1)+r(2)*sin(a)),     ((1-cos(a))*r(1)*r(1)+cos(a)),         ((1-cos(a))*r(1)*r(2)-r(0)*sin(a)),
            ((1-cos(a))*r(0)*r(2)-r(1)*sin(a)),     ((1-cos(a))*r(1)*r(2)+r(0)*sin(a)),    ((1-cos(a))*r(2)*r(2)+cos(a));

#ifdef CROP_TO_TOLERANCE
    // Trim according to default numerical tolerance
    trim(m);
#endif
}

Eigen::Matrix3d Rmath::rot(double a, const Eigen::Vector3d& r)
{
    Eigen::Matrix3d r_aa;
    // Using Eigen comma initializer
    r_aa << ((1-cos(a))*r(0)*r(0)+cos(a)),          ((1-cos(a))*r(0)*r(1)-r(2)*sin(a)),    ((1-cos(a))*r(0)*r(2)+r(1)*sin(a)),
            ((1-cos(a))*r(0)*r(1)+r(2)*sin(a)),     ((1-cos(a))*r(1)*r(1)+cos(a)),         ((1-cos(a))*r(1)*r(2)-r(0)*sin(a)),
            ((1-cos(a))*r(0)*r(2)-r(1)*sin(a)),     ((1-cos(a))*r(1)*r(2)+r(0)*sin(a)),    ((1-cos(a))*r(2)*r(2)+cos(a));

#ifdef CROP_TO_TOLERANCE
    // Trim according to default numerical tolerance
    trim(&r_aa);
#endif

    return r_aa;
}

/*
 * Given a rotation matrix 'R' compute a minimal description of 'R' according to Euler angles formalism.
 * The set of angles is returned as a column vector [ PHI THETA PSI ]', i.e. the sequence of fixed-frame X-Y-Z rotations
 * corresponding to 'R'.
 * Out of singularities the inverse Euler problems yields two solutions of opposite sign for THETA: only the positive
 * one is returned by convention. In case of singularities a zero-vector is returned.
 */
Eigen::Vector3d Rmath::xyzEulerAngles(const Eigen::Matrix3d& R)
{
    Eigen::Vector3d EulerOrientation;
    double phi=0, theta=0, psi=0;

    // For fixed-frame XYZ rotations, sin(THETA) = -R(3,1)
    theta = atan2(-R(2,0), sqrt( R(2,1)*R(2,1) + R(2,2)*R(2,2) ));

    // Singularities for THETA = M_PI/2 or THETA = -M_PI/2
    if( std::abs(theta)-M_PI/2 < TOLL )
        EulerOrientation = Eigen::Vector3d::Zero();
    else
    {
        // Compute PHI and PSI as functions of THETA
        psi = atan2(R(2,1)/cos(theta), R(2,2)/cos(theta));
        phi = atan2(R(1,0)/cos(theta), R(0,0)/cos(theta));

        // Minimal orientation description: [ PHI THETA PSI ]'
        EulerOrientation << phi, theta, psi;
    }

    return EulerOrientation;
}

/*
 * 'Homogenize' a 3x3 matrix by:
 *  - adding 1 in the last diagonal entry and 0 elsewhere;
 *  - adding 1 in the last diagonal entry, a translation vector and 0 elsewhere.
 * The result is stored in 'hm'.
*/
void Rmath::homogenize(const Eigen::Matrix3d& m, Eigen::Matrix4d* hm)
{
    assert(hm != NULL);
    // Using Eigen comma initializer
    (*hm) <<    m,                          Eigen::Vector3d::Zero(),
                Eigen::RowVector3d::Zero(),                       1;
}

void Rmath::homogenize(const Eigen::Matrix3d& m, const Eigen::Vector3d& v, Eigen::Matrix4d* hm)
{
    assert(hm != NULL);
    // Using Eigen comma initializer
    (*hm) <<    m,                              v,
                Eigen::RowVector3d::Zero(),     1;
}

/*
 * Compute the 4x4 homogeneous transformation composed of:
 *  - a rotation around the x-axis of 'a' radians;
 *  - a translation along the vector 'shift'.
 * The result is stored in 'mx'.
 */
void Rmath::Hx(double a, const Eigen::Vector3d& shift, Eigen::Matrix4d* mx)
{
    assert(mx != NULL);
    // Using Eigen comma initializer
    (*mx) << 1,          0,         0,  shift(0),
             0,     cos(a),   -sin(a),  shift(1),
             0,     sin(a),    cos(a),  shift(2),
             0,          0,         0,         1;

#ifdef CROP_TO_TOLERANCE
    // Trim according to default numerical tolerance
    trim(mx);
#endif
}

Eigen::Matrix4d Rmath::homX(double a, const Eigen::Vector3d& shift)
{
    Eigen::Matrix4d hx;
    // Using Eigen comma initializer
    hx  <<   1,          0,         0,  shift(0),
             0,     cos(a),   -sin(a),  shift(1),
             0,     sin(a),    cos(a),  shift(2),
             0,          0,         0,         1;

#ifdef CROP_TO_TOLERANCE
    // Trim according to default numerical tolerance
    trim(&hx);
#endif

    return hx;
}

/*
 * Compute the 4x4 homogeneous transformation composed of:
 *  - a rotation around the y-axis of 'a' radians;
 *  - a translation along the vector 'shift'.
 * The result is stored in 'mx'.
 */
void Rmath::Hy(double a, const Eigen::Vector3d& shift, Eigen::Matrix4d* my)
{
    assert(my != NULL);
    // Using Eigen comma initializer
    (*my) << cos(a),     0,    sin(a),  shift(0),
                  0,     1,         0,  shift(1),
            -sin(a),     0,    cos(a),  shift(2),
                  0,     0,         0,         1;

#ifdef CROP_TO_TOLERANCE
    // Trim according to default numerical tolerance
    trim(my);
#endif
}

Eigen::Matrix4d Rmath::homY(double a, const Eigen::Vector3d& shift)
{
    Eigen::Matrix4d hy;
    // Using Eigen comma initializer
    hy  <<   cos(a),     0,    sin(a),  shift(0),
                  0,     1,         0,  shift(1),
            -sin(a),     0,    cos(a),  shift(2),
                  0,     0,         0,         1;

#ifdef CROP_TO_TOLERANCE
    // Trim according to default numerical tolerance
    trim(&hy);
#endif

    return hy;
}

/*
 * Compute the 4x4 homogeneous transformation composed of:
 *  - a rotation around the z-axis of 'a' radians;
 *  - a translation along the vector 'shift'.
 * The result is stored in 'mx'.
 */
void Rmath::Hz(double a, const Eigen::Vector3d& shift, Eigen::Matrix4d* mz)
{
    assert(mz != NULL);
    // Using Eigen comma initializer
    (*mz) << cos(a), -sin(a),       0,  shift(0),
             sin(a),  cos(a),       0,  shift(1),
             0,            0,       1,  shift(2),
             0,            0,       0,         1;

#ifdef CROP_TO_TOLERANCE
    // Trim according to default numerical tolerance
    trim(mz);
#endif
}

Eigen::Matrix4d Rmath::homZ(double a, const Eigen::Vector3d& shift)
{
    Eigen::Matrix4d hz;
    // Using Eigen comma initializer
    hz  <<   cos(a), -sin(a),       0,  shift(0),
             sin(a),  cos(a),       0,  shift(1),
             0,            0,       1,  shift(2),
             0,            0,       0,         1;

#ifdef CROP_TO_TOLERANCE
    // Trim according to default numerical tolerance
    trim(&hz);
#endif

    return hz;
}

/*
 * Compute the Moore-Penrose pseudo-inversion of matrix 'm' using SVD decomposition.
 * Optional value is the default tolerance for the singular values of 'm'.
 * The result is stored in 'm_pinv'.
 */
void Rmath::pseudoInverse(const Eigen::MatrixXd& m, Eigen::MatrixXd* m_pinv, double toll)
{
    // 'Large' rectangular matrix
    if (m.rows() <= m.cols())
    {
        // SVD decomposition of 'm'
        Eigen::JacobiSVD<Eigen::MatrixXd> m_svd (m, Eigen::ComputeThinU | Eigen::ComputeThinV);
        // Extract singular values
        Eigen::VectorXd SigmaDiag = m_svd.singularValues();

        // Switch each singular value with its reciprocal to compute the pseudo-inverse of Sigma
        for(int i = 0; i < SigmaDiag.size(); ++i){
            // Avoid ill-conditioning by zeroing singular values below tolerance
            if(abs(SigmaDiag(i)) < TOLL) SigmaDiag(i) = 0;
            else SigmaDiag(i) = 1.0/SigmaDiag(i);
        }

        // Pseudo-inverse of Sigma
        Eigen::MatrixXd SigmaCross = SigmaDiag.asDiagonal();
        SigmaCross.transposeInPlace();

        // U and V eigenvectors matrices
        Eigen::MatrixXd V = m_svd.matrixV();
        Eigen::MatrixXd U = m_svd.matrixU();

        // Pseudo-inverse of 'm'
       (*m_pinv) = V * SigmaCross * (U.adjoint());
    }
    // 'Narrow' rectangular matrix
    else
    {
        // Do the computation for 'm' transpose and then transpose back
        pseudoInverse(m.transpose(), m_pinv, toll);
        m_pinv->transposeInPlace();
    }

#ifdef CROP_TO_TOLERANCE
    // Trim according to default numerical tolerance
    trim(m_pinv);
#endif
}

/*
 * Compute a Damped Least Squares (DLS) inversion of matrix 'm'.
 * Optional value is the damping factor 'lambda' (default is 0.0, i.e. far from singular conditions).
 * The result is stored in 'm_dls'.
 */
void Rmath::DLSInverse(const Eigen::MatrixXd& m, Eigen::MatrixXd* m_dls, double lambda)
{
    // A positive damping factor is required
    assert(lambda >= 0.0);

    // lambda*I + m * m'
    Eigen::MatrixXd lambdaI_JJ(m.rows(), m.rows());
    lambdaI_JJ = Eigen::MatrixXd::Identity(m.rows(), m.rows())*lambda + m * m.transpose();

    // DLS inverse
    (*m_dls) = m.transpose() * lambdaI_JJ.inverse();

#ifdef CROP_TO_TOLERANCE
    // Trim according to default numerical tolerance
    trim(m_dls);
#endif
}

/** -------- Kinematics -------- */


/*
 * Given the manipulator structure via Denavit-Hartenberg table of parameters 'dh'
 * (with the following order: a-alpha-d-theta), compute the kinematic chain and build up
 * the related homogeneous matrices from the base frame to each joint.
 * The resulting homogeneous transformation for the whole chain is stored in 'H'.
 * NOTE: in the following implementation only revolute joints are considered!
 */
void Rmath::DirectKin(const Eigen::MatrixXd& dh, Eigen::Matrix4d* H)
{
    assert(H != NULL);
    // An n-by-4 Denavit-Hartenberg table is required
    assert(dh.cols() == 4);

    // Initialization
    (*H) = Eigen::Matrix4d::Identity();

    // Go through the D-H table row-by-row, compute i-to-i+1 transformation and post-multiply
    for (int i=0; i < dh.rows(); ++i)
    {
        Eigen::Matrix4d currentHz,currentHx;
        Eigen::Vector3d currentZshift, currentXshift;

        // Translation vectors
        currentZshift <<              0,     0,     dh(i, D_POS);
        currentXshift <<    dh(i,A_POS),     0,                0;

        // i-to-i+1 roto-translation on the Z and X axis
        Hz(dh(i, THETA_POS), currentZshift, &currentHz);
        Hx(dh(i, ALPHA_POS), currentXshift, &currentHx);

        // Post-multiply the result by the current transformation
        (*H) *= currentHz * currentHx;
    }

#ifdef CROP_TO_TOLERANCE
    // Trim according to default numerical tolerance
    trim(H);
#endif
}

void Rmath::DirectKin(const std::vector<Rmath::Transform*>& chain, Eigen::Matrix4d* H)
{
    assert(H != NULL);
    // Initialization
    (*H) = Eigen::Matrix4d::Identity();

    // Go through the chain, compute i-to-i+1 transformation and post-multiply
    for (unsigned int i=0; i < chain.size(); ++i) (*H) *= chain.at(i)->transform();

#ifdef CROP_TO_TOLERANCE
    // Trim according to default numerical tolerance
    trim(H);
#endif
}

/*
* Given the manipulator structure via Denavit-Hartenberg table of parameters 'dh'
* (with the following order: a-alpha-d-theta) or an explicit chain of transformations,
* compute the geometric Jacobian matrix for the whole chain. The result is stored in 'J'.
* Optional argument is the task space dimension (i.e. J's row size).
* NOTE: in the following implementation only revolute joints are considered!
*/
void Rmath::geomJacobian(const Eigen::MatrixXd& dh, Eigen::MatrixXd* J, int taskSpaceDim)
{
    assert(J != NULL);
    // An n-by-4 Denavit-Hartenberg table is required
    assert(dh.cols() == 4);
    // Task space is 6-dimensional at most (i.e. pose in 3D space)
    assert(taskSpaceDim <= 6);

    // Initialization of J as a zero-matrix
    (*J) = Eigen::MatrixXd::Zero(taskSpaceDim, dh.rows());

    // Default Z-axis in the base frame [ 0 , 0 , 1 ]'
    Eigen::Vector3d z = Eigen::Vector3d::UnitZ();

    // Compute forward kinematics (base-EE homogeneous transformation)
    Eigen::Matrix4d H_ee;
    DirectKin(dh, &H_ee);

    // Base-EE translation vector
    Eigen::Vector3d d_0e = H_ee.topRightCorner(3, 1);

    // Jacobian leftmost column
    Eigen::VectorXd J0(6,1); J0 << z.cross(d_0e), z;
    // Crop to required task space dimension
    J->col(0) = J0.head(taskSpaceDim);

    // Go through the D-H table row-by-row and build up J column-wise
    for (int i=1; i < (*J).cols(); ++i)
    {
        // Compute forward kinematics up to joint i
        Eigen::Matrix4d currentH;
        DirectKin(dh.topLeftCorner(i, dh.cols()), &currentH);

        // Extract current Z-axis in the reference frame of joint i
        Eigen::Vector3d currentZ = currentH.topLeftCorner(3,3) * z;
        // Extract base-to-joint-i translation vector
        Eigen::Vector3d d_0i = currentH.topRightCorner(3,1);

        // Fill Jacobian current column
        Eigen::VectorXd currentJcol(6,1); currentJcol << currentZ.cross(d_0e - d_0i), currentZ;
        // Crop to required task space dimension
        J->col(i) = currentJcol.head(taskSpaceDim);
    }

#ifdef CROP_TO_TOLERANCE
    // Trim according to default numerical tolerance
    trim(J);
#endif

}

void Rmath::geomJacobian(const std::vector<Rmath::Transform*>& chain, const std::vector<int>& i_joints,
                         Eigen::MatrixXd* J, int taskSpaceDim)
{
    assert(J != NULL);
    // Task space is 6-dimensional at most (i.e. pose in 3D space)
    assert(taskSpaceDim <= 6);

    // Initialization of J as a zero-matrix
    (*J) = Eigen::MatrixXd::Zero(taskSpaceDim, i_joints.size());

    // Default Z-axis in the base frame [ 0 , 0 , 1 ]'
    Eigen::Vector3d z = Eigen::Vector3d::UnitZ();

    // Compute forward kinematics (base-EE homogeneous transformation)
    Eigen::Matrix4d H_ee;
    DirectKin(chain, &H_ee);
    // Base-EE translation vector
    Eigen::Vector3d d_0e = H_ee.topRightCorner(3, 1);

    // Jacobian leftmost column
    Eigen::VectorXd J0(6,1); J0 << z.cross(d_0e), z;
    // Crop to required task space dimension
    J->col(0) = J0.head(taskSpaceDim);

    // Go through the chain of transforms and build up J column-wise
    for (int i=1; i < i_joints.size(); ++i)
    {
        // Consider the subchain of transformation leading to joint i-1
        Eigen::Matrix4d currentH;
        std::vector<Rmath::Transform*> currentSubchain; //(chain.begin(), chain.begin()+i_joints.at(i-1));
        for (int j=0; j <= i_joints.at(i)-1; ++j)
            currentSubchain.push_back( chain.at(j) );
        // Compute forward kinematics up to joint i-1
        DirectKin(currentSubchain, &currentH);

        // Extract current Z-axis in the reference frame of joint i-1
        Eigen::Vector3d currentZ = currentH.topLeftCorner(3,3) * z;
        // Extract base-to-joint-i translation vector
        Eigen::Vector3d d_0i = currentH.topRightCorner(3,1);

        // Fill Jacobian current column
        Eigen::VectorXd currentJcol(6,1); currentJcol << currentZ.cross(d_0e - d_0i), currentZ;
        // Crop to required task space dimension
        J->col(i) = currentJcol.head(taskSpaceDim);
    }

#ifdef CROP_TO_TOLERANCE
    // Trim according to default numerical tolerance
    trim(J);
#endif

}
