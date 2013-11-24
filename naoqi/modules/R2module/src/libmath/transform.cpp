/**
* @class: Transform
* @namespace: Rmath
* This class (and related hierarchy) describes a basic rigid transformation.
*
* @file transform.cpp
* @author Claudio Delli Bovi, Chiara Picardi, Francesco Riccio
*/

#include <cmath>

#include "libmath/transform.h"


/** -------- DHtransform -------- */


/*
 * Class copy constructor.
 */
Rmath::DHtransform::DHtransform(const Rmath::DHtransform& Tdh): Transform(Rmath::dh_transform)
{
    // Denavit-Hartenberg parameters initialization
    this->a = Tdh.Xshift();
    this->alpha = Tdh.Xangle();
    this->d = Tdh.Zshift();
    this->theta = Tdh.Zangle();
}

/*
 * Retrieve the Denavit-Hartenberg parameters as map<char,double>:
 * - 'a' is the shift along the X axis (i.e. along the common normal);
 * - 'h' is the twist angle (i.e. the angle between the joint axes);
 * - 'd' is shift along the Z axis (i.e. the joint value in the prismatic joint case);
 * - 't' is the angle around the Z axis (i.e. the joint value in the revolute joint case);
 */
void Rmath::DHtransform::parameters(std::map<char, double>* params)
{
    // Common normal
    (*params)['a'] = a;
    // Twist angle
    (*params)['h'] = alpha;
    // Z-shift (prismatic joint value)
    (*params)['d'] = d;
    // Z-angle (revolute joint value)
    (*params)['t'] = theta;
}

/*
 * Retrieve the D-H transform in homogeneous matrix form.
 * The transformation matrix is obtained by concatenating a roto-translation along the Z axis
 * in the frame of joint i-1, and a roto-translation along the X axis in the intermediate frame.
 */
Eigen::Matrix4d Rmath::DHtransform::transform() const
{
    return Rmath::homZ(theta, Eigen::Vector3d::UnitZ()*d) *
           Rmath::homX(alpha, Eigen::Vector3d::UnitX()*a);
}

/*
 * Print method and stream operator overload: they just print the DH parameters in a row.
 */
void Rmath::DHtransform::print(std::ostream& out) const
{
    out << "[ " << a << ", " << alpha << ", "
      << d << ", "<< theta << " ]" << std::endl;
}

std::ostream& Rmath::operator<<(std::ostream& out, const Rmath::DHtransform& dh)
{
    dh.print(out);
}


/** -------- Translation -------- */

/*
 * Class copy constructor.
 */
Rmath::Translation::Translation(const Rmath::Translation& Ttranslation): Transform(Rmath::translation)
{
    this->values << Ttranslation.asVector()(0), Ttranslation.asVector()(1), Ttranslation.asVector()(2);
}

/*
 * Retrieve the translation parameters as map<char,double>:
 * - 'x' is the first component of the translation vector;
 * - 'y' is the second component of the translation vector;
 * - 'z' is the third component of the translation vector;
 */
void Rmath::Translation::parameters(std::map<char, double>* params)
{
    (*params)['x'] = values(0);
    (*params)['y'] = values(1);
    (*params)['z'] = values(2);
}

/*
 * Retrieve the rigid translation as a homogeneous matrix with a null rotation.
 */
Eigen::Matrix4d Rmath::Translation::transform() const
{
    return Rmath::homZ(0.0, values);
}

/*
 * Print method and stream operator overload: they just print the translation vector.
 */
void Rmath::Translation::print(std::ostream& out) const
{
    out << "[ " << values(0) << ", " << values(1)
        << ", " << values(2) << " ]" << std::endl;
}

std::ostream& Rmath::operator<<(std::ostream& out, const Rmath::Translation& tr)
{
    tr.print(out);
}


/** -------- Rotation -------- */


/*
 * Class copy constructor.
 */
Rmath::Rotation::Rotation(const Rmath::Rotation& Trotation): Transform(Rmath::rotation)
{
    this->axis << Trotation.getAxis()(0), Trotation.getAxis()(1), Trotation.getAxis()(2);
    this->angle = Trotation.getAngle();
}

/*
 * Retrieve the rotation parameters as map<char,double>:
 * - 'x' is the first component of the rotation axis;
 * - 'y' is the second component of the rotation axis;
 * - 'z' is the third component of the rotation axis;
 * - 'a' is the angle value (in radians);
 */
void Rmath::Rotation::parameters(std::map<char, double>* params)
{
    (*params)['x'] = axis(0);
    (*params)['y'] = axis(1);
    (*params)['z'] = axis(2);
    (*params)['a'] = angle;
}

/*
 * Retrieve the rigid translation as a homogeneous matrix with a null translation.
 */
Eigen::Matrix4d Rmath::Rotation::transform() const
{
    Eigen::Matrix4d transf;
    Rmath::homogenize(Rmath::rot(angle,axis), &transf);
    return transf;
}

/*
 * Print method and stream operator overload: they just print the rotation axis and the rotation angle in a row.
 */
void Rmath::Rotation::print(std::ostream& out) const
{
    out << "[ " << axis(0) << ", " << axis(1)
        << ", " << axis(2) << ", " << angle << " ]" << std::endl;
}

std::ostream& Rmath::operator<<(std::ostream& out, const Rmath::Rotation& rt)
{
    rt.print(out);
}

