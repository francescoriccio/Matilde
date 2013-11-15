/**
* @class: Transform
* @namespace: Rmath
* This class and related hierarchy represents a basic rigid transformation
*
* @file transform.cpp
* @author Claudio Delli Bovi, Chiara Picardi, Francesco Riccio
*/

#include "libmath/transform.h"


/** -------- DHtransform -------- */

Rmath::DHtransform::DHtransform(const Rmath::DHtransform& Tdh): Transform(Rmath::dh_transform)
{
    this->a = Tdh.Xshift();
    this->alpha = Tdh.Xangle();
    this->d = Tdh.Zshift();
    this->theta = Tdh.Zangle();
}

void Rmath::DHtransform::parameters(std::map<char, double>* params)
{
    (*params)['a'] = a;
    (*params)['h'] = alpha;
    (*params)['d'] = d;
    (*params)['t'] = theta;
}

Eigen::Matrix4d Rmath::DHtransform::transform() const
{
    return Rmath::homZ(theta, Eigen::Vector3d::UnitZ()*d) *
           Rmath::homX(alpha, Eigen::Vector3d::UnitX()*a);
}

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

Rmath::Translation::Translation(const Rmath::Translation& Ttranslation): Transform(Rmath::translation)
{
    this->values << Ttranslation.asVector()(0), Ttranslation.asVector()(1), Ttranslation.asVector()(2);
}

void Rmath::Translation::parameters(std::map<char, double>* params)
{
    (*params)['x'] = values(0);
    (*params)['y'] = values(1);
    (*params)['z'] = values(2);
}

Eigen::Matrix4d Rmath::Translation::transform() const
{
    return Rmath::homZ(0.0, values);
}

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

Rmath::Rotation::Rotation(const Rmath::Rotation& Trotation): Transform(Rmath::rotation)
{
    this->axis << Trotation.getAxis()(0), Trotation.getAxis()(1), Trotation.getAxis()(2);
    this->angle = Trotation.getAngle();
}

void Rmath::Rotation::parameters(std::map<char, double>* params)
{
    (*params)['x'] = axis(0);
    (*params)['y'] = axis(1);
    (*params)['z'] = axis(2);
    (*params)['a'] = angle;
}

Eigen::Matrix4d Rmath::Rotation::transform() const
{
    Eigen::Matrix4d transf;
    Rmath::homogenize(Rmath::rot(angle,axis), &transf);
    return transf;
}

void Rmath::Rotation::print(std::ostream& out) const
{
    out << "[ " << axis(0) << ", " << axis(1)
        << ", " << axis(2) << ", " << angle << " ]" << std::endl;
}

std::ostream& Rmath::operator<<(std::ostream& out, const Rmath::Rotation& rt)
{
    rt.print(out);
}

