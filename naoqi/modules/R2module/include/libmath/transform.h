/**
* @class: Transform
* @namespace: Rmath
* This class and related hierarchy represents a basic rigid transformation
*
* @file transform.h
* @author Claudio Delli Bovi, Chiara Picardi, Francesco Riccio
*/

#ifndef TRANSFORM
#define TRANSFORM

#include <iostream>
#include <string>
#include <map>
#include <cmath>

#include <Eigen/Core>

#include "libmath/Rutils.h"

namespace Rmath{

enum transform_type{ translation, rotation, dh_transform };

class Transform
{
protected:

    transform_type type;

public:

    Transform(transform_type _type): type(_type){ }

    Transform(const Rmath::Transform& transf){ type = transf.getType(); }

    virtual ~Transform(){}

    inline transform_type getType() const { return type; }

    inline virtual void update(double value) = 0;

    virtual Rmath::Transform* clone() = 0;/* { return new Rmath::Transform(*this); }*/

    virtual void parameters(std::map<char, double>* params) = 0;

    virtual Eigen::Matrix4d transform() const = 0;

    virtual void print(std::ostream& out) const = 0;

};

class DHtransform: public Transform
{
protected:

    double a;
    double alpha;
    double d;
    double theta;

public:

    DHtransform(double _a = 0.0, double _alpha = 0.0, double _d = 0.0, double _theta = 0.0):
        Transform(dh_transform), a(_a), alpha(_alpha), d(_d), theta(_theta){ }

    explicit DHtransform(const Rmath::DHtransform& Tdh);

    inline void changeZshift(double _d) { d = _d; }

    inline void changeXshift(double _a) { a = _a; }

    inline void changeZangle(double _t) { theta = _t; }

    inline void changeXangle(double _a) { alpha = _a; }

    inline const double& Zshift() const { return d; }

    inline const double& Xshift() const { return a; }

    inline const double& Zangle() const { return theta; }

    inline const double& Xangle() const { return alpha; }

    inline void update(double value) { theta = value; }

    Rmath::DHtransform* clone() { return new Rmath::DHtransform(*this); }

    void parameters(std::map<char, double>* params);

    Eigen::Matrix4d transform() const;

    void print(std::ostream& out) const;

    friend std::ostream& operator<<(std::ostream& out, const DHtransform& dh);
};

class Translation: public Transform
{
protected:

    Eigen::Vector3d values;

public:

    Translation(double _tx, double _ty, double _tz):
        Transform(translation) { values << _tx, _ty, _tz; }

    Translation(const Rmath::Translation& Ttranslation);

    inline void changeXshift(double _tx) { values(0) = _tx; }

    inline void changeYshift(double _ty) { values(1) = _ty; }

    inline void changeZshift(double _tz) { values(2) = _tz; }

    inline void update(double value) { }

    Rmath::Translation* clone() { return new Rmath::Translation(*this); }

    void parameters(std::map<char, double>* params);

    const Eigen::Vector3d& asVector() const { return values; }

    Eigen::Matrix4d transform() const;

    void print(std::ostream& out) const;

    friend std::ostream& operator<<(std::ostream& out, const Translation& tr);

};

class Rotation: public Transform
{
protected:

    Eigen::Vector3d axis;
    double angle;

public:

    Rotation(double _rx, double _ry, double _rz, double _angle):
        Transform(rotation), angle(_angle) { axis << _rx, _ry, _rz; }

    Rotation(const Rmath::Rotation& Trotation);

    inline const double& getAngle() const { return angle; }

    inline void changeAngle(double _a) { angle = _a; }

    inline const Eigen::Vector3d& getAxis() const { return axis; }

    inline void update(double value) { }

    Rmath::Rotation* clone() { return new Rmath::Rotation(*this); }

    void changeAxis(const Eigen::Vector3d _axis) { axis = _axis; }

    void parameters(std::map<char, double>* params);

    inline const Eigen::Matrix3d asMatrix() const { return rot(angle, axis); }

    Eigen::Matrix4d transform() const;

    void print(std::ostream& out) const;

    friend std::ostream& operator<<(std::ostream& out, const Rotation& rt);

};

}

#endif
