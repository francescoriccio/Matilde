/**
* @class: Transform
* @namespace: Rmath
* This class (and related hierarchy) describes a basic rigid transformation.
*
* @file transform.h
* @author Claudio Delli Bovi, Chiara Picardi, Francesco Riccio
*/

#ifndef TRANSFORM
#define TRANSFORM

#include <iostream>
#include <string>
#include <map>
#include <Eigen/Core>

#include "libmath/Rutils.h"


namespace Rmath{

// Transform types
enum transform_type{
    // Rigid translation
    translation,
    // Rigid rotation
    rotation,
    // Rototranslation sequence according to Denavit-Hartenberg conventions
    dh_transform
};


/** ------------------------ Transform base class (interface) declaration ------------------------ */


class Transform
{

protected:

    // Transformation type
    transform_type type;

public:

    // Default constructor
    Transform(transform_type _type): type(_type){ }
    // Copy constructor
    Transform(const Rmath::Transform& transf){ type = transf.getType(); }
    // Destructor
    virtual ~Transform(){}

    // Getter for the transformation type
    inline transform_type getType() const { return type; }

    // Update method
    virtual void update(double value) = 0;
    // Clone method
    virtual Rmath::Transform* clone() = 0;

    // Retrieve the transformation parameters
    virtual void parameters(std::map<char, double>* params) = 0;
    // Retrieve the transformation in homogeneous matrix form
    virtual Eigen::Matrix4d transform() const = 0;

    // Print method
    virtual void print(std::ostream& out) const = 0;

};


/** ------------------------ Denavit-Hartenberg transform class declaration ------------------------ */


class DHtransform: public Transform
{

protected:

    // Denavit-Hartenberg parameters
    double a;
    double alpha;
    double d;
    double theta;

public:

    // Default constructor
    DHtransform(double _a = 0.0, double _alpha = 0.0, double _d = 0.0, double _theta = 0.0):
        Transform(dh_transform), a(_a), alpha(_alpha), d(_d), theta(_theta){ }
    // Copy constructor
    explicit DHtransform(const Rmath::DHtransform& Tdh);

    // Parameter getters
    inline const double& Zshift() const { return d; }
    inline const double& Xshift() const { return a; }
    inline const double& Zangle() const { return theta; }
    inline const double& Xangle() const { return alpha; }

    // Parameter setters
    inline void changeZshift(double _d) { d = _d; }
    inline void changeXshift(double _a) { a = _a; }
    inline void changeZangle(double _t) { theta = _t; }
    inline void changeXangle(double _a) { alpha = _a; }

    // Update method
    inline void update(double value) { theta = value; }
    // Clone method
    Rmath::DHtransform* clone() { return new Rmath::DHtransform(*this); }

    // Retrieve the transformation parameters
    void parameters(std::map<char, double>* params);
    // Retrieve the transformation in homogeneous matrix form
    Eigen::Matrix4d transform() const;

    // Print method
    void print(std::ostream& out) const;
    // Stream operator overload
    friend std::ostream& operator<<(std::ostream& out, const DHtransform& dh);
};


/** ------------------------ Rigid translation class declaration ------------------------ */


class Translation: public Transform
{

protected:

    // Translation vector
    Eigen::Vector3d values;

public:

    // Default constructor
    Translation(double _tx, double _ty, double _tz):
        Transform(translation) { values << _tx, _ty, _tz; }
    // Copy constructor
    Translation(const Rmath::Translation& Ttranslation);

    // Parameter setters
    inline void changeXshift(double _tx) { values(0) = _tx; }
    inline void changeYshift(double _ty) { values(1) = _ty; }
    inline void changeZshift(double _tz) { values(2) = _tz; }

    // Update method
    inline void update(double value) { }
    // Clone method
    Rmath::Translation* clone() { return new Rmath::Translation(*this); }

    // Retrieve the transformation parameters
    void parameters(std::map<char, double>* params);
    // Retrieve the transformation as a 3-dimensional vector
    const Eigen::Vector3d& asVector() const { return values; }
    // Retrieve the transformation in homogeneous matrix form
    Eigen::Matrix4d transform() const;

    // Print method
    void print(std::ostream& out) const;
    // Stream operator overload
    friend std::ostream& operator<<(std::ostream& out, const Translation& tr);

};


/** ------------------------ Rigid rotation class declaration ------------------------ */


class Rotation: public Transform
{

protected:

    // Rotation axis
    Eigen::Vector3d axis;
    // Rotation angle (in radians)
    double angle;

public:

    // Default constructor
    Rotation(double _rx, double _ry, double _rz, double _angle):
        Transform(rotation), angle(_angle) { axis << _rx, _ry, _rz; }
    // Copy constructor
    Rotation(const Rmath::Rotation& Trotation);

    // Parameter getters
    inline const double& getAngle() const { return angle; }
    inline const Eigen::Vector3d& getAxis() const { return axis; }

    // Parameter setters
    inline void changeAngle(double _a) { angle = _a; }
    void changeAxis(const Eigen::Vector3d _axis) { axis = _axis; }

    // Update method
    inline void update(double value) { }
    // Clone method
    Rmath::Rotation* clone() { return new Rmath::Rotation(*this); }

    // Retrieve the transformation parameters
    void parameters(std::map<char, double>* params);
    // Retrieve the transformation as a 3x3 rotation matrix
    inline const Eigen::Matrix3d asMatrix() const { return rot(angle, axis); }
    // Retrieve the transformation in homogeneous matrix form
    Eigen::Matrix4d transform() const;

    // Print method
    void print(std::ostream& out) const;
    // Stream operator overload
    friend std::ostream& operator<<(std::ostream& out, const Rotation& rt);

};

}

#endif
