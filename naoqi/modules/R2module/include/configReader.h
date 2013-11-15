/**
* @class: ConfigReader
* This class implements a reader for .cfg files
*
* @file configReader.h
* @author Claudio Delli Bovi, Chiara Picardi, Francesco Riccio
*/

#ifndef CONFIG_READER
#define CONFIG_READER

#include <fstream>
#include <sstream>

#include <string>
#include <vector>
#include <cmath>

#include <soth/HCOD.hpp>

#include "libmath/transform.h"

static double to_double(std::string s){
    double f;
    std::istringstream(s) >> f;
    return f;
}

class ConfigReader{
private:
    std::vector<Rmath::Transform*> transformations;
    std::vector<double> minBounds;
    std::vector<double> maxBounds;
    std::vector<double> zero_pose;
    std::vector<std::string> jointID;

    void dhReader(std::string dh_file_path);
    void paramsReader(std::string bounds_file_path = "");
    double valueReader(std::string _val);

public:
    ConfigReader(std::string dh_file_path, std::string params_file_path = "");

    void storeJointsID(std::vector<std::string> *_jointID);

    void extractJointParams(std::vector<double>* _min, std::vector<double>* _max, std::vector<double>* _zero_pose, int _i, int _j);

    soth::VectorBound extractJointBounds();
    void extractJointBounds(Eigen::MatrixXd* bounds);

    inline std::vector<Rmath::Transform*> Transformations()
    {
        return transformations;
    }
};

#endif
