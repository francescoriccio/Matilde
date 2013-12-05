/**
* @class: ConfigReader
* This class implements a reader for the .cfg files of the project.
*
* @file configReader.h
* @author Claudio Delli Bovi, Chiara Picardi, Francesco Riccio
*/

#include <iterator>

#include "configReader.h"


ConfigReader::ConfigReader(std::string dh_file_path, std::string params_file_path)
{
    dhReader(dh_file_path);
    if(params_file_path != ""){
        paramsReader(params_file_path);
    }
}

void ConfigReader::extractJointParams(std::vector<double>* _min, std::vector<double>* _max, std::vector<double>* _zero_pose, int _i, int _j)
{
    _min->clear();
    _max->clear();
    _zero_pose->clear();

    for(int i=_i; i<_j; ++i)
    {
        _min->push_back( minBounds.at(i) );
        _max->push_back( maxBounds.at(i) );
        _zero_pose->push_back( zero_pose.at(i) );
    }
}

soth::VectorBound ConfigReader::extractJointBounds()
{
    soth::VectorBound bounds(5);//minBounds.size());
//    for(int i=0; i<minBounds.size(); ++i)
    for(int i=7; i<=12; ++i)
    {
        bounds[i] = soth::Bound(minBounds.at(i), maxBounds.at(i));
    }
    return bounds;
}

void ConfigReader::extractJointBounds(Eigen::MatrixXd* bounds)
{
    for(int i=0; i<minBounds.size(); ++i)
    {
        bounds->row(i) << minBounds.at(i), maxBounds.at(i);
    }
}

void ConfigReader::storeJointsID(std::vector<std::string>* _jointID)
{
    *_jointID = jointID;
}

void ConfigReader::dhReader(std::string dh_file_path)
{
    std::ifstream cfg_dh;
    cfg_dh.open(dh_file_path.c_str(), std::ifstream::in);
    if(cfg_dh == NULL)
    {
        std::cerr << "[ConfigReader] Error: DH-table configuration file (.cfg) not specified. " << std::endl;
        return;
    }

    std::string line;
    while(cfg_dh.good())
    {
        getline(cfg_dh, line);
        std::istringstream iss(line);
        std::vector<std::string> tokens;
        std::copy(std::istream_iterator<std::string>(iss),
                  std::istream_iterator<std::string>(),
                  std::back_inserter<std::vector<std::string> >(tokens));

        if(tokens.empty()) continue;

        // Canonical DH transform case
        if (tokens.at(0) == "H")
        {
            Rmath::Transform* dh_transform = new Rmath::DHtransform( to_double(tokens.at(1)),
                                                                     valueReader(tokens.at(2)),
                                                                     to_double(tokens.at(3)),
                                                                     0.0 );
            transformations.push_back(dh_transform);
        }
        // Fixed translation case
        else if (tokens.at(0) == "T")
        {
            Rmath::Transform* transl = new Rmath::Translation( to_double(tokens.at(1)),
                                                               to_double(tokens.at(2)),
                                                               to_double(tokens.at(3)) );
            transformations.push_back(transl);
        }
        // Fixed rotation case
        else if (tokens.at(0) == "R")
        {
            Rmath::Transform* rot = new Rmath::Rotation( to_double(tokens.at(1)),
                                                         to_double(tokens.at(2)) ,
                                                         to_double(tokens.at(3)),
                                                         valueReader(tokens.at(4)) );
            transformations.push_back(rot);
        }

        if (tokens.at(0) == "}") break;
    }
    cfg_dh.close();
}

void ConfigReader::paramsReader(std::string params_file_path)
{

    std::fstream cfg_params;
    cfg_params.open(params_file_path.c_str(), std::fstream::in);
    if(cfg_params == NULL)
    {
        std::cerr << "[ConfigReader] Error: Joint limits configuration file (.cfg) not specified. " << std::endl;
        return;
    }

    std::string line;
    while(cfg_params.good())
    {
        getline(cfg_params, line);
        std::istringstream iss(line);
        std::vector<std::string> tokens;
        std::copy(std::istream_iterator<std::string>(iss),
                  std::istream_iterator<std::string>(),
                  std::back_inserter<std::vector<std::string> >(tokens));

        if(tokens.empty()) continue;
        if( tokens.at(0) == "[" ) continue;
        if( tokens.at(0) == "{" || tokens.at(0) == "}" ) continue;

        zero_pose.push_back( valueReader(tokens.at(0)) );
        minBounds.push_back( to_double(tokens.at(1)) );
        maxBounds.push_back( to_double(tokens.at(2)) );
        jointID.push_back( tokens.at(3) );
    }
    cfg_params.close();
}

double ConfigReader::valueReader(std::string _val)
{
    std::string num = "";

    if(_val[0] == 'M' && _val.size() == 4) return M_PI;
    else if(_val[0] == '-' && _val[1] == 'M' && _val.size() == 5) return -M_PI;

    else if(_val[0] == 'M' && _val[4] == '/')
    {
        for(int i=5; i< _val.size(); ++i) num +=_val[i];
        return M_PI/to_double(num);
    }
    else if(_val[0] == '-' && _val[1] == 'M' && _val[5] == '/')
    {
        for(int i=6; i< _val.size(); ++i) num +=_val[i];
        return -M_PI/to_double(num);
    }
    else if(_val[0] == 'M' && _val[4] == '*')
    {
        for(int i=5; i< _val.size(); ++i) num +=_val[i];
        return M_PI*to_double(num);
    }
    else if(_val[0] == '-' && _val[1] == 'M' && _val[5] == '*')
    {
        for(int i=6; i< _val.size(); ++i) num +=_val[i];
        return -M_PI*to_double(num);
    }
    else return to_double(_val);
}
