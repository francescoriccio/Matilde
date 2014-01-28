/**
* @class: ConfigReader
* This class implements a reader for the .cfg files of the project.
*
* @file configReader.h
* @author Claudio Delli Bovi, Chiara Picardi, Francesco Riccio
*/

#include <iterator>

#include "configReader.h"

ConfigReader::ConfigReader(std::string params_file_path)
{
    paramsReader(params_file_path);
}

 void ConfigReader::setKinChain(std::string dh_file_path)
 {
     transformations.clear();
     kinChainJointIDs.clear();
     dhReader(dh_file_path);
 }

std::vector<double> ConfigReader::getJointParams(std::string _id)
{
    assert( jointParams.find(_id) != jointParams.end() );
    return jointParams[_id];
}

soth::VectorBound ConfigReader::extractJointBounds()
{
    soth::VectorBound bounds(jointParams.size());
    for(int index=0; index<jointID.size();++index)
        bounds[index] = soth::Bound( jointParams[jointID.at(index)].at(2),
                                     jointParams[jointID.at(index)].at(1) );
    return bounds;
}

void ConfigReader::extractJointBounds(Eigen::MatrixXd* bounds)
{
    for(int index=0; index<jointID.size();++index)
        bounds->row(index) << jointParams[jointID.at(index)].at(2),
                jointParams[jointID.at(index)].at(1);
}

void ConfigReader::storeJointsID(std::vector<std::string>* _jointID)
{
    *_jointID = jointID;
}

void ConfigReader::kinChainJointsID(std::vector<std::string>* _kcJointID)
{
    *_kcJointID = kinChainJointIDs;
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

    kinChainJointIDs.clear();

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
        if(tokens.at(0) == "#") continue;

        // Canonical DH transform case
        if (tokens.at(0) == "H")
        {
            Rmath::Transform* dh_transform = new Rmath::DHtransform( to_double(tokens.at(1)),
                                                                     valueReader(tokens.at(2)),
                                                                     to_double(tokens.at(3)),
                                                                     0.0 );
            kinChainJointIDs.push_back(tokens.at(6));
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
                                                         to_double(tokens.at(2)),
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
        if( tokens.at(0) == "#" ) continue;
        if( tokens.at(0) == "{" ) continue;

        if( tokens.at(0) == "}" ) break;

        std::vector<double> params;
        params.push_back(valueReader(tokens.at(0)));
        params.push_back(to_double(tokens.at(2)));
        params.push_back(to_double(tokens.at(1)));

        jointParams.insert(std::pair<std::string,std::vector<double> >(tokens.at(3), params) );

        jointID.push_back(tokens.at(3));
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
