#ifndef POTENTIAL_FIELD_H
#define POTENTIAL_FIELD_H

/*
  PotentialField class here implemented strongly refers to [1], it provides an algorithm for corridor navigation and obstacle avoidance using
  visual potentials. The robot is equipped with a camera system which captures frame from the environment.
  The optical ﬂow and eventually the visual potential field are extracted from a sequence of images captured by the camera mounted on the
  robot. The algorithm makes the robot able to avoid obstacles without any apriori knowledge of the environment.

  Ones the dominant plane is extracted an artificial repulsive force from the obstacle area is generated using the gradient vector field.
  Simultaneously, the artificial attractive force is computed using both the gradient vector field and a constant feed forward that makes the robot
  move forward.
  As overall output the class is able to provide a velocity vector which is an average of the whole potential field. The velocity vector is splitted
  in its own x and y component that will be used as input for the controller (see Section 3 of the report).

  References:
    [1] Onishi N., Imiya A. - Corridor Navigation and Obstacle Avoidance using Visual Potential for Mobile Robot , 2007.

  authors: Maria Laura Aceto, Claudio Delli Bovi, Francesco Riccio.
*/

//openCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>   //calcOpticalFlowPyrLK include
#include <opencv2/imgproc/imgproc.hpp>  //sobel include
#include <opencv2/calib3d/calib3d.hpp>  //findHomography include

#include <iostream>
#include <vector>
#include <cmath>
#include <string>

#define PLANAR_FLOW true

#define GRID_SIZE 10
#define COLS_RAND_INTERVAL 0.3
#define AFFINITY_SAMPLES 10
#define ERR_THRESH 1

#define PLANAR_FLOW_WEIGHT 1.1
#define OBSTACLES_WEIGHT 1.3

// to_string: simply converts an integer into a string.
static std::string to_string(const float& i) {
    std::stringstream ss;
    ss << i;
    return ss.str();
}

class PotentialField{

private:
    //members
    cv::Mat currFrame;
    cv::Mat prevFrame;

    cv::Mat opticalFlowField;
    cv::Mat planarFlowField;
    cv::Mat dominantPlane;
    cv::Mat bluredDominantPlane;
    cv::Mat potentialField;
    cv::Mat velocityAverage;

    std::vector<float> prevVelocity;
    std::vector<float> velocity;

    bool verbose;

    int reds;

    std::vector<cv::Point2f> gridPoints;
    std::vector<cv::Point2f> opticalFlowPoints;
    std::vector<cv::Point2f> planarFlowPoints;

    std::vector<cv::Mat> potentialFieldXY;
    std::vector<double> errors;

    /*
      fill: takes as input a binary image detecting contours and possible holes in that image. Once hole is recognized it will filled
      with a rect big enough to cover the entire hole.
    */
    void fill(cv::Mat dominantPlane);

    /*
      gridFeaturesToTrack: arranges grid point samples of an image in a std::vector according to a choosen granularity (i.e. GRID_SIZE).
    */
    void gridFeaturesToTrack();

    /*
      matMean: simply computes the average of a cv::Mat matrix depending on the choosen GRID_SIZE.
    */
    double matMean(cv::Mat mat);

    /*
      vecMean: simply computes the average of values stored in a std::vector.
    */
    double vecMean(std::vector<double> vec);

    /*
      arrow: just draws an arrow on an image.
    */
    void arrow(cv::Mat img, int x, int y, int u, int v, cv::Scalar color, int size, int thickness);

    /*
      opticalFlowDetection: this method is a wrapper for the openCV 'calcOpticalFlowPyrLK' function. The resulting detection of
      points that have been tracked will be stored in a std::vector, namely 'opticalFlowPoints'.
    */
    void opticalFlowDetection();

    /*
      planarFlowPrediction: the method performs a selection of random samples onto the two images in order to calculate the related homography
      using the openCV 'findHomography' function. Ones the homography 'H' is applied to each grid point the error between the i-th point of the
      optical flow detection and the i-th point of the planar flow prediction is stored in an std::vector, namely 'error'.
    */
    void planarFlowPrediction();

    /*
      buildDominantPlane: provide a dominant plane comparing the optical flow detection and the planar flow prediction, this comparison
      gives us a model of the obstacles in the environment.
    */
    void buildDominantPlane();

    /*
      buildPotentialField: starting from a Gaussian blur on the dominant plane this method computes the Sobel derivates and
      calculates the gradients of the blured image component-wise. After this first step, the method starts to build up the potential field
      considering attractive and repulsive potential fields.
      In the attractive potential case we sum up toghether the current value of the gradient, the planar flow field and the feed forward
      multiplying by their weight. In the other case, the repulsive one, we just store the current value of the gradient multiplied by its weight
      (see Section 3 of the report).
    */
    void buildPotentialField();

    /*
      potentialFieldAverage: First, calculates the average of the potential field component-wise and then stores those components in a vector
    */
    void potentialFieldAverage();

    /*
      visualize: shows a requested image representing steps of the algorith:
        1 - Current camera frame
        2 - Previous camera frame
        3 - Optical flow field
        4 - Planar flow field
        5 - Dominant plane
        6 - Blured dominant plane
        7 - Potential field
        8 - Velocity
    */
    void visualize(int win = 0);

    /*
      run: just a "main" method that performs sequentially the needed steps to provide the required output
    */
    void run();

public:
    //constructor
    PotentialField(float vx = 0, float vy = 0, int v = false);

    //distructor
    ~PotentialField();

    /*
      update: The method takes as input the current and the previous frame retrieved from the Nao's camera in a cv::Mat format
      and a feed forward in velocity. After that calls the run method to process the new available data and compute the requested output.
    */
    void update(cv::Mat pf, cv::Mat cf, float vx = 0, float vy = 0);

    // getters
    inline const std::vector<float>& getVelocity() const{ return velocity; }
    inline const float getRedsPercentage(){ return ( (float) reds) / ((prevFrame.rows/GRID_SIZE) * (prevFrame.cols/GRID_SIZE)); }
};

#endif //POTENTIAL_FIELD_H
