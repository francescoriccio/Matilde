#include "potentialField.h"

PotentialField::PotentialField(float vx, float vy, int v){
    prevVelocity.push_back(vx);
    prevVelocity.push_back(vy);

    velocity.push_back(vx);
    velocity.push_back(vy);

    reds = 0;
    verbose = v;
}

PotentialField::~PotentialField(){
    prevVelocity.clear();
    velocity.clear();

    potentialFieldXY.clear();
    errors.clear();

    planarFlowPoints.clear();
    opticalFlowPoints.clear();
    gridPoints.clear();

    currFrame.release();
    prevFrame.release();
    opticalFlowField.release();
    planarFlowField.release();
    dominantPlane.release();
    potentialField.release();
    velocityAverage.release();

    cv::destroyAllWindows();
}

void PotentialField::fill(cv::Mat dominantPlane){

    cv::Mat filled = cv:: Mat::zeros(prevFrame.rows, prevFrame.cols, CV_8U);

    //set the min and max dimensions of the rectangle that will be recognaized in the image.
    const int mimRectDim = GRID_SIZE;
    const int maxRectDim = GRID_SIZE*25;

    for (int i = 0; i <prevFrame.rows; i++){
        for (int j = 0; j <prevFrame.cols; j++){

            int value = dominantPlane.at<uchar>(i, j);
            // evaluating whether or not the current white part of the dominant plane is an hole.
            if(value == 255){
                cv::Rect rect;

                // detect an hole in the image and store its dimensions in 'rect'.
                floodFill(dominantPlane, cv::Point(j, i), cv::Scalar(254), &rect);

                // check size
                if(rect.width >= mimRectDim && rect.width <= maxRectDim && rect.height >= mimRectDim && rect.height <= maxRectDim){

                    // center of the rectangle, i.e. the current hole.
                    int x = rect.x + rect.width / 2;
                    int y = rect.y + rect.height / 2;

                    //plot the rectangle to cover the hole.
                    rectangle(filled, cv::Point2f(x-rect.width/2,y-rect.height/2), cv::Point2f(x+rect.width/2, y+rect.height/2), 255, CV_FILLED);
                }
            }
        }
    }

    cv::Mat inverted;
    bitwise_not(filled, inverted);

    cv::Mat intersection;
    bitwise_and(dominantPlane, inverted, intersection);

    intersection.copyTo(dominantPlane);
}

void PotentialField::gridFeaturesToTrack(){

    gridPoints.clear();
    for(int i=0; i<prevFrame.rows; i+=GRID_SIZE){
        for(int j=0; j<prevFrame.cols; j+=GRID_SIZE){
            gridPoints.push_back(cv::Point2f(j,i));
        }
    }
}

double PotentialField::matMean(cv::Mat mat){

    double mean = 0;
    for(int i=0; i<mat.cols; i+=GRID_SIZE){
      for(int j=0; j<mat.rows; j+=GRID_SIZE){

          mean += (float) mat.at<char>(j, i);
      }
    }
    return mean /(gridPoints.size());
}

double PotentialField::vecMean(std::vector<double> vec){

    double mean = 0;
    for ( int i=0; i<vec.size(); i++ ) mean += vec.at(i);

    return mean / vec.size();
}

void PotentialField::arrow(cv::Mat img, int x, int y, int u, int v, cv::Scalar color, int size, int thickness){
    if(u == 0 && v == 0) cv::circle(img, cv::Point2f(x, y), thickness, color, CV_FILLED);
    else if(!(u == 0 && v == 0)){
        float l = size;

        float theta = 0;
        if(u == 0 && v > 0) theta = 1.57;
        else if(u == 0 && v < 0) theta = -1.57;

        float alpha = 2.30;
        if(u != 0) theta = atan2(v,u);

        cv::Point2f p0( x +u, y +v);
        cv::Point2f p1( x +u +(l*2)*cos(theta), y +v +(l*2)*sin(theta) );
        cv::Point2f p2( x +u +l*cos(theta-alpha) , y +v +l*sin(theta-alpha) );
        cv::Point2f p3( x +u +l*cos(theta+alpha) , y +v +l*sin(theta+alpha) );

        cv::line( img, cv::Point2f( x, y ), cv::Point2f( x+ u, y+ v ), color, thickness, 8);

        cv::line( img, p1, p2, color, thickness, 1);
        cv::line( img, p1, p3, color, thickness, 1);
        cv::line( img, p0, p3, color, thickness, 1);
        cv::line( img, p0, p2, color, thickness, 1);
    }
}

void PotentialField::opticalFlowDetection(){

    // build the grid of points onto the image which will be used in the optical flow function
    gridFeaturesToTrack();

    opticalFlowPoints.clear();
    opticalFlowPoints.reserve(gridPoints.size());

    // set parameters
    std::vector<uchar> features_found;
    features_found.reserve(gridPoints.size());
    std::vector<float> feature_errors;
    feature_errors.reserve(gridPoints.size());

    // set the pyramid size of the optical flow
    cv::Size pyr_sz = cv::Size( prevFrame.cols/8, prevFrame.rows/8);

    // call the openCV function 'calcOpticalFlowPyrLK' which takes as input two consecutive frames and the std::vector of points to track
    // storing in another std::vector the corrisponding detection.
    cv::calcOpticalFlowPyrLK( prevFrame, currFrame,
                              gridPoints, opticalFlowPoints,
                              features_found, feature_errors,
                              pyr_sz, 1,
                              cv::TermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 30, 0.01));

    if(verbose){
        for( int i=0; i < gridPoints.size(); i++ ){
            arrow(opticalFlowField,
                      gridPoints[i].x, gridPoints[i].y,
                      opticalFlowPoints[i].x - gridPoints[i].x, opticalFlowPoints[i].y - gridPoints[i].y,
                      CV_RGB(255, 255, 255), 1.5, 1.1f);
        }
    }
}

void PotentialField::planarFlowPrediction(){

    std::vector<cv::Point2f> prev_pt, curr_pt;

    srand(time(NULL));
    int count = 0;

    // collect random samples supposed to be on the dominant plane
    while (count < AFFINITY_SAMPLES){

        // normalize the index to the number of samples
        int i = rand() % gridPoints.size();

        // if the random sample is not onto the lower part of the image then will be discarded
        if( gridPoints[i].y < prevFrame.rows/2 || gridPoints[i].y > (prevFrame.rows - 2*GRID_SIZE) ||
                gridPoints[i].x > prevFrame.cols/2 + prevFrame.cols*COLS_RAND_INTERVAL ||
                gridPoints[i].x < prevFrame.cols/2 - prevFrame.cols*COLS_RAND_INTERVAL ) continue;

        // store the selected sample
        prev_pt.push_back(gridPoints[i]);
        curr_pt.push_back(opticalFlowPoints[i]);

        if(verbose){
            circle( prevFrame , gridPoints[i], 5, CV_RGB(255, 0, 0));
            circle( currFrame , opticalFlowPoints[i], 5, CV_RGB(255, 0, 0));
        }

        // update the counter
        ++count;
    }

    // compute the homography 'H' using the openCV 'findHomography' function using the selected samples through a RANSAC process.
    cv::Mat H = findHomography(prev_pt, curr_pt, CV_RANSAC);
    //cv::Mat H = getAffineTransform(aff_pt1, aff_pt2);

    H.convertTo(H, CV_32FC1, 1, 0);

    errors.clear();
    planarFlowPoints.clear();

    // for each point in the grid the method computed the related point projected in the second image 'ptB' through the homography 'H'.
    for(int i=0; i< gridPoints.size(); ++i){

        std::vector<cv::Point3f> point;
        point.push_back( cv::Point3f(gridPoints[i].x, gridPoints[i].y, 1) );

        cv::Mat Hp = H* cv::Mat(point).reshape(1).t();

        cv::Point2f projectedPoint = cv::Point2f( Hp.at<float>(0,0), Hp.at<float>(1,0) );

        // calculate an error as a difference between the i-th point of the optical flow detection and the i-th point of the planar flow prediction.
        cv::Point2f err;
        if(PLANAR_FLOW) err = projectedPoint - opticalFlowPoints[i];
        else err = opticalFlowPoints[i] - gridPoints[i];

        // store the affinity points and the norm of the above error.
        planarFlowPoints.push_back(projectedPoint);
        errors.push_back( norm(err) );

        if(verbose){
            arrow(planarFlowField,
                      gridPoints[i].x, gridPoints[i].y,
                      projectedPoint.x - gridPoints[i].x, projectedPoint.y - gridPoints[i].y,
                      CV_RGB(255.0, 0.0, 255.0), 1.5, 1.1f);
        }
    }
}

void PotentialField::buildDominantPlane(){

    int index = 0;
    cv::Mat tmpGrayImg = cv::Mat::zeros(prevFrame.rows, prevFrame.cols, CV_8U);

    for(int i=0; i<dominantPlane.rows; i+=GRID_SIZE){
        for(int j=0; j<dominantPlane.cols; j+=GRID_SIZE){

            // if the error related to the current sample in between the optical flow detection and the planar flow prediction is less then a given
            // threshold then that portion of the image will be recognized as dominant plane, otherwise as an obstacle.
            if(errors.at(index++) < ERR_THRESH)
                rectangle(tmpGrayImg, cv::Point2f(j, i), cv::Point2f(j+GRID_SIZE, i+GRID_SIZE), CV_RGB(255,255,255), CV_FILLED);
            else
                rectangle(tmpGrayImg, cv::Point2f(j, i), cv::Point2f(j+GRID_SIZE, i+GRID_SIZE), CV_RGB(0,0,0), CV_FILLED);
        }
    }

    // turns the image in a binary one.
    threshold( tmpGrayImg, dominantPlane, 30, 255, CV_THRESH_BINARY);

    // fill holes giving a more robust obstacles detection.
    fill(dominantPlane);
}

void PotentialField::buildPotentialField(){

    // gaussian blur function used to blur the dominant plane.
    GaussianBlur(dominantPlane, bluredDominantPlane, cv::Size(319, 239*2 -1), /*340*/-1, 0);

    // generate potentialField_x and potentialField_y.
    cv::Mat potentialField_x, potentialField_y, abs_potentialField_x, abs_potentialField_y;

    // potentialFieldient X using openCV Sobel derivative.
    Sobel( bluredDominantPlane, potentialField_x, CV_16S,  1, 0, 3/*CV_SCHARR*/, 1, 0, cv::BORDER_DEFAULT );
    Sobel( bluredDominantPlane, abs_potentialField_x, CV_8U,  1, 0, 3/*CV_SCHARR*/, 1, 0, cv::BORDER_DEFAULT );
    cv::convertScaleAbs(abs_potentialField_x, abs_potentialField_x);
    cv::convertScaleAbs(potentialField_x, potentialField_x);

    // potentialFieldient Y using openCV Sobel derivative.
    Sobel( bluredDominantPlane, potentialField_y, CV_16S,  0, 1, 3/*CV_SCHARR*/, 1, 0, cv::BORDER_DEFAULT );
    Sobel( bluredDominantPlane, abs_potentialField_y, CV_8U,  0, 1, 3/*CV_SCHARR*/, 1, 0, cv::BORDER_DEFAULT );
    cv::convertScaleAbs(abs_potentialField_y, abs_potentialField_y);
    cv::convertScaleAbs(potentialField_y, potentialField_y);

    potentialFieldXY.clear();

    // update potential fields.
    potentialFieldXY.push_back(potentialField_x);
    potentialFieldXY.push_back(potentialField_y);

    for(int i=0; i< prevFrame.rows; i+=GRID_SIZE){
        for(int j=0; j< prevFrame.cols; j+=GRID_SIZE){

            cv::Point2f grid_point(j, i);
            int d = dominantPlane.at<uchar>(i +GRID_SIZE/2, j +GRID_SIZE/2);

            // x component
            float gX;
            if(abs_potentialField_x.at<char>(i, j) == 0){
                gX = -(float) potentialFieldXY.at(0).at<char>(i, j);
                potentialFieldXY.at(0).at<char>(i, j) = gX;
            }
            else gX = (float) potentialFieldXY.at(0).at<char>(i, j);

            // y component
            float gY;
            if(abs_potentialField_y.at<char>(i, j) == 0){
                gY = -(float) potentialFieldXY.at(1).at<char>(i, j);
                potentialFieldXY.at(1).at<char>(i, j) = gY;
            }
            else gY = (float) potentialFieldXY.at(1).at<char>(i, j);

            // if we are evaluating a sample on the dominant plane
            if (d != 0){

                // retrieve the planar flow value related to the current sample.
                cv::Point2f planar_point = planarFlowPoints.at( (int) ((i/GRID_SIZE)*(prevFrame.cols/GRID_SIZE) + j/GRID_SIZE));

                // if we are using the planar flow prediction
                if(PLANAR_FLOW){
                    // the attractive potential field is updated using the planar flow and the feed forward times their weight.
                    potentialFieldXY.at(0).at<char>(i, j) = ( -(planar_point - grid_point).x - prevVelocity.at(0) )*PLANAR_FLOW_WEIGHT;
                    potentialFieldXY.at(1).at<char>(i, j) = ( -(planar_point - grid_point).y - prevVelocity.at(1) )*PLANAR_FLOW_WEIGHT;
                }
                // if we are using just the perception of the optical flow field
                else{
                    // the attractive potential field is updated just using the feed forward times its weight.
                    potentialFieldXY.at(0).at<char>(i, j) = ( - prevVelocity.at(0) )*PLANAR_FLOW_WEIGHT;
                    potentialFieldXY.at(1).at<char>(i, j) = ( - prevVelocity.at(1) )*PLANAR_FLOW_WEIGHT;
                }

                if(verbose){
                    arrow(potentialField,
                              grid_point.x, grid_point.y,
                              gX - (planar_point - grid_point).x - prevVelocity.at(0), gY - (planar_point - grid_point).y - prevVelocity.at(1),
                              CV_RGB(0, 255, 0), 1, 1.0f);
                }

            }
            // if we are evaluating a sample which is not on the dominant plane
            else{

                // update reds count.
                ++reds;

                // basically, the repulsive potential field is directely retrieved from the Sobel derivative and multiplied by its weight.
                potentialFieldXY.at(0).at<char>(i, j) = gX *OBSTACLES_WEIGHT;
                potentialFieldXY.at(1).at<char>(i, j) = gY *OBSTACLES_WEIGHT;

                if(verbose) arrow(potentialField, grid_point.x, grid_point.y, gX, gY, CV_RGB(255, 0, 0), 1, 1.0f);
            }
        }
    }
}

void PotentialField::potentialFieldAverage(){

    velocity.clear();

    // compute both x and y average of the relative potential fields.
    float ux = matMean(potentialFieldXY.at(0));
    float uy = matMean(potentialFieldXY.at(1));
    float mod = sqrt(ux*ux + uy*uy);

    // compute theta angle in between the current velocity and the Y-axis in degrees.
    float theta = 0.0;
    if(ux != 0 && mod !=0) theta = atan2(ux/mod, -uy/mod)*180/3.14;

    // store the normilized values of the current velocity avoiding nan and/or infinite values.
    if(mod != 0){
        velocity.push_back(-uy/mod);
        velocity.push_back(ux/mod);
    }
    else{
        velocity.push_back(-uy);
        velocity.push_back(ux);
    }

    // plot current velocity
    if(verbose){

        // data
        if(mod != 0){
            cv::putText(velocityAverage, "   vx: " + to_string(ux/mod), cv::Point2i(210, 20) , CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar::all(255));
            cv::putText(velocityAverage, "   vy: " + to_string(-uy/mod), cv::Point2i(210, 35) , CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar::all(255));
        }
        else{
            cv::putText(velocityAverage, "   vx: " + to_string(ux), cv::Point2i(210, 20) , CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar::all(255));
            cv::putText(velocityAverage, "   vy: " + to_string(uy), cv::Point2i(210, 35) , CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar::all(255));
        }
        cv::putText(velocityAverage, "theta: " + to_string(theta) , cv::Point2i(210, 50) , CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar::all(255));

        // Y-axis
        arrow(velocityAverage, prevFrame.cols/2 ,0 , 0, prevFrame.rows, CV_RGB(255,255, 255), 1.5, 1.1f);
        for(int i=0; i<prevFrame.rows; i+=GRID_SIZE)
            cv::line(velocityAverage, cv::Point2f(prevFrame.cols/2,i), cv::Point2f(prevFrame.cols/2 +GRID_SIZE/4,i), CV_RGB(255,255,255));

        // X-axis
        arrow(velocityAverage, 0, prevFrame.rows/2, prevFrame.cols, 0, CV_RGB(255,255, 255), 1.5, 1.1f);
        for(int i=0; i<prevFrame.cols; i+=GRID_SIZE)
            cv::line(velocityAverage, cv::Point2f(i, prevFrame.rows/2), cv::Point2f(i, prevFrame.rows/2 +GRID_SIZE/4), CV_RGB(255,255,255));

        // velocity
        arrow(velocityAverage, prevFrame.cols/2 , prevFrame.rows/2, 10*ux, 10*uy, CV_RGB(143, 0, 255), 2, 3.1f);
    }
}

void PotentialField::visualize(int win){

    if ( (char) cv::waitKey(33) != 27 ){

        if(win == 0) cv::imshow("Current frame", currFrame);
        else if(win == 1) cv::imshow("Previous frame", prevFrame);

        else if(win == 2) cv::imshow("Optical flow field", opticalFlowField);
        else if(win == 3) cv::imshow("Planar flow field", planarFlowField);

        else if(win == 4) cv::imshow("Dominant plane", dominantPlane);
        else if(win == 5) cv::imshow("Blured dominant plane", bluredDominantPlane);

        else if(win == 6) cv::imshow("Potential field", potentialField);
        else if(win == 7) cv::imshow("Velocity", velocityAverage);

    }
}

void PotentialField::update(cv::Mat pf, cv::Mat cf, float vx, float vy){
    prevVelocity.clear();

    // reset reds percentage
    reds = 0;

    // update previous velocities
    prevVelocity.push_back(vx);
    prevVelocity.push_back(vy);

    // update previous and current frame
    prevFrame = pf.clone();
    currFrame = cf.clone();

    // reset images
    opticalFlowField = cv::Mat::zeros(pf.rows, pf.cols, CV_8U);
    planarFlowField = cv::Mat::zeros(pf.rows, pf.cols, CV_8U);
    dominantPlane = cv::Mat::zeros(pf.rows, pf.cols, CV_8U);
    bluredDominantPlane = cv::Mat::zeros(pf.rows, pf.cols, CV_8U);
    potentialField = cv::Mat::zeros(pf.rows, pf.cols, CV_8UC3);
    velocityAverage = cv::Mat::zeros(pf.rows, pf.cols, CV_8UC3);

    run();
}

void PotentialField::run(){

    opticalFlowDetection();
    planarFlowPrediction();

    buildDominantPlane();

    buildPotentialField();
    potentialFieldAverage();

    if(verbose){
        visualize(0);   //current frame
//        visualize(1);   //previous frame
        visualize(2);   //Optical flow field
//        visualize(3);   //Planar flow field
        visualize(4);   //Dominant plane
        visualize(5);   //Blured dominant plane
        visualize(6);   //Potential field
        visualize(7);   //Velocity
    }
}

