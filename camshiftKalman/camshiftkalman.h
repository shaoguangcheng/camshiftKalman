/**
 *An object tracking project using camshift and Kalman Filter based on OpenCV

 *I implement an object tracking algorithm using camshift and Kalman Filter. Three features, hue,saturation and rotation invariant Local Binary Pattern, are used to model the tracking object. Kalman Filter is employed to smooth the motion trajectory and predict the next position when the object is occluded
 *If you any questions about this project, please contact me directly. E-mail : chengshaoguang1291@gmail.com From Northwestern Polytechnical University.
 *
 */

#ifndef CAMSHIFTKALMAN_H
#define CAMSHIFTKALMAN_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.hpp>

#include <string>

#include "LBP.h"

using namespace std;
using namespace cv;

/**
 * @brief The featureType enum the kind of type to be chosed for tracking
 */
enum featureType{
    HUE = 0,
    SATURATION_HUE,
    LBP_HUE,
    LBP_SATURATION_HUE
    };

/**
 * @brief The camShiftKalman class the core of algorithm
 */
class camShiftKalman
{
public:
    camShiftKalman(const string videoName_, double start, const Mat target, const Rect targetWindow, featureType type_);
    ~camShiftKalman();

    /**
     * @brief extractTargetModel build the model for the object
     */
    void extractTargetModel();

    /**
     * @brief track all tracking process is executed in this function
     */
    void track();

    /**
     * @brief getCurrentObjectCenter return the current position of object
     * @return
     */
    Point getCurrentObjectCenter() const;

    /**
     * @brief getCurrentTrackWindow return the tracking window
     * @return
     */
    Rect getCurrentTrackWindow() const;

private :
    /**
     * @brief initKalman initialize the Kalman Filter
     */
    void initKalman(double);

    /**
     * @brief getCurrentState return the current state of KF
     * @return
     */
    Point getCurrentState() const;

    /**
     * @brief setCurrentTrackWindow adjust the tracking window to follow the object
     */
    void setCurrentTrackWindow();

    /**
     * @brief drawHist1d draw histgram of one dimision
     * @param hist histgram data to draw
     * @param histSize the number of bins
     * @return
     */
    Mat drawHist1d(const Mat hist, int histSize) const;

    /**
     * @brief drawHist2d draw histgram of two dimision
     * @param hist histgram data to draw
     * @param histSizeX the number of bins in horizen
     * @param histSizeY the number of bins in vertical
     * @return
     */
    Mat drawHist2d(const Mat hist, int histSizeX, int histSizeY) const;

    /**
     * @brief drawTrackResult draw the tracking result on image
     */
    void drawTrackResult();

    /**
     * @brief normalizeHist normalize the hist (only for 3-dim)
     * @param hist histgram data to normalize
     */
    void normalizeHist(Mat &hist);

private :
    featureType type;

    string  videoName;

    Mat currentFrame;
    Mat hist;
    Mat backProject;

    Point camCenter;
    Point KFPredictCenter;
    Point KFCorrectCenter;
    Rect trackWindow;

    int frameStart;
    int vMin;
    int vMax;
    int sMin;

    KalmanFilter KF;
    Mat_<float> measurement;

    string winName;

    bool isShowHist;
    bool isShowBackProject;
};

#endif // CAMSHIFTKALMAN_H
