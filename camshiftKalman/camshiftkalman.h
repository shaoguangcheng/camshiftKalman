#ifndef CAMSHIFTKALMAN_H
#define CAMSHIFTKALMAN_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.hpp>

#include <string>

#include "LBP.h"

using namespace std;
using namespace cv;

enum featureType{
    HUE = 0,
    SATURATION_HUE,
    LBP_HUE,
    LBP_SATURATION_HUE
    };

class camShiftKalman
{
public:
    camShiftKalman(const string videoName_, double start, const Mat target, const Rect targetWindow, featureType type_);
    ~camShiftKalman();

    void extractTargetModel();
    void track();
    Point getCurrentObjectCenter() const;
    Rect getCurrentTrackWindow() const;

private :
    void initKalman(double);
    Point getCurrentState() const;
    void setCurrentTrackWindow();
    Mat drawHist1d(const Mat hist, int histSize) const;
    Mat drawHist2d(const Mat hist, int histSizeX, int histSizeY) const;
    void drawTrackResult();
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
