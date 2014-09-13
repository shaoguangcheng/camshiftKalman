#include "camshiftkalman.h"
#include "camShift.h"

//#include "marco.h"


#define DEBUG

#include <stdlib.h>

camShiftKalman::camShiftKalman(const string videoName_, double start, const Mat target, const Rect targetWindow, featureType type_)
{
    videoName = videoName_;
    frameStart = start;
    currentFrame = target;
    trackWindow = targetWindow;
    type = type_;

#ifdef DEBUG
    imshow("current",currentFrame);
    waitKey();
#endif

    vMin = 10;
    vMax = 256;
    sMin = 0;

    namedWindow(winName, WINDOW_AUTOSIZE);
    createTrackbar("vMin", winName, &vMin, 255, 0);
    createTrackbar("vMax", winName, &vMax, 255, 0);
    createTrackbar("sMin", winName, &sMin, 255, 0);

    isShowHist = true;
    isShowBackProject = true;
    if(isShowHist)
        namedWindow("target_histgram", WINDOW_AUTOSIZE);
    if(isShowBackProject)
        namedWindow("back_projection", WINDOW_AUTOSIZE);

    namedWindow(winName, WINDOW_AUTOSIZE);
}

camShiftKalman::~camShiftKalman()
{

}


void camShiftKalman::extractTargetModel()
{
    /*
     * calculate track model with the input image constaining target
     */
    Mat hsv, mask;

    cvtColor(currentFrame, hsv, CV_BGR2HSV);
    inRange(hsv, Scalar(0, sMin, MIN(vMin, vMax)), Scalar(180, 256, MAX(vMin, vMax)), mask);

    Mat ROIMask(mask, trackWindow);

    switch(type){
    case HUE :
    {
        int histSize = 200;
        float rangeH[] = {0, 180};
        const float* range = rangeH;
        int fromTo[] = {0,0};

        Mat hue;
        hue.create(hsv.size(), hsv.depth());
        mixChannels(&hsv, 1, &hue, 1, fromTo, 1);

        Mat ROI(hue, trackWindow);

        calcHist(&ROI, 1, 0, ROIMask, hist, 1, &histSize, &range, true, false);
        normalize(hist, hist, 0, 255, CV_MINMAX);

        if(isShowHist){
            Mat histImage = drawHist1d(hist, histSize);
            imshow("target_histgram", histImage);
        }

//        delete range;
        break;
    }
    case SATURATION_HUE :
    {
        float rangeH[] = {0, 180};
        float rangeS[] = {0, 255};
        const float* range[] = {rangeH, rangeS};
        const int channels[] = {0,1};
        const int histSize[] = {200,100};

        Mat ROI(hsv, trackWindow);

        calcHist(&ROI, 1, channels, ROIMask, hist, 2, histSize, range);

        normalize(hist, hist, 0, 255, CV_MINMAX);

        isShowHist = false;

        break;
    }
    case LBP_HUE :
    {

        break;
    }
    case LBP_SATURATION_HUE :
    {
        int fromTo_1[] = {2,0};
        int formTo_2[] = {0,2};
        const int histSize[] = {50, 10, 36};
        const int channels[] = {0, 1, 2};

        float* rangeH = new float [histSize[0]+1];
        float* rangeS = new float [histSize[1]+1];
        float* rangeLBP = new float [histSize[2]+1];

        float step = 180.0/histSize[0];
        for(int i=0;i<histSize[0]+1;i++)
            rangeH[i] = i*step;

        step = 255.0/histSize[1];
        for(int i=0;i<histSize[1]+1;i++)
            rangeS[i] = i*step;

        float pattern36[] = {0,1,3,5,7,9,11,13,15,17,19,21,23,25,27,29,31,37,39,43,45,47,51,53,55,59,61,63,85,87,91,95,111,119,127,255};
        for(int i=0;i<histSize[2];i++)
            rangeLBP[i] = pattern36[i];
        rangeLBP[histSize[2]] = 256;

        const float* range[] = {rangeH, rangeS, rangeLBP};

        Mat value;
        value.create(hsv.size(), hsv.depth());
        mixChannels(&hsv, 1, &value, 1, fromTo_1, 1);

        /*
         * compute LBPRI using value channel
         */
        getLBPRI(value);
        mixChannels(&value, 1, &hsv, 1, formTo_2, 1);

        Mat ROI(hsv, trackWindow);
        calcHist(&ROI, 1, channels, ROIMask, hist, 3, histSize, range, false, false);

        /*
         * normalize hist
         */
        normalizeHist(hist);

        isShowHist = false;

        delete [] rangeH;
        delete [] rangeS;
        delete [] rangeLBP;

        break;
    }
    default :
        break;
    }
}

/**
 * @brief norm_L2 calculate the L2 norm
 * @param x
 * @param y
 * @return
 */
inline double norm_L2(const cv::Point &x, const cv::Point &y)
{
    return std::sqrt((double)((x.x-y.x)*(x.x-y.x)+(x.y-y.y)*(x.y-y.y)));
}

void camShiftKalman::track()
{
    Mat hsv, mask;
    Point lastCenter(trackWindow.x,trackWindow.y);
    bool isLost = false;

    TermCriteria term(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 );

    VideoCapture video;
    video.open(videoName);
    if(!video.isOpened()){
        cerr << "open " << videoName << "error" << endl;
        exit(-1);
    }

        /*
         * initialize the kalman filter
         */
    double interval = 1.0/video.get(CV_CAP_PROP_FPS);
    initKalman(interval);

    video.set(CV_CAP_PROP_POS_FRAMES, frameStart);
    while(1){
        video.read(currentFrame);
        if(currentFrame.empty())
             break;

#ifdef DEBUG
    cout << "nframe : " << video.get(CV_CAP_PROP_POS_FRAMES) << endl;
#endif

         cvtColor(currentFrame, hsv, CV_BGR2HSV);
         inRange(hsv, Scalar(0, sMin, MIN(vMin, vMax)), Scalar(180, 256, MAX(vMin, vMax)), mask);

         switch (type){
             case HUE :{
                 float rangeH[] = {0, 180};
                 const float* range = rangeH;
                 int fromTo[] = {0,0};

                 Mat hue;
                 hue.create(hsv.size(), hsv.depth());
                 mixChannels(&hsv, 1, &hue, 1, fromTo, 1);

                 calcBackProject(&hue, 1, 0, hist, backProject, &range);

                 break;
             }
             case SATURATION_HUE :
             {
                 float rangeH[] = {0, 180};
                 float rangeS[] = {0, 255};
                 const float* range[] = {rangeH, rangeS};
                 const int channels[] = {0,1};

                 calcBackProject(&hsv, 1, channels, hist, backProject, range);

                 break;
             }
             case LBP_SATURATION_HUE :
             {
                int fromTo_1[] = {2,0};
                int fromTo_2[] = {0,2};
                const int channels[] = {0,1,2};
                float rangeH[] = {0, 180};
                float rangeS[] = {0, 255};
                float rangeLBP[] = {0, 255};
                const float* range[] = {rangeH, rangeS, rangeLBP};

                Mat value;
                value.create(hsv.size(), hsv.depth());
                mixChannels(&hsv, 1, &value, 1, fromTo_1, 1);

                /*
                 * compute LBPRI using value channel
                 */
                getLBPRI(value);
                mixChannels(&value, 1, &hsv, 1, fromTo_2, 1);

                calcBackProject(&hsv, 1, channels, hist, backProject, range);

                break;
             }
             case LBP_HUE :
             {
                break;
             }
             default :
                break;
         }

         backProject &= mask;

         /*
          * do camshift
          */
         if(trackWindow.width <= 0 || trackWindow.height <=0)
            trackWindow = Rect(0, 0, currentFrame.cols, currentFrame.rows);

         RotatedRect box = camShift(backProject, trackWindow, term);
         camCenter = Point(box.center.x, box.center.y);

#ifdef DEBUG
    std::cout << "[ x : " << trackWindow.x << " y : " << trackWindow.y << " width : " << trackWindow.width << " height : " << trackWindow.height << std::endl;
#endif

         /*
          * do kalman prediction
          */
         KF.predict();
         KFPredictCenter = getCurrentState();

         /*
          * set measurement
          */
         measurement.at<float>(0) = camCenter.x;
         measurement.at<float>(1) = camCenter.y;

         /*
          * do kalman correction
          */
         KF.correct(measurement);
         KFCorrectCenter = getCurrentState();

         if(isShowBackProject)
            imshow("back_projection", backProject);

         if(norm_L2(KFCorrectCenter, lastCenter)>350 || trackWindow.area() < 10 || trackWindow.width <= 0 || trackWindow.height <=0){
             isLost = true;

             Mat image;
             currentFrame.copyTo(image);
             putText(image, "Target Lost", Point(image.rows/2,image.cols/4), cv::FONT_HERSHEY_PLAIN, 2.0, Scalar(0,0,255), 2);
             imshow(winName, image);
         }
         else
            drawTrackResult();

         if(!isLost)
            lastCenter = KFCorrectCenter;

         waitKey();
         int key = waitKey(int(interval*1000));
         if(key == 27)
            break;

         setCurrentTrackWindow();
    }
}

Point camShiftKalman::getCurrentObjectCenter() const
{
    return KFCorrectCenter;
}

Rect camShiftKalman::getCurrentTrackWindow() const
{
    return trackWindow;
}

void camShiftKalman::initKalman(double interval)
{
    const int stateNum = 4;
    const int measureNum = 2;

    Mat statePost = (Mat_<float>(stateNum, 1) << trackWindow.x+trackWindow.width/2.0,
                     trackWindow.y+trackWindow.height/2.0,
                     0,0);
    Mat transitionMatrix = (Mat_<float>(stateNum, stateNum) << 1, 0, 1, 0,
                            0, 1, 0, 1,
                            0, 0, 1, 0,
                            0, 0, 0, 1);

    KF.init(stateNum, measureNum);

    KF.transitionMatrix = transitionMatrix;
    KF.statePost = statePost;
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(1e-1));
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-3));
    setIdentity(KF.errorCovPost, Scalar::all(0.1));

    measurement = Mat::zeros(measureNum, 1, CV_32F);
}

Point camShiftKalman::getCurrentState() const
{
    Mat statePost = KF.statePost;
    return Point(statePost.at<float>(0), statePost.at<float>(1));
}

void camShiftKalman::setCurrentTrackWindow()
{
    int cols = currentFrame.cols;
    int rows = currentFrame.rows;

    trackWindow.x = KFCorrectCenter.x - trackWindow.width/2;
    trackWindow.y = KFCorrectCenter.y - trackWindow.height/2;

//    trackWindow.x = MAX(0, trackWindow.x);
//    trackWindow.x = MIN(cols, trackWindow.width);
//    trackWindow.y = MAX(0, trackWindow.y);
//    trackWindow.y = MIN(rows, trackWindow.height);

    trackWindow &= Rect(0, 0, cols, rows);

    if(trackWindow.width <= 0 || trackWindow.height <=0){
        int width = MIN(KFCorrectCenter.x, cols-KFCorrectCenter.x)*2;
        int height = MIN(KFCorrectCenter.y, rows-KFCorrectCenter.y)*2;

        trackWindow = Rect(KFCorrectCenter.x-width/2, KFCorrectCenter.y-height/2, width, height);
    }
}

void camShiftKalman::normalizeHist(Mat &hist)
{
    if(hist.dims != 3){
        cout << "can only normalize hist when hist.dims = 3" << endl;
        exit(-1);
    }

    /*
     *  find the min and max elements in hist
     */
    float min = 1e10, max = 0;
    for(int i=0;i<hist.size[0];i++)
    for(int j=0;j<hist.size[1];j++)
    for(int k=0;k<hist.size[2];k++){
        float tmp = hist.at<float>(i, j, k);
        if(tmp < min)
            min = tmp;
        if(tmp > max)
            max = tmp;
    }

    /*
     * normalize the hist
     */
    hist = (hist-min)/(max-min)*255;

}

void camShiftKalman::drawTrackResult()
{
    Mat image;

    currentFrame.copyTo(image);

    circle(image, camCenter, 2, Scalar(255,0,0), 2, CV_AA); // draw camshift result
    circle(image, KFPredictCenter, 2, Scalar(0,255,0), 2, CV_AA); // draw kalman predict result
    circle(image, KFCorrectCenter, 2, Scalar(0,0,255), 2, CV_AA); // draw kalman correct result
    rectangle(image, trackWindow, Scalar(0,0,255), 3, CV_AA); // draw track window

    imshow(winName, image);
}

Mat camShiftKalman::drawHist1d(const Mat hist, int histSize) const
{
    Mat histimg = Mat::zeros(640, 480, CV_8UC3);

    histimg = Scalar::all(0);
    int binW = histimg.cols / histSize;
    Mat buf(1, histSize, CV_8UC3);
    for( int i = 0; i < histSize; i++ )
        buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./histSize), 255, 255);
    cvtColor(buf, buf, CV_HSV2BGR);

    for( int i = 0; i < histSize; i++ )
    {
        int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows/255);
        rectangle( histimg, Point(i*binW,histimg.rows),
                   Point((i+1)*binW,histimg.rows - val),
                   Scalar(buf.at<Vec3b>(i)), -1, 8 );
    }

    return histimg;
}

Mat camShiftKalman::drawHist2d(const Mat hist, int histSizeX, int histSizeY) const
{
    Mat histImg = Mat::zeros(640, 480, CV_8UC3);
	return histImg;
}
