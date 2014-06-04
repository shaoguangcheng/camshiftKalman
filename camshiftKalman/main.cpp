#define DEBUG

#include "camshiftkalman.h"
#include "util.h"

#include <opencv2/core/core.hpp>

Rect selection;
Point origin;
Mat frame;

bool trackObject = false;
bool selectObject;

 void onMouse( int event, int x, int y, int, void* )
{
    if( selectObject )
    {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);

        selection &= Rect(0, 0, frame.cols, frame.rows);
    }

    switch( event )
    {
    case CV_EVENT_LBUTTONDOWN:
        origin = Point(x,y);
        selection = Rect(x,y,0,0);
        selectObject = true;
        break;
    case CV_EVENT_LBUTTONUP:
        selectObject = false;
        if( selection.width > 0 && selection.height > 0 )
            trackObject = true;
        break;
    default :
        break;
    }

}

int main(int argc, const char* argv[])
{
    const char* args = {
        "{ m | useMouse  | true | how to choose object to track}"
        "{ v | videoName |      | the video to track}"
        "{ t | featureType | 0  | 0 -- HUE 1 -- SATURATION_HUE 2 -- LBP_HUE 3 -- LBP_SATURATION_HUE}"
    };

    CommandLineParser parser(argc, argv, args);
    bool isUseMouse = parser.get<bool>("useMouse");

    Rect trackWindow;

    string videoName;
    if(isUseMouse)
        videoName = parser.get<string>("videoName");
    else
    {
        /*
         * read config parameters from file
         */
        string configFile = "config.yaml";
        readXML(configFile, videoName, trackWindow);
    }

    featureType type;
    int num = parser.get<int>("featureType");
    switch(num){
    case 0 :
        type = HUE;
        break;
    case 1 :
        type = SATURATION_HUE;
        break;
    case 2 :
        type = LBP_HUE;
        break;
    case 3 :
        type = LBP_SATURATION_HUE;
        break;
    default :
        cerr << "ERROR : feature type can only be chosed among 0,1,2,3" << endl;
        return -1;
    }

    VideoCapture video;
    video.open(videoName);
    if(!video.isOpened()){
        cerr << "open " << videoName << " error" << endl;
        cerr << "current parameters : " << endl;
        parser.printParams();
        return -1;
    }

    Mat image;
    if(isUseMouse){
        string winName = "choose_windows";
        namedWindow(winName, WINDOW_AUTOSIZE);

        setMouseCallback(winName, onMouse, 0);

        double interval = 1.0/video.get(CV_CAP_PROP_FPS);
        while(true)
        {
            if(trackObject)
                break;

            video.read(frame);
            if(frame.empty())
                return -1;

            frame.copyTo(image);
            if( selectObject && selection.width > 0 && selection.height > 0 )
            {
                Mat roi(frame, selection);
                bitwise_not(roi, roi);
            }

            imshow(winName, frame);
            int key = waitKey(int(interval*1000));
            if(key == 27)
                return -1;
        }

        trackWindow = selection;

        destroyWindow(winName);
    }
    else
    {
        video.read(image);
    }

    /*
     * do tracking
     */
    double n = video.get(CV_CAP_PROP_POS_FRAMES);
    camShiftKalman tracker(videoName, n, image, trackWindow, type);

    tracker.extractTargetModel();
    tracker.track();

    return 0;
}

