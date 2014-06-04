#include "util.h"

void readConfig(char* configFileName, char* videoPath, cv::Rect &box)
{
    int x;
    int y;
    int w;
    int h;

    std::fstream f;
    char cstring[1000];
    int readS=0;

    f.open(configFileName, std::fstream::in);

    char param1[200]; strcpy(param1,"");
    char param2[200]; strcpy(param2,"");
    char param3[200]; strcpy(param3,"");

    f.getline(cstring, sizeof(cstring));
    readS=sscanf (cstring, "%s %s %s", param1,param2, param3);

    strcpy(videoPath,param3);

    f.getline(cstring, sizeof(cstring));
    f.getline(cstring, sizeof(cstring));
    f.getline(cstring, sizeof(cstring));


    readS=sscanf (cstring, "%s %s %i %i %i %i", param1,param2, &x, &y, &w, &h);

    box = cv::Rect(x, y, w, h);

}

void writeXML(std::string configFileName, const std::string videoPath, const cv::Rect box)
{
    cv::FileStorage fs(configFileName, cv::FileStorage::WRITE);

    fs << "videoPath" << videoPath;
    fs << "topLeft_x" << box.x;
    fs << "topLeft_y" << box.y;
    fs << "width" << box.width;
    fs << "height" << box.height;

    fs.release();
}

void readXML(std::string configFileName, std::string &videoPath, cv::Rect &box)
{
    cv::FileStorage fs;
    fs.open(configFileName, cv::FileStorage::READ);

    videoPath = (std::string)fs["videoPath"];
    box.x = (int)fs["topLeft_x"];
    box.y = (int)fs["topLeft_y"];
    box.width = (int)fs["width"];
    box.height = (int)fs["height"];

    fs.release();
}

