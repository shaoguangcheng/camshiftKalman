#ifndef UTIL_H
#define UTIL_H

#include <opencv2/core/core.hpp>
#include <string>
#include <fstream>

/**
 * @brief readConfig read config text
 * @param configFileName config file name
 * @param videoPath the returned video name
 * @param box returned value of initialize tracking window
 */

void readConfig(char* configFileName, char* videoPath, cv::Rect &box);

/**
 * @brief writeXML write XML and YMAL config file
 * @param configFileName
 * @param videoPath
 * @param box
 */
void writeXML(std::string configFileName, const std::string videoPath, const cv::Rect box);

/**
 * @brief readXML read XML and YMAL config file
 * @param configFileName
 * @param videoPath
 * @param box
 */
void readXML(std::string configFileName, std::string &videoPath, cv::Rect &box);

/**
 *  @unique compute the unique elements in src
 *  @len the length of src
 *  @return the total number of elements in dest src[0] ... src[n-1]
 */
template <class T> int unique(T* src, int len)
{
    assert(src != NULL);

    int index = 1, end = 0;
    for(;index < len; index++){
        int k = 0;
        for(;k <= end; k++)
            if(src[index] == src[k])
                break;
        if(k > end){
            end++;
            src[end] = src[index];
        }
    }

    return end+1;
}
#endif // UTIL_H
