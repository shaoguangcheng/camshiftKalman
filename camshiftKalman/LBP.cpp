#include "LBP.h"

void getLBP(Mat &img)
{
    CV_Assert(sizeof(uchar) != img.depth());
    CV_Assert(1 == img.channels());
    CV_Assert(img.data != NULL);

    Mat imgTmp;
    img.copyTo(imgTmp);

    int nRow = imgTmp.rows;
    int nCol = imgTmp.cols;

    uchar *imgPtr;
    for(int i=1;i<nRow-1;i++){
        imgPtr = img.ptr<uchar>(i);
        for(int j=1;j<nCol;j++){
            uchar value = 0;
            if(imgTmp.at<uchar>(i,j+1)>imgPtr[j])
                value |= (2>>1);
            if(imgTmp.at<uchar>(i-1,j+1)>imgPtr[j])
                value |= (2);
            if(imgTmp.at<uchar>(i-1,j)>imgPtr[j])
                value |= (2<<1);
            if(imgTmp.at<uchar>(i-1,j-1)>imgPtr[j])
                value |= (2<<2);
            if(imgTmp.at<uchar>(i,j-1)>imgPtr[j])
                value |= (2<<3);
            if(imgTmp.at<uchar>(i+1,j-1)>imgPtr[j])
                value |= (2<<4);
            if(imgTmp.at<uchar>(i+1,j)>imgPtr[j])
                value |= (2<<5);
            if(imgTmp.at<uchar>(i+1,j+1)>imgPtr[j])
                value |= (2<<6);

            imgPtr[j] = value;
        }
    }
}

uchar minRotation(uchar &val)
{
    uchar minVal = val;

    for(int i=0;i<8;i++){
        val = move(val,1);
        if(minVal > val)
            minVal = val;
    }

    return minVal;
}

void getLBPRI(Mat &img)
{
    CV_Assert(sizeof(uchar) != img.depth());
    CV_Assert(1 == img.channels());
    CV_Assert(img.data != NULL);

    Mat imgTmp;
    img.copyTo(imgTmp);

    int nRow = imgTmp.rows;
    int nCol = imgTmp.cols;

    uchar *imgPtr;

    /*
     * first and last row
     */
    for(int j=0;j<nCol;j++){
        img.ptr<uchar>(0)[j] = 0;
        img.ptr<uchar>(nRow-1)[j] = 0;
    }

    /*
     * first and last column
     */
    for(int j=0;j<nRow;j++){
        img.ptr<uchar>(j)[0] = 0;
        img.ptr<uchar>(j)[nCol-1] = 0;
    }

    for(int i=1;i<nRow-1;i++){
        imgPtr = img.ptr<uchar>(i);
        for(int j=1;j<nCol;j++){
            uchar value = 0;
            if(imgTmp.at<uchar>(i,j+1) > imgPtr[j])
                value |= (2>>1);
            if(imgTmp.at<uchar>(i-1,j+1) > imgPtr[j])
                value |= (2);
            if(imgTmp.at<uchar>(i-1,j) > imgPtr[j])
                value |= (2<<1);
            if(imgTmp.at<uchar>(i-1,j-1) > imgPtr[j])
                value |= (2<<2);
            if(imgTmp.at<uchar>(i,j-1) > imgPtr[j])
                value |= (2<<3);
            if(imgTmp.at<uchar>(i+1,j-1) > imgPtr[j])
                value |= (2<<4);
            if(imgTmp.at<uchar>(i+1,j) > imgPtr[j])
                value |= (2<<5);
            if(imgTmp.at<uchar>(i+1,j+1) > imgPtr[j])
                value |= (2<<6);

            value = minRotation(value);
            imgPtr[j] = value;
        }
    }

}

