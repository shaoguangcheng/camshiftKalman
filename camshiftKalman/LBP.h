#ifndef LBP_H
#define LBP_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>


using namespace cv;

/**
 * @brief getLBP calculate the LBP feature
 * @param img
 */
void getLBP(Mat &img);

/**
 * rotation shift
 * val
 * if n > 0 then left shift
 * if n < 0 then right shift
 */
template <class T> T move(T val, int n)
{
    const int N = sizeof(T)*8;
    if(n > 0)
        val = ((val<<n)|(val>>(N-n)));
    if(n < 0){
        n *= -1;
        val = ((val>>n)|(val<<(N-n)));
    }

    return val;
}

/**
 *  print binary format of val
 */
template<class T> void printBinary(T val)
{
    const int N = sizeof(T)*8;
    unsigned char *bits = new unsigned char [N];
    memset(bits, 0, sizeof(unsigned char)*N);
    for(int i=0;i<N;i++){
        bits[i] = val&1;
        val = val >> 1;
    }

    for(int i=N-1;i>=0;i--){
        printf("%d", bits[i]);
        if(i%4 == 0)
            printf(" ");
    }

    printf("\n");
}

/**
 * @brief minRotation calculate the minimal value of the rotation shift
 * @param val
 * @return
 */
uchar minRotation(uchar &val);

/**
 * @brief getLBPRI compute the rotation invariant LBP
 * @param img
 */
void getLBPRI(Mat &img);


#endif // LBP_H
