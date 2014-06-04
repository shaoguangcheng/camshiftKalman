#include "test.h"

#include "LBP.h"
#include "util.h"

void testGetLBP(int argc, char* argv[])
{
    if(argc < 2){
        printf("program [image name]\n");
        return;
    }

    Mat image = imread(string(argv[1]), IMREAD_GRAYSCALE);
    Mat img = imread(string(argv[1]), IMREAD_GRAYSCALE);

    getLBP(image);
    getLBPRI(img);

    imshow("LBP",image);
    imshow("LBPRI", img);

    imwrite("soccer_LBP.jpg",image);
    imwrite("soccer_LBPRI.jpg",img);

    int N = img.cols*img.rows;
    uchar *ptr = new uchar [N];
    int k = 0;
    for(int i=0;i<img.rows;i++)
        for(int j=0;j<img.cols;j++){
            ptr[k++] = img.at<uchar>(i,j);
        }

    int n = unique(ptr, N);
    for(int i=0;i<n;i++)
        printf("%d ", ptr[i]);
    printf("%d\n", n);

    waitKey();

    delete [] ptr;
}

void testBinary()
{
    uchar x = 28;
    printBinary(x);
    printf("x = %d\n", x);

    x = move(x, 5);
    printBinary(x);
    printf("x = %d\n", x);

    uchar p[256];
    for(uchar i=0;i<255;i++)
        p[i] = minRotation(i);

    int n = unique(p, 256);
    for(int i =0;i<n;i++)
        printf("%d ", p[i]);
}
