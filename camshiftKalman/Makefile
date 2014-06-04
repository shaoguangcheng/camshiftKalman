CC   = cl
CXX  = cl
LINK = link

###############################################################################
###############################################################################
DEFAULT_CFLAGS = -c -nologo -Zm200 -Zc:wchar_t- -O2 -MD -GR -EHsc -W3 -w34100 -w34189 \
				 -DUNICODE -DWIN32
DEFAULT_LDFLAGS= /NOLOGO /DYNAMICBASE /NXCOMPAT /INCREMENTAL:NO


###############################################################################
# run EXPORT PKG_CONFIG_PATH=/opt/OpenCV-2.4/lib/pkgconfig
###############################################################################
OPENCV_CFLAGS = 
OPENCV_LIBS   = opencv_calib3d248.lib opencv_contrib248.lib opencv_core248.lib \
				opencv_features2d248.lib opencv_flann248.lib opencv_gpu248.lib \
				opencv_highgui248.lib opencv_imgproc248.lib opencv_legacy248.lib \
				opencv_ml248.lib opencv_nonfree248.lib opencv_objdetect248.lib \
				opencv_ocl248.lib opencv_photo248.lib opencv_stitching248.lib \
				opencv_superres248.lib opencv_ts248.lib opencv_video248.lib \
				opencv_videostab248.lib 

###############################################################################
###############################################################################
CFLAGS  = $(OPENCV_CFLAGS) 
LDFLAGS = $(OPENCV_LIBS)

###############################################################################
###############################################################################
src-cxx-all := $(wildcard *.cpp)
src-c-all   := $(wildcard *.c)
bin-all     := $(patsubst %.cpp,%.exe,$(src-cxx-all)) $(patsubst %.c,%.exe,$(src-c-all))
%obj-all     := $(patsubst %.cpp,%.obj,$(src-cxx-all)) $(patsubst %.c,%.obj,$(src-c-all))

obj-all = main.obj LBP.obj camshiftkalman.obj util.obj

all : main.exe

main.exe : $(obj-all)
	$(LINK) $(DEFAULT_LDFLAGS) /out:$@ $(obj-all) $(LDFLAGS)


%.exe:%.obj
	$(LINK) $(DEFAULT_LDFLAGS) /out:$@ $? $(LDFLAGS)
	
%.obj:%.cpp
	$(CXX) $(DEFAULT_CFLAGS) $(CFLAGS) /Fo$@ $? 

%.obj:%.c
	$(CXX) $(DEFAULT_CFLAGS) $(CFLAGS) /Fo$@ $? 

clean :
	del *.exe *.obj

