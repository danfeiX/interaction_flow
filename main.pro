DEFINES += _USE_MATH_DEFINES

CONFIG += console

QT += core opengl xml
OPENCV_LIB
!win32 {
QMAKE_CXXFLAGS += -std=c++11
QMAKE_CXXFLAGS_WARN_ON += -Wall
QMAKE_CXXFLAGS_WARN_ON += -Wall
}

win32 {
QMAKE_CXXFLAGS_RELEASE += /Zi
QMAKE_LFLAGS_RELEASE += /DEBUG
QMAKE_CXXFLAGS_WARN_ON += /W0
QMAKE_CXXFLAGS_WARN_ON += /W0

QMAKE_CXXFLAGS += -MP8
QMAKE_LFLAGS += -LARGEADDRESSAWARE

QMAKE_CFLAGS_RELEASE += /MD
QMAKE_CXXFLAGS_RELEASE += /MD
QMAKE_CFLAGS_DEBUG += /MDd
QMAKE_CXXFLAGS_DEBUG += /MDd
}

INCLUDEPATH += \
$(CUDA_PATH)\\include \
Include \
Include\\CUDA \
$(GLEW_INCLUDE_PATH) \
$(FREEGLUT_INCLUDE_PATH) \
$(ASSIMP_INCLUDE_PATH) \
Include\\CUDA \
$(OPENCV_INCLUDE_PATH) \
$(KINECT_INCLUDE_PATH) \
$(FREENECT2_INCLUDE_PATH) \
$(ARUCO_DIR)\\include \
$(LIBREALSENSE_DIR)\\include \

CONFIG(release, debug | release){
LIBS += \
$(CUDA_PATH)\\lib\\$(PlatformName)\\cudart.lib \
$(OPENCV_DIR)\\lib\\opencv_core2410.lib \
$(OPENCV_DIR)\\lib\\opencv_imgproc2410.lib \
$(OPENCV_DIR)\\lib\\opencv_highgui2410.lib \
$(OPENCV_DIR)\\lib\\opencv_contrib2410.lib \
$(OPENCV_DIR)\\lib\\opencv_calib3d2410.lib \
$(OPENCV_DIR)\\lib\\opencv_flann2410.lib \
$(OPENCV_DIR)\\lib\\opencv_gpu2410.lib \
$(FREENECT2_LIB_PATH)\\freenect2.lib \
$(ARUCO_DIR)\\lib\\aruco130.lib \
$(LIBREALSENSE_DIR)\\bin\\x64\\realsense.lib\
}
else {
LIBS += \
$(CUDA_PATH)\\lib\\$(PlatformName)\\cudart.lib \
$(OPENCV_DIR)\\lib\\opencv_core2410d.lib \
$(OPENCV_DIR)\\lib\\opencv_imgproc2410d.lib \
$(OPENCV_DIR)\\lib\\opencv_highgui2410d.lib \
$(OPENCV_DIR)\\lib\\opencv_contrib2410d.lib \
$(OPENCV_DIR)\\lib\\opencv_calib3d2410d.lib \
$(OPENCV_DIR)\\lib\\opencv_flann2410d.lib \
$(OPENCV_DIR)\\lib\\opencv_gpu2410d.lib \
$(FREENECT2_LIB_PATH)\\freenect2.lib \
$(ARUCO_DIR)\\lib\\aruco130.lib \
$(LIBREALSENSE_DIR)\\bin\\x64\\realsense.lib\
}

TARGET = Main

RESOURCES = main.qrc

HEADERS +=  \
include\\*.h \

SOURCES +=  \
main.cpp \
src\\*.cpp \
src\\*.cu \
