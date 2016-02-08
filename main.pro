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

CONFIG(release, debug | release){
LIBS += \
$(CUDA_PATH)\\lib\\$(PlatformName)\\cudart.lib \
$(GLEW_LIB_PATH)\\glew32.lib \
$(FREEGLUT_LIB_PATH)\\Release\\freeglut.lib \
$(ASSIMP_LIB_PATH)\\Release\\assimp-vc120-mt.lib \
$(KINECT_LIB_PATH)\\Kinect20.lib \
$(OPENCV_DIR)\\lib\\opencv_core2410.lib \
$(OPENCV_DIR)\\lib\\opencv_imgproc2410.lib \
$(OPENCV_DIR)\\lib\\opencv_highgui2410.lib \
$(OPENCV_DIR)\\lib\\opencv_contrib2410.lib \
$(FREENECT2_LIB_PATH)\\freenect2.lib \
}
else {
LIBS += \
$(CUDA_PATH)\\lib\\$(PlatformName)\\cudart.lib \
$(GLEW_LIB_PATH)\\glew32.lib \
$(FREEGLUT_LIB_PATH)\\Debug\\freeglutd.lib \
$(ASSIMP_LIB_PATH)\\Debug\\assimp-vc120-mtd.lib \
$(KINECT_LIB_PATH)\\Kinect20.lib \
$(OPENCV_DIR)\\lib\\opencv_core2410d.lib \
$(OPENCV_DIR)\\lib\\opencv_imgproc2410d.lib \
$(OPENCV_DIR)\\lib\\opencv_highgui2410d.lib \
$(OPENCV_DIR)\\lib\\opencv_contrib2410d.lib \
$(FREENECT2_LIB_PATH)\\freenect2.lib \
}

TARGET = Main

RESOURCES = main.qrc

HEADERS +=  \
*.h \
kinect\\kinect_interface.h \

SOURCES +=  \
*.cpp \
main.cpp \
*.cu \
kinect\\kinect_interface.cpp \
