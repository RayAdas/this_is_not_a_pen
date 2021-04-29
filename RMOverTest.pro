TEMPLATE = app
CONFIG += console c++11
CONFIG += c++11
CONFIG -= app_bundle
CONFIG -= qt
QMAKE_CXXFLAGS += -std=c++0x
QMAKE_CXXFLAGS += -D_GLIBCXX_USE_CXX11_ABI=0
LIBS += /usr/lib/libopencv_imgproc.so \
        /usr/lib/libopencv_highgui.so \
        /usr/lib/libopencv_core.so \
        /usr/lib/libopencv_imgcodecs.so \
        /usr/lib/libopencv_video.so \
        /usr/lib/libopencv_videoio.so \
        /usr/lib/libopencv_calib3d.so \
        /usr/lib/libopencv_ml.so \
        /usr/lib/aarch64-linux-gnu/libpthread.so \
        #/usr/lib/x86_64-linux-gnu/libpthread.so \

INCLUDEPATH+=/opt/GALAXY
INCLUDEPATH += /opt/DahuaTech/DependsInclude/Include
INCLUDEPATH += /opt/DahuaTech/DependsInclude/Depends
DEPENDPATH += /opt/DahuaTech/DependsInclude/Depends

win32:CONFIG(release, debug|release): LIBS += -L/opt/DahuaTech/DependsInclude/Depends/ -lImageConvert
else:win32:CONFIG(debug, debug|release): LIBS += -L/opt/DahuaTech/DependsInclude/Depends/ -lImageConvert
else:unix: LIBS += -L/opt/DahuaTech/DependsInclude/Depends/ -lImageConvert

win32:CONFIG(release, debug|release): LIBS += -L/opt/DahuaTech/DependsInclude/Depends/ -lMVSDK
else:win32:CONFIG(debug, debug|release): LIBS += -L/opt/DahuaTech/DependsInclude/Depends/ -lMVSDK
else:unix: LIBS += -L/opt/DahuaTech/DependsInclude/Depends/ -lMVSDK

#win32:CONFIG(release, debug|release): LIBS += -L/opt/GALAXY/ -lgxiapi -lpthread
#else:win32:CONFIG(debug, debug|release): LIBS += -L/opt/GALAXY/ -lgxiapi -lpthread
#else:unix: LIBS += -L/opt/GALAXY/ -lgxiapi -lpthread

#INCLUDEPATH += $$PWD/Depends
#DEPENDPATH += $$PWD/Depends

#win32:CONFIG(release, debug|release): LIBS += -L$$PWD/Depends/ -lImageConvert
#else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/Depends/ -lImageConvert
#else:unix: LIBS += -L$$PWD/Depends/ -lImageConvert

#win32:CONFIG(release, debug|release): LIBS += -L$$PWD/Depends/ -lMVSDK
#else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/Depends/ -lMVSDK
#else:unix: LIBS += -L$$PWD/Depends/ -lMVSDK
SOURCES += \
    serialPort/serialPort.cpp \
    coordinateTransform/coordinateTransform.cpp \
    targetModel/robot.cpp \
    targetModel/buff.cpp \
    targetModel/targetModel.cpp \
    #targetModel/criticalcore.cpp \
    videoSource/camera.cpp \
    main.cpp \
    tool/tool.cpp \
    tool/kalman.cpp \
    controller.cpp \
    tool/triplebuffering.cpp \
    serialPort/pmbcbspkeeper.cpp \
    trajectoryCalculation.cpp \
    #videoSource/galaxycamera.cpp \
    videoSource/videosource.cpp \

HEADERS += \
    serialPort/serialPort.h \
    coordinateTransform/coordinateTransform.h \
    targetModel/robot.h \
    targetModel/buff.h \
    targetModel/targetModel.h \
    #targetModel/criticalcore.h \
    videoSource/camera.h \
    tool/tool.h \
    tool/kalman.h \
    controller.h \
    tool/triplebuffering.h \
    serialPort/pmbcbspkeeper.h \
    trajectoryCalculation.h \
    #videoSource/galaxycamera.h \
    preferences.h \
    videoSource/videosource.h \

DISTFILES +=
