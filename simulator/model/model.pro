TEMPLATE = lib

QT -= gui
QT += xml

DEFINES += MODEL_LIB

TARGET = model

DESTDIR = ../../../lib

CONFIG(debug, debug|release) {

    TARGET = $$join(TARGET,,,_d)

    LIBS += -L../../../lib -lfilesystem_d
    LIBS += -L../../../lib -llog_d
    LIBS += -L../../../lib -lCfgReader_d
    LIBS += -L../../../lib -lphysics_d
    LIBS += -L../../../lib -lvehicle_d
    LIBS += -L../../../lib -lcoupling_d
    LIBS += -L../../../lib -lsolver_d
    LIBS += -L../../../lib -ltrain_d

} else {

    LIBS += -L../../../lib -lfilesystem
    LIBS += -L../../../lib -llog
    LIBS += -L../../../lib -lCfgReader
    LIBS += -L../../../lib -lphysics
    LIBS += -L../../../lib -lvehicle
    LIBS += -L../../../lib -lcoupling
    LIBS += -L../../../lib -lsolver
    LIBS += -L../../../lib -ltrain
}

INCLUDEPATH += ./include
INCLUDEPATH += ../common-headers
INCLUDEPATH += ../train/include
INCLUDEPATH += ../solver/include
INCLUDEPATH += ../vehicle/include
INCLUDEPATH += ../coupling/include
INCLUDEPATH += ../../CfgReader/include
INCLUDEPATH += ../../filesystem/include
INCLUDEPATH += ../../log/include

HEADERS += $$files(./include/*.h)
HEADERS += $$files(../common-headers/*.h)
SOURCES += $$files(./src/*.cpp)