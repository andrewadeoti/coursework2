TEMPLATE = app
CONFIG += console c++20
CONFIG -= app_bundle
CONFIG -= qt

QMAKE_CXXFLAGS += -std=c++20 -Wall -Wfatal-errors

OBJECTS_DIR = $$_PRO_FILE_PWD_/bin/
DESTDIR = $$_PRO_FILE_PWD_/bin/
TARGET = track-tests

LIBS += -lboost_unit_test_framework

INCLUDEPATH += headers/ headers/gpx/ headers/xml/

HEADERS += \
    headers/earth.h \
    headers/geometry.h \
    headers/track.h \
    headers/trackpoint.h \
    headers/types.h \
    headers/waypoint.h \
    headers/gpx/gpx-parser.h \
    headers/xml/xml-element.h \
    headers/xml/xml-parser.h

SOURCES += \
    src/earth.cpp \
    src/geometry.cpp \
    src/track.cpp \
    src/waypoint.cpp \
    src/gpx/gpx-parser.cpp \
    src/xml/xml-element.cpp \
    src/xml/xml-parser.cpp

SOURCES += \
    tests/track-tests.cpp

OTHER_FILES += \
    data/NorthYorkMoors.gpx
