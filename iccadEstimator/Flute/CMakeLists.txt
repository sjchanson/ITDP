SET(FLUTE_HOME ${PROJECT_SOURCE_DIR}/Flute)
set(POWV9_DAT ${FLUTE_HOME}/POWV9.dat)
set(PORT9_DAT ${FLUTE_HOME}/PORT9.dat)

add_library(flute SHARED 
            flute_new.cpp)