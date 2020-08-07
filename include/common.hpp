#pragma once

#define LINUX 1
#define DEBUG 1

//----------system operation----------
#ifdef LINUX
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#endif
#ifdef WIN32
#include <io.h>
#include <direct.h>
#endif

//-----------stl headers-------------------------
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <cstdlib>
#include <stdlib.h>
#include <string.h>
