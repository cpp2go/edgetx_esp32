/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * ESP-IDF bundles FatFs R0.15+ which renamed the directory object type from
 * the legacy "DIR" to "FF_DIR". The EdgeTX radio code (file browsers, FTP
 * server, storage, ...) still uses the old "DIR" name.
 *
 * On ESP32 targets the EdgeTX-bundled FatFs is not compiled and ESP-IDF's
 * FatFs is used instead (see radio/src/hal/CMakeLists.txt). This header maps
 * DIR -> FF_DIR so all legacy FatFs user code keeps compiling.
 */

#ifndef _FF_COMPAT_H_
#define _FF_COMPAT_H_

#include "ff.h"

#if defined(FF_FS_MINIMIZE) && (FF_FS_MINIMIZE <= 1)
typedef FF_DIR DIR;
#endif

#endif /* _FF_COMPAT_H_ */
