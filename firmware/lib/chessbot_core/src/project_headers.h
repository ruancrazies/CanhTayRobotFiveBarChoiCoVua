// The ONLY place the core reaches the project's include/ directory (config.h and protocol.h).
//
// Why a path relative to this file and not #include "config.h":
//  * PlatformIO does not put the project include/ directory on the include path of libraries in lib/;
//  * even where a -I flag adds it, PlatformIO appends project flags AFTER the framework's include
//    directories, so an SDK header that happens to be called config.h or protocol.h can be found first.
// A quoted include is first looked up relative to the directory of the file that contains it, before any
// -I directory, so this one cannot be shadowed and needs no build flag. It makes every core header
// self-contained. Change the path here if the directory layout ever changes (a test checks it resolves).
#ifndef CHESSBOT_PROJECT_HEADERS_H
#define CHESSBOT_PROJECT_HEADERS_H

#include "../../../include/config.h"
#include "../../../include/protocol.h"

#endif  // CHESSBOT_PROJECT_HEADERS_H
