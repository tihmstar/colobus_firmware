//
//  macros.h
//  libgeneral
//
//  Created by tihmstar on 03.05.19.
//  Copyright © 2019 tihmstar. All rights reserved.
//

#ifndef macros_h
#define macros_h

#ifdef HAVE_CONFIG_H
#   include <config.h>
#endif //HAVE_CONFIG_H

#ifndef VERSION_COMMIT_COUNT
#   define VERSION_COMMIT_COUNT "VERSION_COMMIT_COUNT_not_set"
#endif
#ifndef VERSION_COMMIT_SHA
#   define VERSION_COMMIT_SHA "VERSION_COMMIT_SHA_not_set Build: " __DATE__ " " __TIME__
#endif

#ifndef PACKAGE_NAME
#define PACKAGE_NAME "PACKAGE_NAME_not_set"
#endif //PACKAGE_NAME

#ifndef VERSION_MAJOR
#define VERSION_MAJOR "0"
#endif //VERSION_MAJOR

#ifdef DEBUG
#define BUILD_TYPE "DEBUG"
#else
#define BUILD_TYPE "RELEASE"
#endif

#define VERSION_STRING PACKAGE_NAME " version: " VERSION_MAJOR "." VERSION_COMMIT_COUNT "-" VERSION_COMMIT_SHA "-" BUILD_TYPE


// ---- functions ----

// -- logging --
#      define info(a ...) ({printf(a),printf("\n");})
#      define warning(a ...) ({printf("[WARNING] "), printf(a),printf("\n");})
#      define error(a ...) ({printf("[Error] "),printf(a),printf("\n");})
#       ifdef DEBUG
#           define debug(a ...) ({printf("[DEBUG] "),printf(a),printf("\n");})
#       else
#           define debug(a ...)
#       endif


#define safeFree(ptr) ({if (ptr) {free(ptr); ptr=NULL;}})
#define safeFreeCustom(ptr,func) ({if (ptr) {func(ptr); ptr=NULL;}})
#define safeFreeConst(ptr) ({if(ptr){void *fbuf = (void*)ptr;ptr = NULL; free(fbuf);}})
#define safeClose(fd) ({if (fd != -1) {close(fd); fd=-1;}})
#define safeCloseCustom(fd,func) ({if (fd != -1) {func(fd); fd=-1;}})

#ifdef STRIP_ASSURES
#   define LIBGENERAL__FILE__ "<file>"
#   define LIBGENERAL_ERRSTR(errstr ...) "<errstr>"
#else //STRIP_ASSURES
#   define LIBGENERAL__FILE__ __FILE__
#   define LIBGENERAL_ERRSTR(errstr ...) errstr
#endif //STRIP_ASSURES

// -- assure --
#define cassure(a) do{ if ((a) == 0){err=__LINE__; goto error;} }while(0)
#define cretassure(cond, errstr ...) do{ if ((cond) == 0){err=__LINE__;error(LIBGENERAL_ERRSTR(errstr)); goto error;} }while(0)
#define creterror(errstr ...) do{error(LIBGENERAL_ERRSTR(errstr));err=__LINE__; goto error; }while(0)


#endif /* macros_h */