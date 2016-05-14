/*
 * Mutex.cpp
 *
 *  Created on: 02/09/2011
 *      Author: Ignacio Mellado
 *      Description: This class implements a recursive mutex
 *      Notes: The class only supports Linux by now.
 */

#include <cvgMutex.h>

#ifdef PTHREAD_MUTEX_RECURSIVE_NP
#define MUTEX_RECURSIVE_FLAG PTHREAD_MUTEX_RECURSIVE_NP
#else	// In OS X, the constant is defined without the trailing _NP
#define MUTEX_RECURSIVE_FLAG PTHREAD_MUTEX_RECURSIVE
#endif
 
cvgMutex::cvgMutex(cvg_bool recursive) {
	pthread_mutexattr_t mutexAttr;
	if (recursive) {
		if (pthread_mutexattr_init(&mutexAttr) != 0)
			throw cvgException("[Mutex] Unable to initialize mutex attributes");
		if (pthread_mutexattr_settype(&mutexAttr, MUTEX_RECURSIVE_FLAG) != 0) {
			pthread_mutexattr_destroy(&mutexAttr);
			throw cvgException("[Mutex] Unable to set mutex type to recursive");
		}
	}
	if (pthread_mutex_init(&handler, recursive ? &mutexAttr : NULL) != 0) {
		if (recursive) pthread_mutexattr_destroy(&mutexAttr);
		throw cvgException("[Mutex] Unable to create mutex");
	}
	if (recursive) pthread_mutexattr_destroy(&mutexAttr);
}

cvgMutex::~cvgMutex() {
	pthread_mutex_destroy(&handler);
}

cvg_bool cvgMutex::lock() {
	return pthread_mutex_lock(&handler) == 0;
}

cvg_bool cvgMutex::unlock() {
	return pthread_mutex_unlock(&handler) == 0;
}


