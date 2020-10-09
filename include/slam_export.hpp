#ifndef SLAM_EXPORT_H
#define SLAM_EXPORT_H

#ifdef slam_BUILT_AS_STATIC
#  define SLAM_EXPORT
#  define SLAM_CORE_NO_EXPORT
#else
#  ifndef SLAM_EXPORT
#    ifdef slam_core_EXPORTS
        /* We are building this library */
#      define SLAM_EXPORT 
#    else
        /* We are using this library */
#      define SLAM_EXPORT 
#    endif
#  endif

#  ifndef SLAM_CORE_NO_EXPORT
#    define SLAM_CORE_NO_EXPORT 
#  endif
#endif

#ifndef SLAM_CORE_DEPRECATED
#  define SLAM_CORE_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef SLAM_CORE_DEPRECATED_EXPORT
#  define SLAM_CORE_DEPRECATED_EXPORT SLAM_EXPORT SLAM_CORE_DEPRECATED
#endif

#ifndef SLAM_CORE_DEPRECATED_NO_EXPORT
#  define SLAM_CORE_DEPRECATED_NO_EXPORT SLAM_CORE_NO_EXPORT SLAM_CORE_DEPRECATED
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define SLAM_CORE_NO_DEPRECATED
#endif

#endif
