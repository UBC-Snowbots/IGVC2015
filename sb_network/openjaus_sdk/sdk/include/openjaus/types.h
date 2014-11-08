// File Header Here

#ifndef OPENJAUS_TYPES_H
#define OPENJAUS_TYPES_H

#if defined(__linux) || defined(linux) || defined(__linux__) || defined(__APPLE__) || defined(__QNX__)
	#include <stdint.h>
	#define OPENJAUS_EXPORT
#else
	typedef signed char	int8_t;
	typedef short int	int16_t;
	typedef int			int32_t;
	typedef long int	int64_t;

	typedef unsigned char		uint8_t;
	typedef unsigned short int	uint16_t;
	typedef unsigned int		uint32_t;
	typedef unsigned long int	uint64_t;

	#define OPENJAUS_EXPORT	__declspec(dllexport)
#endif


// Type Definitions
typedef int8_t int8;
typedef int16_t int16;
typedef int32_t int32;
typedef int64_t int64;
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;

#endif // OPENJAUS_TYPES_H
