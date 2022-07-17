#ifndef UTILS_H
#define UTILS_H

#define GCC_COMPILER_DETECTED (defined(__GNUC__) && !defined(__clang__))
#define CLANG_COMPILER_DETECTED (!GCC_COMPILER_DETECTED && defined(__clang__))

#define concat_(a, b) a##b
#define label(prefix, lnum) concat_(prefix, lnum)

#endif