/* SPDX-License-Identifier: GPL-3.0-only */
/* The engine's one libc header, for the freestanding WebAssembly build: the SDK (vibe_sdk.c) has memset and memcpy. */
#pragma once
#include <stddef.h>
void *memset(void *d, int c, size_t n);
void *memcpy(void *d, const void *s, size_t n);
