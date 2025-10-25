#!/bin/sh
set -x
${CLANG_FORMAT:-clang-format} -i --style=file:rocketlib/.clang-format Core/Src/health_check.c Core/Src/ov5640.c Core/Src/platform.c Core/Src/video.c Core/Inc/health_check.h Core/Inc/ov5640.h Core/Inc/platform.h Core/Inc/video.h
