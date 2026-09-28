#pragma once

#if defined(_WIN32)
#if defined(MEDIAPIPE2MANORUNTIME_BUILD)
#define M2M_RUNTIME_API __declspec(dllexport)
#else
#define M2M_RUNTIME_API __declspec(dllimport)
#endif
#else
#define M2M_RUNTIME_API
#endif
