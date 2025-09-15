// Monitoring configuration (Serial output)
#pragma once

// Enable/disable monitoring output task
#ifndef SERIAL_DATA_ENABLE
#define SERIAL_DATA_ENABLE 1
#endif

// Debug prints control (maps DEBUG* macros)
#ifndef SERIAL_DEBUG_ENABLE
#define SERIAL_DEBUG_ENABLE 1
#endif
#undef DEBUG_ENABLED
#define DEBUG_ENABLED (SERIAL_DEBUG_ENABLE)

// Monitoring mode: 0=Visualizer (key:value), 1=Human (fixed width)
#ifndef MON_MODE
#define MON_MODE 0
#endif

// Include timestamp in monitoring lines
#ifndef MON_INCLUDE_TS
#define MON_INCLUDE_TS 1
#endif

// Show fusion sub-values (components used by fused outputs)
#ifndef MON_SHOW_FUSION_PARTS
#define MON_SHOW_FUSION_PARTS 1
#endif
