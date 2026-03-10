#pragma once

#include <stdint.h>

#ifndef ROBOT_PROFILE_NAME
#define ROBOT_PROFILE_NAME "dev"
#endif

#ifndef ROBOT_CONFIG_VERSION
#define ROBOT_CONFIG_VERSION "2026.03-base"
#endif

#include "robot_config_platform.h"
#include "robot_config_control.h"
#include "robot_config_safety.h"
#include "robot_config_link.h"

#if defined(ROBOT_PROFILE_DEV)
#include "robot_profile_dev.h"
#elif defined(ROBOT_PROFILE_PROD)
#include "robot_profile_prod.h"
#elif defined(ROBOT_PROFILE_VEHICLE_A)
#include "robot_profile_vehicle_a.h"
#else
#error "Unknown ROBOT_PROFILE selection. Please set -DROBOT_PROFILE=dev|prod|vehicle_a"
#endif
