/* This file is automatically generated by /home/bitcraze/crazyflie-firmware/tools/make/versionTemplate.py!
 * Do not edit manually, any manual change will be overwritten.
 */
/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * version.tmpl - version of the build
 */
#include <stdint.h>
#include <stdbool.h>

#include "config.h"
#include "param.h"

const char * V_SLOCAL_REVISION="70";
const char * V_SREVISION="39cf2a2bbb80";
const char * V_STAG="2023.02 +70";
const char * V_BRANCH="master";
const bool V_MODIFIED=false;
const bool V_PRODUCTION_RELEASE=false;

/* Version recoverable from the ground */
const uint32_t V_REVISION_0=0x39cf2a2b;
const uint16_t V_REVISION_1=0xbb80;

/**
 * Read-only parameters that describe the current quad firmware.
 */
PARAM_GROUP_START(firmware)

/**
 * @brief Byte `0 - 7` of firmware revision
 */
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, revision0, &V_REVISION_0)

/**
 * @brief Byte `8 - 11` of firmware revision
 */
PARAM_ADD_CORE(PARAM_UINT16 | PARAM_RONLY, revision1, &V_REVISION_1)

/**
 * @brief Nonzero if firmware has local changes
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, modified, &V_MODIFIED)

PARAM_GROUP_STOP(firmware)
