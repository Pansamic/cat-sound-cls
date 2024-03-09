/**
  ******************************************************************************
  * @file    cat_sound_cls_data_params.h
  * @author  AST Embedded Analytics Research Platform
  * @date    Fri Mar  8 21:22:41 2024
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */

#ifndef CAT_SOUND_CLS_DATA_PARAMS_H
#define CAT_SOUND_CLS_DATA_PARAMS_H
#pragma once

#include "ai_platform.h"

/*
#define AI_CAT_SOUND_CLS_DATA_WEIGHTS_PARAMS \
  (AI_HANDLE_PTR(&ai_cat_sound_cls_data_weights_params[1]))
*/

#define AI_CAT_SOUND_CLS_DATA_CONFIG               (NULL)


#define AI_CAT_SOUND_CLS_DATA_ACTIVATIONS_SIZES \
  { 11424, }
#define AI_CAT_SOUND_CLS_DATA_ACTIVATIONS_SIZE     (11424)
#define AI_CAT_SOUND_CLS_DATA_ACTIVATIONS_COUNT    (1)
#define AI_CAT_SOUND_CLS_DATA_ACTIVATION_1_SIZE    (11424)



#define AI_CAT_SOUND_CLS_DATA_WEIGHTS_SIZES \
  { 356008, }
#define AI_CAT_SOUND_CLS_DATA_WEIGHTS_SIZE         (356008)
#define AI_CAT_SOUND_CLS_DATA_WEIGHTS_COUNT        (1)
#define AI_CAT_SOUND_CLS_DATA_WEIGHT_1_SIZE        (356008)



#define AI_CAT_SOUND_CLS_DATA_ACTIVATIONS_TABLE_GET() \
  (&g_cat_sound_cls_activations_table[1])

extern ai_handle g_cat_sound_cls_activations_table[1 + 2];



#define AI_CAT_SOUND_CLS_DATA_WEIGHTS_TABLE_GET() \
  (&g_cat_sound_cls_weights_table[1])

extern ai_handle g_cat_sound_cls_weights_table[1 + 2];


#endif    /* CAT_SOUND_CLS_DATA_PARAMS_H */
