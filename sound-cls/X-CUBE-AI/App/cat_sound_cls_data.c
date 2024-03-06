/**
  ******************************************************************************
  * @file    cat_sound_cls_data.c
  * @author  AST Embedded Analytics Research Platform
  * @date    Tue Mar  5 23:13:53 2024
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */
#include "cat_sound_cls_data.h"
#include "ai_platform_interface.h"

AI_API_DECLARE_BEGIN
ai_buffer g_cat_sound_cls_data_map_activations[AI_CAT_SOUND_CLS_DATA_ACTIVATIONS_COUNT] = {
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 11424, 1, 1),
    11424, NULL, NULL),    /* heap_overlay_pool */
  };
ai_buffer g_cat_sound_cls_data_map_weights[AI_CAT_SOUND_CLS_DATA_WEIGHTS_COUNT] = {
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 332864, 1, 1),
    332864, NULL, s_cat_sound_cls_dense_dense_weights_array_u64),   /* dense_dense_weights_array */
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 1024, 1, 1),
    1024, NULL, s_cat_sound_cls_dense_dense_bias_array_u64),   /* dense_dense_bias_array */
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 16448, 1, 1),
    16448, NULL, s_cat_sound_cls_dense_1_dense_weights_array_u64),   /* dense_1_dense_weights_array */
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 512, 1, 1),
    512, NULL, s_cat_sound_cls_dense_1_dense_bias_array_u64),   /* dense_1_dense_bias_array */
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 5120, 1, 1),
    5120, NULL, s_cat_sound_cls_dense_2_dense_weights_array_u64),   /* dense_2_dense_weights_array */
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 40, 1, 1),
    40, NULL, s_cat_sound_cls_dense_2_dense_bias_array_u64),   /* dense_2_dense_bias_array */
  };


/*!
 * @brief Get network activations buffer initialized struct.
 * @ingroup cat_sound_cls_data
 * @param[in] ptr a pointer to the activations array storage area
 * @return an ai_buffer initialized struct
 */
AI_DEPRECATED
AI_API_ENTRY
ai_buffer ai_cat_sound_cls_data_activations_buffer_get(const ai_handle ptr)
{
  ai_buffer buf = AI_BUFFER_INIT(
    AI_FLAG_NONE, AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, AI_CAT_SOUND_CLS_DATA_ACTIVATIONS_SIZE, 1, AI_CAT_SOUND_CLS_DATA_ACTIVATIONS_COUNT),
    AI_CAT_SOUND_CLS_DATA_ACTIVATIONS_SIZE,
    NULL, ptr);
  return buf;
}

/*!
 * @brief Get network weights buffer initialized struct.
 * @ingroup cat_sound_cls_data
 * @param[in] ptr a pointer to the weights array storage area
 * @return an ai_buffer initialized struct
 */
AI_DEPRECATED
AI_API_ENTRY
ai_buffer ai_cat_sound_cls_data_weights_buffer_get(const ai_handle ptr)
{
  ai_buffer buf = AI_BUFFER_INIT(
    AI_FLAG_NONE, AI_BUFFER_FORMAT_U8|AI_BUFFER_FMT_FLAG_CONST,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, AI_CAT_SOUND_CLS_DATA_WEIGHTS_SIZE, 1, AI_CAT_SOUND_CLS_DATA_WEIGHTS_COUNT),
    AI_CAT_SOUND_CLS_DATA_WEIGHTS_SIZE,
    NULL, ptr);
  return buf;
}


/*!
 * @brief Get network weights array pointer as a handle ptr.
 * @ingroup cat_sound_cls_data
 * @return a ai_handle pointer to the weights array
 */
AI_DEPRECATED
AI_API_ENTRY
ai_handle ai_cat_sound_cls_data_weights_get(void)
{
  return AI_HANDLE_PTR(g_cat_sound_cls_weights_table);

}


/*!
 * @brief Get network params configuration data structure.
 * @ingroup cat_sound_cls_data
 * @return true if a valid configuration is present, false otherwise
 */
AI_API_ENTRY
ai_bool ai_cat_sound_cls_data_params_get(ai_network_params* params)
{
  if (!params) return false;
  
  const ai_buffer_array map_activations = 
    AI_BUFFER_ARRAY_OBJ_INIT(AI_FLAG_NONE, AI_CAT_SOUND_CLS_DATA_ACTIVATIONS_COUNT, g_cat_sound_cls_data_map_activations);
  
  const ai_buffer_array map_weights = 
    AI_BUFFER_ARRAY_OBJ_INIT(AI_FLAG_NONE, AI_CAT_SOUND_CLS_DATA_WEIGHTS_COUNT, g_cat_sound_cls_data_map_weights);

  return ai_platform_bind_network_params(params, &map_weights, &map_activations);
}


AI_API_DECLARE_END
