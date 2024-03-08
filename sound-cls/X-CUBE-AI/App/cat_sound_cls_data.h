/**
  ******************************************************************************
  * @file    cat_sound_cls_data.h
  * @author  AST Embedded Analytics Research Platform
  * @date    Thu Mar  7 20:09:30 2024
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

#ifndef CAT_SOUND_CLS_DATA_H
#define CAT_SOUND_CLS_DATA_H
#pragma once

#include "cat_sound_cls_config.h"
#include "cat_sound_cls_data_params.h"

AI_DEPRECATED
#define AI_CAT_SOUND_CLS_DATA_ACTIVATIONS(ptr_)  \
  ai_cat_sound_cls_data_activations_buffer_get(AI_HANDLE_PTR(ptr_))

AI_DEPRECATED
#define AI_CAT_SOUND_CLS_DATA_WEIGHTS(ptr_)  \
  ai_cat_sound_cls_data_weights_buffer_get(AI_HANDLE_PTR(ptr_))


AI_API_DECLARE_BEGIN


extern const ai_u64 s_cat_sound_cls_dense_dense_weights_array_u64[41608];
extern const ai_u64 s_cat_sound_cls_dense_dense_bias_array_u64[128];
extern const ai_u64 s_cat_sound_cls_dense_1_dense_weights_array_u64[2056];
extern const ai_u64 s_cat_sound_cls_dense_1_dense_bias_array_u64[64];
extern const ai_u64 s_cat_sound_cls_dense_2_dense_weights_array_u64[640];
extern const ai_u64 s_cat_sound_cls_dense_2_dense_bias_array_u64[5];



/*!
 * @brief Get network activations buffer initialized struct.
 * @ingroup cat_sound_cls_data
 * @param[in] ptr a pointer to the activations array storage area
 * @return an ai_buffer initialized struct
 */
AI_DEPRECATED
AI_API_ENTRY
ai_buffer ai_cat_sound_cls_data_activations_buffer_get(const ai_handle ptr);

/*!
 * @brief Get network weights buffer initialized struct.
 * @ingroup cat_sound_cls_data
 * @param[in] ptr a pointer to the weights array storage area
 * @return an ai_buffer initialized struct
 */
AI_DEPRECATED
AI_API_ENTRY
ai_buffer ai_cat_sound_cls_data_weights_buffer_get(const ai_handle ptr);

/*!
 * @brief Get network weights array pointer as a handle ptr.
 * @ingroup cat_sound_cls_data
 * @return a ai_handle pointer to the weights array
 */
AI_DEPRECATED
AI_API_ENTRY
ai_handle ai_cat_sound_cls_data_weights_get(void);


/*!
 * @brief Get network params configuration data structure.
 * @ingroup cat_sound_cls_data
 * @return true if a valid configuration is present, false otherwise
 */
AI_API_ENTRY
ai_bool ai_cat_sound_cls_data_params_get(ai_network_params* params);


AI_API_DECLARE_END

#endif /* CAT_SOUND_CLS_DATA_H */

