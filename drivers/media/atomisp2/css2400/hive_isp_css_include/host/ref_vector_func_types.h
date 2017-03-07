/*
 * INTEL CONFIDENTIAL
 *
 * Copyright (C) 2010 - 2013 Intel Corporation.
 * All Rights Reserved.
 *
 * The source code contained or described herein and all documents
 * related to the source code ("Material") are owned by Intel Corporation
 * or licensors. Title to the Material remains with Intel
 * Corporation or its licensors. The Material contains trade
 * secrets and proprietary and confidential information of Intel or its
 * licensors. The Material is protected by worldwide copyright
 * and trade secret laws and treaty provisions. No part of the Material may
 * be used, copied, reproduced, modified, published, uploaded, posted,
 * transmitted, distributed, or disclosed in any way without Intel's prior
 * express written permission.
 *
 * No License under any patent, copyright, trade secret or other intellectual
 * property right is granted to or conferred upon you by disclosure or
 * delivery of the Materials, either expressly, by implication, inducement,
 * estoppel or otherwise. Any license under such intellectual property rights
 * must be express and approved by Intel in writing.
 */


#ifndef __REF_VECTOR_FUNC_TYPES_H_INCLUDED__
#define __REF_VECTOR_FUNC_TYPES_H_INCLUDED__


/*
 * Prerequisites:
 *
 */
#include "mpmath.h"
#include "isp_op1w_types.h"
#include "isp_op2w_types.h"

/*
 * Struct type specification
 */

typedef unsigned short tscalar1w_3bit;
typedef unsigned short tvector_5bit;
typedef unsigned short tvector_4bit;
typedef unsigned short tvector1w_unsigned;
typedef unsigned int   tvector2w_unsigned;

typedef struct {
  tvector1w     v0  ;
  tvector1w     v1 ;
} s_1w_2x1_matrix;

typedef struct {
  tvector1w     v00  ;
  tvector1w     v01 ;
  tvector1w     v02 ;
} s_1w_1x3_matrix;

typedef struct {
  tvector1w     v00  ; tvector1w     v01 ; tvector1w     v02  ;
  tvector1w     v10  ; tvector1w     v11 ; tvector1w     v12  ;
  tvector1w     v20  ; tvector1w     v21 ; tvector1w     v22  ;
} s_1w_3x3_matrix;

typedef struct {
  tvector1w     v00  ; tvector1w     v01 ; tvector1w     v02  ;
  tvector1w     v10  ; tvector1w     v11 ; tvector1w     v12  ;
  tvector1w     v20  ; tvector1w     v21 ; tvector1w     v22  ;
  tvector1w     v30  ; tvector1w     v31 ; tvector1w     v32  ;
} s_1w_4x3_matrix;

typedef struct {
  tvector1w     v00 ;
  tvector1w     v01 ;
  tvector1w     v02 ;
  tvector1w     v03 ;
  tvector1w     v04 ;
} s_1w_1x5_matrix;

typedef struct {
  tvector1w     v00  ; tvector1w     v01 ; tvector1w     v02  ; tvector1w     v03 ; tvector1w     v04  ;
  tvector1w     v10  ; tvector1w     v11 ; tvector1w     v12  ; tvector1w     v13 ; tvector1w     v14  ;
  tvector1w     v20  ; tvector1w     v21 ; tvector1w     v22  ; tvector1w     v23 ; tvector1w     v24  ;
  tvector1w     v30  ; tvector1w     v31 ; tvector1w     v32  ; tvector1w     v33 ; tvector1w     v34  ;
  tvector1w     v40  ; tvector1w     v41 ; tvector1w     v42  ; tvector1w     v43 ; tvector1w     v44  ;
} s_1w_5x5_matrix;


#endif /* __REF_VECTOR_FUNC_TYPES_H_INCLUDED__ */
