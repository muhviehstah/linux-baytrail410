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


#ifndef _REF_VECTOR_FUNC_H_INCLUDED_
#define _REF_VECTOR_FUNC_H_INCLUDED_

#ifndef STORAGE_CLASS_REF_VECTOR_FUNC_C
#define STORAGE_CLASS_REF_VECTOR_FUNC_C extern
#endif

#include "ref_vector_func_types.h"

/** @brief Normalised FIR with coefficients [3,4,1]
 *
 * @param[in] m	1x3 matrix with pixels
 *
 * @return		filtered output
 *
 * This function will calculate the
 * Normalised FIR with coefficients [3,4,1],
 *-5dB at Fs/2, -90 degree phase shift (quarter pixel)
 */
STORAGE_CLASS_REF_VECTOR_FUNC_C tvector1w fir1x3m_5dB_m90_nrm (
	const s_1w_1x3_matrix		m);

/** @brief Normalised FIR with coefficients [1,4,3]
 *
 * @param[in] m	1x3 matrix with pixels
 *
 * @return		filtered output
 *
 * This function will calculate the
 * Normalised FIR with coefficients [1,4,3],
 *-5dB at Fs/2, +90 degree phase shift (quarter pixel)
 */
STORAGE_CLASS_REF_VECTOR_FUNC_C tvector1w fir1x3m_5dB_p90_nrm (
	const s_1w_1x3_matrix		m);

/** @brief Normalised FIR with coefficients [1,2,1]
 *
 * @param[in] m	1x3 matrix with pixels
 *
 * @return		filtered output
 *
 * This function will calculate the
 * Normalised FIR with coefficients [1,2,1], -6dB at Fs/2
 */
STORAGE_CLASS_REF_VECTOR_FUNC_C tvector1w fir1x3m_6dB_nrm (
	const s_1w_1x3_matrix		m);

/** @brief Normalised FIR with coefficients [13,16,3]
 *
 * @param[in] m	1x3 matrix with pixels
 *
 * @return		filtered output
 *
 * This function will calculate the
 * Normalised FIR with coefficients [13,16,3],
 */
STORAGE_CLASS_REF_VECTOR_FUNC_C tvector1w fir1x3m_6dB_nrm_ph0 (
	const s_1w_1x3_matrix		m);

/** @brief Normalised FIR with coefficients [9,16,7]
 *
 * @param[in] m	1x3 matrix with pixels
 *
 * @return		filtered output
 *
 * This function will calculate the
 * Normalised FIR with coefficients [9,16,7],
 */
STORAGE_CLASS_REF_VECTOR_FUNC_C tvector1w fir1x3m_6dB_nrm_ph1 (
	const s_1w_1x3_matrix		m);

/** @brief Normalised FIR with coefficients [5,16,11]
 *
 * @param[in] m	1x3 matrix with pixels
 *
 * @return		filtered output
 *
 * This function will calculate the
 * Normalised FIR with coefficients [5,16,11],
 */
STORAGE_CLASS_REF_VECTOR_FUNC_C tvector1w fir1x3m_6dB_nrm_ph2 (
	const s_1w_1x3_matrix		m);

/** @brief Normalised FIR with coefficients [1,16,15]
 *
 * @param[in] m	1x3 matrix with pixels
 *
 * @return		filtered output
 *
 * This function will calculate the
 * Normalised FIR with coefficients [1,16,15],
 */
STORAGE_CLASS_REF_VECTOR_FUNC_C tvector1w fir1x3m_6dB_nrm_ph3 (
	const s_1w_1x3_matrix		m);

/** @brief Normalised FIR with programable phase shift
 *
 * @param[in] m	1x3 matrix with pixels
 * @param[in] coeff	phase shift
 *
 * @return		filtered output
 *
 * This function will calculate the
 * Normalised FIR with coefficients [8-coeff,16,8+coeff],
 */
STORAGE_CLASS_REF_VECTOR_FUNC_C tvector1w fir1x3m_6dB_nrm_calc_coeff (
	const s_1w_1x3_matrix		m, tscalar1w_3bit coeff);

/** @brief 3 tab FIR with coefficients [1,1,1]
 *
 * @param[in] m	1x3 matrix with pixels
 *
 * @return		filtered output
 *
 * This function will calculate the
 * FIR with coefficients [1,1,1], -9dB at Fs/2 normalized with factor 1/2
 */
STORAGE_CLASS_REF_VECTOR_FUNC_C tvector1w fir1x3m_9dB_nrm (
	const s_1w_1x3_matrix		m);


/** @brief Normalised 2D FIR with coefficients  [1;2;1] * [1,2,1]
 *
 * @param[in] m	3x3 matrix with pixels
 *
 * @return		filtered output
 *
 * This function will calculate the
 * Normalised FIR with coefficients  [1;2;1] * [1,2,1]
 * Unity gain filter through repeated scaling and rounding
 *	- 6 rotate operations per output
 *	- 8 vector operations per output
 * _______
 *   14 total operations
 */
STORAGE_CLASS_REF_VECTOR_FUNC_C tvector1w fir3x3m_6dB_nrm (
	const s_1w_3x3_matrix		m);

/** @brief Normalised 2D FIR with coefficients  [1;1;1] * [1,1,1]
 *
 * @param[in] m	3x3 matrix with pixels
 *
 * @return		filtered output
 *
 * This function will calculate the
 * Normalised FIR with coefficients [1;1;1] * [1,1,1]
 *
 * (near) Unity gain filter through repeated scaling and rounding
 *	- 6 rotate operations per output
 *	- 8 vector operations per output
 * _______
 *   14 operations
 */
STORAGE_CLASS_REF_VECTOR_FUNC_C tvector1w fir3x3m_9dB_nrm (
	const s_1w_3x3_matrix		m);

/** @brief Normalised dual output 2D FIR with coefficients  [1;2;1] * [1,2,1]
 *
 * @param[in] m	4x3 matrix with pixels
 *
 * @return		two filtered outputs (2x1 matrix)
 *
 * This function will calculate the
 * Normalised FIR with coefficients  [1;2;1] * [1,2,1]
 * and produce two outputs (vertical)
 * Unity gain filter through repeated scaling and rounding
 * compute two outputs per call to re-use common intermediates
 *	- 4 rotate operations per output
 *	- 6 vector operations per output (alternative possible, but in this
 *	    form it's not obvious to re-use variables)
 * _______
 *   10 total operations
 */
 STORAGE_CLASS_REF_VECTOR_FUNC_C s_1w_2x1_matrix fir3x3m_6dB_out2x1_nrm (
	const s_1w_4x3_matrix		m);

/** @brief Normalised dual output 2D FIR with coefficients [1;1;1] * [1,1,1]
 *
 * @param[in] m	4x3 matrix with pixels
 *
 * @return		two filtered outputs (2x1 matrix)
 *
 * This function will calculate the
 * Normalised FIR with coefficients [1;1;1] * [1,1,1]
 * and produce two outputs (vertical)
 * (near) Unity gain filter through repeated scaling and rounding
 * compute two outputs per call to re-use common intermediates
 *	- 4 rotate operations per output
 *	- 7 vector operations per output (alternative possible, but in this
 *	    form it's not obvious to re-use variables)
 * _______
 *   11 total operations
 */
STORAGE_CLASS_REF_VECTOR_FUNC_C s_1w_2x1_matrix fir3x3m_9dB_out2x1_nrm (
	const s_1w_4x3_matrix		m);

/** @brief Normalised 2D FIR 5x5
 *
 * @param[in] m	5x5 matrix with pixels
 *
 * @return		filtered output
 *
 * This function will calculate the
 * Normalised FIR with coefficients [1;1;1] * [1;2;1] * [1,2,1] * [1,1,1]
 * and produce a filtered output
 * (near) Unity gain filter through repeated scaling and rounding
 *	- 20 rotate operations per output
 *	- 28 vector operations per output
 * _______
 *   48 total operations
*/
STORAGE_CLASS_REF_VECTOR_FUNC_C tvector1w fir5x5m_15dB_nrm (
	const s_1w_5x5_matrix	m);

/** @brief Normalised FIR 1x5
 *
 * @param[in] m	1x5 matrix with pixels
 *
 * @return		filtered output
 *
 * This function will calculate the
 * Normalised FIR with coefficients [1,2,1] * [1,1,1] = [1,4,6,4,1]
 * and produce a filtered output
 * (near) Unity gain filter through repeated scaling and rounding
 *	- 4 rotate operations per output
 *	- 5 vector operations per output
 * _______
 *   9 total operations
*/
STORAGE_CLASS_REF_VECTOR_FUNC_C tvector1w fir1x5m_12dB_nrm (
	const s_1w_1x5_matrix m);

/** @brief Normalised 2D FIR 5x5
 *
 * @param[in] m	5x5 matrix with pixels
 *
 * @return		filtered output
 *
 * This function will calculate the
 * Normalised FIR with coefficients [1;2;1] * [1;2;1] * [1,2,1] * [1,2,1]
 * and produce a filtered output
 * (near) Unity gain filter through repeated scaling and rounding
 *	- 20 rotate operations per output
 *	- 30 vector operations per output
 * _______
 *   50 total operations
*/
STORAGE_CLASS_REF_VECTOR_FUNC_C tvector1w fir5x5m_12dB_nrm (
	const s_1w_5x5_matrix m);
#endif /*_REF_VECTOR_FUNC_H_INCLUDED_*/

