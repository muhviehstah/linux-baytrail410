/*
 * Support for Intel Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 - 2014 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include "ia_css_mipi.h"
#include <type_support.h>
#include "ia_css_err.h"
#include "ia_css_stream_format.h"
#include "ia_css_stream_public.h"
#include "ia_css_frame_public.h"
#include "ia_css_input_port.h"
#include "ia_css_debug.h"
#include "sh_css_struct.h"
#include "sh_css_defs.h"

enum ia_css_err
ia_css_mipi_frame_specify(const unsigned int size_mem_words,
				const bool contiguous)
{
	enum ia_css_err err = IA_CSS_SUCCESS;

	my_css.size_mem_words = size_mem_words;
	my_css.contiguous = contiguous;

	return err;
}

/* Assumptions:
 *	- A line is multiple of 4 bytes = 1 word.
 *	- Each frame has SOF and EOF (each 1 word).
 *	- Each line has format header and optionally SOL and EOL (each 1 word).
 *	- Odd and even lines of YUV420 format are different in bites per pixel size.
 *	- Custom size of embedded data.
 *  -- Interleaved frames are not taken into account.
 *  -- Lines are multiples of 8B, and not necessary of (custom 3B, or 7B
 *  etc.).
 * Result is given in DDR mem words, 32B or 256 bits
 */
enum ia_css_err
ia_css_mipi_frame_calculate_size(const unsigned int width,
				const unsigned int height,
				const enum ia_css_stream_format format,
				const bool hasSOLandEOL,
				const unsigned int embedded_data_size_words,
				unsigned int *size_mem_words)
{
	enum ia_css_err err = IA_CSS_SUCCESS;

	unsigned int bits_per_pixel = 0;
	unsigned int even_line_bytes = 0;
	unsigned int odd_line_bytes = 0;
	unsigned int words_per_odd_line = 0;
	unsigned int words_for_first_line = 0;
	unsigned int words_per_even_line = 0;
	unsigned int mem_words_per_even_line = 0;
	unsigned int mem_words_per_odd_line = 0;
	unsigned int mem_words_for_first_line = 0;
	unsigned int mem_words_for_EOF = 0;
	unsigned int mem_words = 0;
	unsigned int width_padded = width;

#if defined(USE_INPUT_SYSTEM_VERSION_2401)
	/* The changes will be reverted as soon as RAW
	 * Buffers are deployed by the 2401 Input System
	 * in the non-continuous use scenario.
	 */
	width_padded += (2 * ISP_VEC_NELEMS);
#endif

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_mipi_frame_calculate_size() "
		"enter: padded_width=%d, height=%d, format=%d, hasSOLandEOL=%d, embedded_data_size_words=%d\n",
		width_padded, height, format, hasSOLandEOL, embedded_data_size_words);

	switch (format) {
		case IA_CSS_STREAM_FORMAT_RAW_6:			/* 4p, 3B, 24bits */
			bits_per_pixel = 6;	break;
		case IA_CSS_STREAM_FORMAT_RAW_7:			/* 8p, 7B, 56bits */
			bits_per_pixel = 7;		break;
		case IA_CSS_STREAM_FORMAT_RAW_8:			/* 1p, 1B, 8bits */
		case IA_CSS_STREAM_FORMAT_BINARY_8:      /*  8bits, TODO: check. */
		case IA_CSS_STREAM_FORMAT_YUV420_8:		/* odd 2p, 2B, 16bits, even 2p, 4B, 32bits */
			bits_per_pixel = 8;		break;
		case IA_CSS_STREAM_FORMAT_YUV420_10:		/* odd 4p, 5B, 40bits, even 4p, 10B, 80bits */
		case IA_CSS_STREAM_FORMAT_RAW_10:		/* 4p, 5B, 40bits */
#if !defined(HAS_NO_PACKED_RAW_PIXELS)
			/* The changes will be reverted as soon as RAW
			 * Buffers are deployed by the 2401 Input System
			 * in the non-continuous use scenario.
			 */
			bits_per_pixel = 10;
#else
			bits_per_pixel = 16;
#endif
			break;
		case IA_CSS_STREAM_FORMAT_YUV420_8_LEGACY:	/* 2p, 3B, 24bits */
		case IA_CSS_STREAM_FORMAT_RAW_12:			/* 2p, 3B, 24bits */
			bits_per_pixel = 12;	break;
		case IA_CSS_STREAM_FORMAT_RAW_14:		/* 4p, 7B, 56bits */
			bits_per_pixel = 14;	break;
		case IA_CSS_STREAM_FORMAT_RGB_444:		/* 1p, 2B, 16bits */
		case IA_CSS_STREAM_FORMAT_RGB_555:		/* 1p, 2B, 16bits */
		case IA_CSS_STREAM_FORMAT_RGB_565:		/* 1p, 2B, 16bits */
		case IA_CSS_STREAM_FORMAT_YUV422_8:		/* 2p, 4B, 32bits */
			bits_per_pixel = 16;	break;
		case IA_CSS_STREAM_FORMAT_RGB_666:		/* 4p, 9B, 72bits */
			bits_per_pixel = 18;	break;
		case IA_CSS_STREAM_FORMAT_YUV422_10:		/* 2p, 5B, 40bits */
			bits_per_pixel = 20;	break;
		case IA_CSS_STREAM_FORMAT_RGB_888:		/* 1p, 3B, 24bits */
			bits_per_pixel = 24;	break;

		case IA_CSS_STREAM_FORMAT_RAW_16:        /* TODO: not specified in MIPI SPEC, check */
		default:
			return IA_CSS_ERR_INVALID_ARGUMENTS;
	}

	odd_line_bytes = (width_padded * bits_per_pixel + 7) >> 3; /* ceil ( bits per line / 8 ) */

	/* Even lines for YUV420 formats are double in bits_per_pixel. */
	if (format == IA_CSS_STREAM_FORMAT_YUV420_8
		|| format == IA_CSS_STREAM_FORMAT_YUV420_10) {
		even_line_bytes = (width_padded * 2 * bits_per_pixel + 7) >> 3; /* ceil ( bits per line / 8 ) */
	} else {
		even_line_bytes = odd_line_bytes;
	}

   /*  a frame represented in memory:  ()- optional; data - payload words.
	*  addr		0		1		2		3		4		5		6		7:
	*  first	SOF		(SOL)	PACK_H	data	data	data	data	data
	*	data	data	data	data	data	data	data	data
	*           ...
	*			data	data	0		0		0		0		0		0
	*  second   (EOL)	(SOL)	PACK_H	data	data	data	data	data
	*	data	data	data	data	data	data	data	data
	*           ...
	*			data	data	0		0		0		0		0		0
	*  ...
	*  last		(EOL)	EOF		0		0		0		0		0		0
	*
	*  Embedded lines are regular lines stored before the first and after
	*  payload lines.
	*/


	words_per_odd_line	 = ((odd_line_bytes   + 3) >> 2 );		/* ceil(odd_line_bytes/4); word = 4 bytes */
	words_per_even_line  = ((even_line_bytes  + 3) >> 2 );
    words_for_first_line = words_per_odd_line + 2 + (hasSOLandEOL ? 1 : 0); /* + SOF +packet header + optionally (SOL), but (EOL) is not in the first line */
	words_per_odd_line	+= (1 + (hasSOLandEOL ? 2 : 0));  /* each non-first line has format header, and optionally (SOL) and (EOL). */
	words_per_even_line += (1 + (hasSOLandEOL ? 2 : 0));

	mem_words_per_odd_line	 = ((words_per_odd_line + 7) >> 3);	/* ceil(words_per_odd_line/8); mem_word = 32 bytes, 8 words */
	mem_words_for_first_line = ((words_for_first_line + 7) >> 3);
	mem_words_per_even_line  = ((words_per_even_line + 7) >> 3);
	mem_words_for_EOF        = 1; /* last line consisit of the optional (EOL) and EOF */

	mem_words = ((embedded_data_size_words + 7) >> 3) +
		mem_words_for_first_line +
				(((height + 1) >> 1) - 1) * mem_words_per_odd_line + /* ceil (height/2) - 1 (first line is calculated separatelly) */
				(  height      >> 1     ) * mem_words_per_even_line + /* floor(height/2) */
				mem_words_for_EOF;

	*size_mem_words = mem_words; /* ceil(words/8); mem word is 32B = 8words. */ //Check if this is still needed.

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_mipi_frame_calculate_size() leave: return_err=%d\n",err);
	return err;
}

#if !defined(HAS_NO_INPUT_SYSTEM) && defined(USE_INPUT_SYSTEM_VERSION_2)
enum ia_css_err
ia_css_mipi_frame_enable_check_on_size(const enum ia_css_csi2_port port,
				const unsigned int	size_mem_words)
{
	uint32_t idx;

	enum ia_css_err err = IA_CSS_ERR_RESOURCE_NOT_AVAILABLE;

	OP___assert(port < N_CSI_PORTS);
	OP___assert(size_mem_words != 0);


	for (idx = 0; idx < IA_CSS_MIPI_SIZE_CHECK_MAX_NOF_ENTRIES_PER_PORT &&
		my_css.mipi_sizes_for_check[port][idx] != 0;
		idx++){}
	if (idx < IA_CSS_MIPI_SIZE_CHECK_MAX_NOF_ENTRIES_PER_PORT) {
		my_css.mipi_sizes_for_check[port][idx] = size_mem_words;
		err = IA_CSS_SUCCESS;
	}

	return err;
}
#endif
