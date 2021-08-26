/*!
 * \file ch_math_utils.h
 * \date July 20, 2017
 * \author nparikh
 * \copyright Copyright 2017 Chirp Microsystems.  All rights reserved.
 * \brief Functions for performing fixed point arithmetic.
 *        https://github.com/dmoulding/log2fix
 *        https://github.com/chmike/fpsqrt
 */

#ifndef CH_MATH_UTILS_H_
#define CH_MATH_UTILS_H_

#include <stdio.h>
#include <math.h>
#include <stdint.h>

#define FRACT_BITS 16

#define INT2FIXED(x) ((x) << FRACT_BITS)
#define FLOAT2FIXED(x) ((fixed_t)((x) * (1 << FRACT_BITS))) 

#define FIXED2INT(x) ((x) >> FRACT_BITS)
#define FIXED2FLOAT(x) (((float)(x)) / (1 << FRACT_BITS))

#define FIXEDDIV(x,y) ((fixed_t)(((uint64_t)(x) << FRACT_BITS) / (y)))
#define FIXEDMUL(x,y) ((fixed_t)(((x) >> (FRACT_BITS/2)) * ((y) >> (FRACT_BITS/2))))

#define FIXED_PI 0x3243FU

#define INV_LOG2_E_Q1DOT31  0x58b90bfcU // Inverse log base 2 of e

typedef uint32_t fixed_t;

fixed_t FP_sqrt(fixed_t x);

fixed_t FP_log2(fixed_t x);

fixed_t FP_log(fixed_t x);

int32_t sqrt_int32( int32_t v );

#endif /* CH_MATH_UTILS_H_ */
