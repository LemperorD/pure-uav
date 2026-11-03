#ifndef _AWNN_QUANTIZE_H_
#define _AWNN_QUANTIZE_H_

#include <vip_lite.h>

vip_int32_t fp32_to_dfp(const float in,  const signed char fl, const vip_enum type);
vip_int32_t fp32_to_affine(
    const float in,
    const float scale,
    const  int zero_point,
    const vip_enum type
    );
vip_status_e integer_convert(
    const void * src,
    void *dest,
    vip_enum src_dtype,
    vip_enum dst_dtype
    );
unsigned short fp32_to_fp16(float in);
vip_status_e float32_to_dtype(
    float src,
    unsigned char *dst,
    const vip_enum data_type,
    const vip_enum quant_format,
    signed char fixed_point_pos,
    float tf_scale,
    vip_int32_t tf_zerop
    );

float int8_to_fp32(signed char val, signed char fixedPointPos);
float int16_to_fp32(vip_int16_t val, signed char fixedPointPos);
vip_float_t affine_to_fp32(vip_int32_t val, vip_int32_t zeroPoint, vip_float_t scale);
vip_float_t uint8_to_fp32(vip_uint8_t val, vip_int32_t zeroPoint, vip_float_t scale);
float fp16_to_fp32(const short in);

vip_uint32_t type_get_bytes(const vip_enum type);

#endif
