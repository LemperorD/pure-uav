#include <stdio.h>
#include <string.h>

#include "awnn_quantize.h"

#define MATH_ABS(x)      (((x) < 0)    ? -(x) :  (x))
#define MATH_MAX(a,b)    (((a) > (b)) ? (a) : (b))
#define MATH_MIN(a,b)    (((a) < (b)) ? (a) : (b))

vip_int32_t type_is_integer(const vip_enum type)
{
    vip_int32_t ret;
    ret = 0;
    switch(type)
    {
    case VIP_BUFFER_FORMAT_INT8:
    case VIP_BUFFER_FORMAT_INT16:
    case VIP_BUFFER_FORMAT_INT32:
    case VIP_BUFFER_FORMAT_UINT8:
    case VIP_BUFFER_FORMAT_UINT16:
    case VIP_BUFFER_FORMAT_UINT32:
        ret = 1;
        break;
    default:
        break;
    }

    return ret;
}

vip_int32_t type_is_signed(const vip_enum type)
{
    vip_int32_t ret;
    ret = 0;
    switch(type)
    {
    case VIP_BUFFER_FORMAT_INT8:
    case VIP_BUFFER_FORMAT_INT16:
    case VIP_BUFFER_FORMAT_INT32:
    case VIP_BUFFER_FORMAT_BFP16:
    case VIP_BUFFER_FORMAT_FP16:
    case VIP_BUFFER_FORMAT_FP32:
        ret = 1;
        break;
    default:
        break;
    }

    return ret;
}

void type_get_range(vip_enum type, double *max_range, double * min_range)
{
    vip_int32_t bits;
    double from, to;
    from = 0.0;
    to = 0.0;
    bits = type_get_bytes(type) * 8;
    if(type_is_integer(type)) {
        if(type_is_signed(type)) {
            from = (double)(-(1L << (bits - 1)));
            to = (double)((1UL << (bits - 1)) - 1);
        }
        else {
            from = 0.0;
            to = (double)((1UL << bits) - 1);
        }
    }
    else {
        //  TODO: Add float
    }
    if(NULL != max_range) {
        *max_range = to;
    }
    if(NULL != min_range) {
        *min_range = from;
    }
}

double copy_sign(double number, double sign)
{
    double value = MATH_ABS(number);
    return (sign > 0) ? value : (-value);
}

int math_floorf(double x)
{
    if (x >= 0)
    {
        return (int)x;
    }
    else
    {
        return (int)x - 1;
    }
}

double rint(double x)
{
#define _EPSILON 1e-8
    double decimal;
    double inter;
    int intpart;

    intpart = (int)x;
    decimal = x - intpart;
    inter = (double)intpart;

    if(MATH_ABS((MATH_ABS(decimal) - 0.5f)) < _EPSILON )
    {
        inter += (vip_int32_t)(inter) % 2;
    }
    else
    {
        return copy_sign(math_floorf(MATH_ABS(x) + 0.5f), x);
    }

    return inter;
}

vip_int32_t fp32_to_dfp(const float in,  const signed char fl, const vip_enum type)
{
    vip_int32_t data;
    double max_range;
    double min_range;
    type_get_range(type, &max_range, &min_range);
    if(fl > 0 )
    {
        data = (vip_int32_t)rint(in * (float)(1 << fl ));
    }
    else
    {
        data = (vip_int32_t)rint(in * (1.0f / (float)(1 << -fl )));
    }
    data = MATH_MIN(data, (vip_int32_t)max_range);
    data = MATH_MAX(data, (vip_int32_t)min_range);

    return data;
}

vip_int32_t fp32_to_affine(
    const float in,
    const float scale,
    const  int zero_point,
    const vip_enum type
    )
{
    vip_int32_t data;
    double max_range;
    double min_range;
    type_get_range(type, &max_range, &min_range);
    data = (vip_int32_t)(rint(in / scale ) + zero_point);
    data = MATH_MAX((vip_int32_t)min_range, MATH_MIN((vip_int32_t)max_range , data ));
    return data;
}

vip_status_e integer_convert(
    const void * src,
    void *dest,
    vip_enum src_dtype,
    vip_enum dst_dtype
    )
{
    vip_status_e status = VIP_SUCCESS;

        unsigned char all_zeros[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        unsigned char all_ones[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
        vip_uint32_t src_sz = type_get_bytes(src_dtype);
        vip_uint32_t dest_sz = type_get_bytes(dst_dtype);
        unsigned char* buffer = all_zeros;
        if(((vip_int8_t *)src)[src_sz - 1] & 0x80 )
        {
            buffer = all_ones;
        }
        memcpy(buffer, src, src_sz);
        memcpy(dest, buffer, dest_sz);

    return status;
}

static unsigned short  fp32_to_bfp16_rtne(float in)
{
    /*
    Convert a float point to bfloat16, with round-nearest-to-even as rounding method.
    */
    vip_uint32_t fp32 = 0;
    memcpy((vip_uint8_t*)&fp32, (vip_uint8_t*)&in, sizeof(vip_uint32_t));
    unsigned short  out;

    vip_uint32_t lsb = (fp32 >> 16) & 1;    /* Least significant bit of resulting bfloat. */
    vip_uint32_t rounding_bias = 0x7fff + lsb;

    if (0x7FC00000 == in ) {
        out = 0x7fc0;
    }
    else {
        fp32 += rounding_bias;
        out = (unsigned short ) (fp32 >> 16);
    }

    return out;
}

unsigned short fp32_to_fp16(float in)
{
    vip_uint32_t fp32 = 0;
    vip_uint32_t t1 = 0;
    vip_uint32_t t2 = 0;
    vip_uint32_t t3 = 0;
    vip_uint32_t fp16 = 0u;

    memcpy((vip_uint8_t*)&fp32, (vip_uint8_t*)&in, sizeof(vip_uint32_t));

    t1 = (fp32 & 0x80000000u) >> 16;  /* sign bit. */
    t2 = (fp32 & 0x7F800000u) >> 13;  /* Exponent bits */
    t3 = (fp32 & 0x007FE000u) >> 13;  /* Mantissa bits, no rounding */

    if(t2 >= 0x023c00u )
    {
        fp16 = t1 | 0x7BFF;     /* Don't round to infinity. */
    }
    else if(t2 <= 0x01c000u )
    {
        fp16 = t1;
    }
    else
    {
        t2 -= 0x01c000u;
        fp16 = t1 | t2 | t3;
    }

    return (unsigned short) fp16;
}

vip_status_e float32_to_dtype(
    float src,
    unsigned char *dst,
    const vip_enum data_type,
    const vip_enum quant_format,
    signed char fixed_point_pos,
    float tf_scale,
    vip_int32_t tf_zerop
    )
{
    vip_status_e status = VIP_SUCCESS;

    switch(data_type )
    {
    case VIP_BUFFER_FORMAT_FP32:
        memcpy((vip_uint8_t*)dst, (vip_uint8_t*)&src, sizeof(float));
        break;
    case VIP_BUFFER_FORMAT_FP16:
        *(vip_int16_t *)dst = fp32_to_fp16(src);
        break;
    case VIP_BUFFER_FORMAT_BFP16:
        *(vip_int16_t *)dst = fp32_to_bfp16_rtne(src);
        break;
    case VIP_BUFFER_FORMAT_INT8:
    case VIP_BUFFER_FORMAT_UINT8:
    case VIP_BUFFER_FORMAT_INT16:
        {
            vip_int32_t dst_value = 0;
            switch(quant_format)
            {
            case VIP_BUFFER_QUANTIZE_DYNAMIC_FIXED_POINT:
                dst_value = fp32_to_dfp(src, fixed_point_pos, data_type);
                break;
            case VIP_BUFFER_QUANTIZE_TF_ASYMM:
                dst_value = fp32_to_affine(src, tf_scale, tf_zerop, data_type);
                break;
            case VIP_BUFFER_QUANTIZE_NONE:
                dst_value = (vip_int32_t)src;
                break;
            default:
                break;
            }
            integer_convert(&dst_value, dst, VIP_BUFFER_FORMAT_INT32, data_type);
        }
        break;
    default:
        printf("unsupported tensor type\n");;
    }

    return status;
}

float int8_to_fp32(signed char val, signed char fixedPointPos)
{
    float result = 0.0f;

    if (fixedPointPos > 0) {
        result = (float)val * (1.0f / ((float) (1 << fixedPointPos)));
    }
    else {
        result = (float)val * ((float) (1 << -fixedPointPos));
    }

    return result;
}

float int16_to_fp32(vip_int16_t val, signed char fixedPointPos)
{
    float result = 0.0f;

    if (fixedPointPos > 0) {
        result = (float)val * (1.0f / ((float) (1 << fixedPointPos)));
    }
    else {
        result = (float)val * ((float) (1 << -fixedPointPos));
    }

    return result;
}
vip_float_t affine_to_fp32(vip_int32_t val, vip_int32_t zeroPoint, vip_float_t scale)
{
    vip_float_t result;
    result = ((vip_float_t)val - zeroPoint) * scale;
    return result;
}

vip_float_t uint8_to_fp32(vip_uint8_t val, vip_int32_t zeroPoint, vip_float_t scale)
{
    vip_float_t result;
    result = (val - (vip_uint8_t)zeroPoint) * scale;
    return result;
}

typedef union
{
    unsigned int u;
    float f;
} _fp32_t;

float fp16_to_fp32(const short in)
{
    const _fp32_t magic = { (254 - 15) << 23 };
    const _fp32_t infnan = { (127 + 16) << 23 };
    _fp32_t o;
    // Non-sign bits
    o.u = (in & 0x7fff ) << 13;
    o.f *= magic.f;
    if(o.f  >= infnan.f)
    {
        o.u |= 255 << 23;
    }
    //Sign bit
    o.u |= (in & 0x8000 ) << 16;
    return o.f;
}

vip_uint32_t type_get_bytes(const vip_enum type)
{
    switch(type)
    {
        case VIP_BUFFER_FORMAT_INT8:
        case VIP_BUFFER_FORMAT_UINT8:
            return 1;
        case VIP_BUFFER_FORMAT_INT16:
        case VIP_BUFFER_FORMAT_UINT16:
        case VIP_BUFFER_FORMAT_FP16:
        case VIP_BUFFER_FORMAT_BFP16:
            return 2;
        case VIP_BUFFER_FORMAT_FP32:
        case VIP_BUFFER_FORMAT_INT32:
        case VIP_BUFFER_FORMAT_UINT32:
            return 4;
        case VIP_BUFFER_FORMAT_FP64:
        case VIP_BUFFER_FORMAT_INT64:
        case VIP_BUFFER_FORMAT_UINT64:
            return 8;

        default:
            return 0;
    }
}