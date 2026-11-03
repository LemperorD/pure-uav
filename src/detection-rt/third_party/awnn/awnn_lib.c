#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "awnn_internal.h"
#include "awnn_quantize.h"
#include "awnn_lib.h"

#define LOG_NDEBUG 1
//#include <log/log.h>
#define ALOGD(...) ((void)0)
#define ALOGE(...) ((void)0)
#define ALOGW(...) ((void)0)

/**
 * @brief version code
 * awnn_lib库。
 */
#define AWNN_LIB_VERSION    "AWNN_LIB_1.0.2"

static vip_status_e query_hardware_info(void) {
    vip_uint32_t version = vip_get_version();
    vip_uint32_t device_count = 0;
    vip_uint32_t cid = 0;
    vip_uint32_t *core_count = VIP_NULL;
    vip_uint32_t i = 0;

    ALOGD("VIPLite driver version=0x%08x...\n", version);
    if (version >= 0x00010601) {
        vip_query_hardware(VIP_QUERY_HW_PROP_CID, sizeof(vip_uint32_t), &cid);
        vip_query_hardware(VIP_QUERY_HW_PROP_DEVICE_COUNT, sizeof(vip_uint32_t), &device_count);
        core_count = (vip_uint32_t*)malloc(sizeof(vip_uint32_t) * device_count);
        vip_query_hardware(VIP_QUERY_HW_PROP_CORE_COUNT_EACH_DEVICE,
                          sizeof(vip_uint32_t) * device_count, core_count);
        ALOGD("VIP cid=0x%x, device_count=%d\n", cid, device_count);
        for (i = 0; i < device_count; i++) {
            ALOGD("* device[%d] core_count=%d\n", i, core_count[i]);
        }
        free(core_count);
    }
    return VIP_SUCCESS;
}

static void __destroy_network(Awnn_Context_t *info) {
    int i = 0;

    if (info == VIP_NULL) {
        ALOGW("info is NULL\n");
        return;
    }
    for (i = 0; i < info->output_count; i++) {
        if (info->user_output_buffers[i]) {
            free(info->user_output_buffers[i]);
        }
    }
    free(info->user_output_buffers);

    vip_destroy_network(info->network);

    for (i = 0; i < info->input_count; i++) {
        vip_destroy_buffer(info->input_buffers[i]);
    }
    free(info->input_buffers);

    for (i = 0; i < info->output_count; i++) {
        vip_destroy_buffer(info->output_buffers[i]);
    }
    free(info->output_buffers);
    info->output_buffers = VIP_NULL;

    for (i = 0; i < info->output_count; i++) {
        if (info->quantize_maps[i]) {
            free(info->quantize_maps[i]);
            info->quantize_maps[i] = VIP_NULL;
        }
    }
    free(info->quantize_maps);
    info->quantize_maps = VIP_NULL;
}

static void dump_param(Awnn_params_t *param, const char *ioname, int index) {
    char buffer[4096];
    int i;
    size_t ss = 0;
    ss += sprintf(buffer + ss, "%s %d dim", ioname, index);
    for (i = 0; i < param->vip_param.num_of_dims; i++) {
        ss += sprintf(buffer + ss, " %d", param->vip_param.sizes[i]);
    }
    ss += sprintf(buffer + ss, ", data_format=%d, name=%s, elements=%u", param->vip_param.data_format, param->name, param->elements);
    switch(param->vip_param.quant_format) {
        case VIP_BUFFER_QUANTIZE_DYNAMIC_FIXED_POINT:
            ss += sprintf(buffer + ss, ", dfp=%d\n", param->vip_param.quant_data.dfp.fixed_point_pos);
            break;
        case VIP_BUFFER_QUANTIZE_TF_ASYMM:
            ss += sprintf(buffer + ss, ", scale=%f, zero_point=%d\n", param->vip_param.quant_data.affine.scale, param->vip_param.quant_data.affine.zeroPoint);
            break;
        default:
            ss += sprintf(buffer + ss, ", none-quant\n");
    }
    ALOGD("%s", buffer);
}

/* Create the network in the info. */
vip_status_e load_param(Awnn_Context_t *info) {
    vip_status_e status = VIP_SUCCESS;
    int i = 0;
    int j = 0;

    vip_buffer_create_params_t *param;
    vip_uint32_t buff_size;
    /* Create input buffers. */
    vip_query_network(info->network, VIP_NETWORK_PROP_INPUT_COUNT, &info->input_count);
    info->input_buffers = (vip_buffer *)malloc(sizeof(vip_buffer) * info->input_count);
    info->input_params = (Awnn_params_t *)malloc(sizeof(Awnn_params_t) * info->input_count);
    for (i = 0; i < info->input_count; i++) {
        param = &info->input_params[i].vip_param;
        param->memory_type = VIP_BUFFER_MEMORY_TYPE_DEFAULT;
        vip_query_input(info->network, i, VIP_BUFFER_PROP_DATA_FORMAT, &param->data_format);
        vip_query_input(info->network, i, VIP_BUFFER_PROP_NUM_OF_DIMENSION, &param->num_of_dims);
        vip_query_input(info->network, i, VIP_BUFFER_PROP_SIZES_OF_DIMENSION, param->sizes);
        vip_query_input(info->network, i, VIP_BUFFER_PROP_QUANT_FORMAT, &param->quant_format);
        vip_query_input(info->network, i, VIP_BUFFER_PROP_NAME, info->input_params[i].name);
        switch(param->quant_format) {
            case VIP_BUFFER_QUANTIZE_DYNAMIC_FIXED_POINT:
                vip_query_input(info->network, i, VIP_BUFFER_PROP_FIXED_POINT_POS,
                                &param->quant_data.dfp.fixed_point_pos);
                break;
            case VIP_BUFFER_QUANTIZE_TF_ASYMM:
                vip_query_input(info->network, i, VIP_BUFFER_PROP_TF_SCALE,
                                &param->quant_data.affine.scale);
                vip_query_input(info->network, i, VIP_BUFFER_PROP_TF_ZERO_POINT,
                                &param->quant_data.affine.zeroPoint);
                break;
            default:
                break;
        }
        dump_param(&info->input_params[i], "input", i);
        info->input_params[i].elements = 1;
        for (j = 0; j < param->num_of_dims; j++) {
            info->input_params[i].elements *= param->sizes[j];
        }
        status = vip_create_buffer(param, 0, &info->input_buffers[i]);
        if (status != VIP_SUCCESS) {
            ALOGE("fail to create input %d buffer, status=%d\n", i, status);
            return status;
        } else {
            buff_size = vip_get_buffer_size(info->input_buffers[i]);
            ALOGD("create input buffer %d: %u\n", i, buff_size);
        }
    }

    /* Create output buffer. */
    vip_query_network(info->network, VIP_NETWORK_PROP_OUTPUT_COUNT, &info->output_count);
    info->output_buffers = (vip_buffer *)malloc(sizeof(vip_buffer) * info->output_count);
    info->output_params = (Awnn_params_t *)malloc(sizeof(Awnn_params_t) * info->output_count);
    info->quantize_maps = (float **)calloc(info->output_count, sizeof(float *));
    info->user_output_buffers = (float **)malloc(sizeof(float *) * info->output_count);
    for (i = 0; i < info->output_count; i++) {
        param = &info->output_params[i].vip_param;
        param->memory_type = VIP_BUFFER_MEMORY_TYPE_DEFAULT;
        vip_query_output(info->network, i, VIP_BUFFER_PROP_DATA_FORMAT, &param->data_format);
        vip_query_output(info->network, i, VIP_BUFFER_PROP_NUM_OF_DIMENSION, &param->num_of_dims);
        vip_query_output(info->network, i, VIP_BUFFER_PROP_SIZES_OF_DIMENSION, param->sizes);
        vip_query_output(info->network, i, VIP_BUFFER_PROP_QUANT_FORMAT, &param->quant_format);
        vip_query_output(info->network, i, VIP_BUFFER_PROP_NAME, info->output_params[i].name);
        switch(param->quant_format) {
            case VIP_BUFFER_QUANTIZE_DYNAMIC_FIXED_POINT:
                vip_query_output(info->network, i, VIP_BUFFER_PROP_FIXED_POINT_POS,
                                &param->quant_data.dfp.fixed_point_pos);
                break;
            case VIP_BUFFER_QUANTIZE_TF_ASYMM:
                vip_query_output(info->network, i, VIP_BUFFER_PROP_TF_SCALE,
                                &param->quant_data.affine.scale);
                vip_query_output(info->network, i, VIP_BUFFER_PROP_TF_ZERO_POINT,
                                &param->quant_data.affine.zeroPoint);
                break;
            default:
                break;
        }
        info->output_params[i].elements = 1;
        for (j = 0; j < param->num_of_dims; j++) {
            info->output_params[i].elements *= param->sizes[j];
        }
        dump_param(&info->output_params[i], "output", i);

        status = vip_create_buffer(param, 0, &info->output_buffers[i]);
        if (status != VIP_SUCCESS) {
            ALOGE("fail to create output %d buffer, status=%d\n", i, status);
            return status;
        } else {
            buff_size = vip_get_buffer_size(info->output_buffers[i]);
            ALOGD("create output buffer %d: %u\n", i, buff_size);
        }
        info->user_output_buffers[i] = (float *)malloc(info->output_params[i].elements * sizeof(float));

        if (param->data_format == VIP_BUFFER_FORMAT_UINT8) {
            info->quantize_maps[i] = (float *)malloc(sizeof(float) * 256);
            for (j = 0; j < 256; j++) {
                info->quantize_maps[i][j] = uint8_to_fp32(j, param->quant_data.affine.zeroPoint, param->quant_data.affine.scale);
            }
        } else if (param->data_format == VIP_BUFFER_FORMAT_INT8) {
            info->quantize_maps[i] = (float *)malloc(sizeof(float) * 256);
            if (param->quant_format == VIP_BUFFER_QUANTIZE_DYNAMIC_FIXED_POINT) {
                for (j = 0; j < 256; j++) {
                    info->quantize_maps[i][j] = int8_to_fp32(j, param->quant_data.dfp.fixed_point_pos);
                }
            } else if (param->quant_format == VIP_BUFFER_QUANTIZE_TF_ASYMM) {
                for (j = 0; j < 256; j++) {
                    vip_int32_t src_value = 0;
                    integer_convert(&j, &src_value, VIP_BUFFER_FORMAT_INT8, VIP_BUFFER_FORMAT_INT32);
                    info->quantize_maps[i][j] = affine_to_fp32(src_value, param->quant_data.affine.zeroPoint, param->quant_data.affine.scale);
                }
            }
        }
    }

    vip_uint32_t mem_pool_size = 0;
    vip_query_network(info->network, VIP_NETWORK_PROP_MEMORY_POOL_SIZE, &mem_pool_size);
    ALOGD("memory pool size=%d bytes\n", mem_pool_size);

    return status;
}

static vip_status_e set_network_io(Awnn_Context_t *info) {
    vip_status_e status = VIP_SUCCESS;
    int i = 0;

    /* Load input buffer data. */
    for (i = 0; i < info->input_count; i++) {
        /* Set input. */
        status = vip_set_input(info->network, i, info->input_buffers[i]);
        if (status != VIP_SUCCESS) {
            ALOGE("fail to set input %d\n", i);
            goto exit;
        }
    }

    for (i = 0; i < info->output_count; i++) {
        if (info->output_buffers[i] != VIP_NULL) {
            status = vip_set_output(info->network, i, info->output_buffers[i]);
            if (status != VIP_SUCCESS) {
                ALOGE("fail to set output\n");
                goto exit;
            }
        } else {
            ALOGE("fail output %d is null. output_counts=%d\n", i, info->output_count);
            status = VIP_ERROR_FAILURE;
            goto exit;
        }
    }

exit:
    return status;
}

void awnn_init() {
TimeBegin(1);
    vip_status_e status;

    status = vip_init();
    if (status != VIP_SUCCESS) {
        ALOGE("viplite init failed!\n");
        goto exit;
    } else {
        ALOGD("viplite init OK.\n");
    }
    query_hardware_info();

exit:
TimeEnd(1, "%s total: ", __func__);
}

void awnn_uninit() {
TimeBegin(1);
    vip_status_e status;
    status = vip_destroy();
    if (status != VIP_SUCCESS) {
        ALOGE("viplite uninit failed!\n");
    }

TimeEnd(1, "%s total: ", __func__);
}

Awnn_Context_t* awnn_create(const char* nbg) {
TimeBegin(1);
    vip_status_e status;
    Awnn_Context_t *info = (Awnn_Context_t *)calloc(1, sizeof(Awnn_Context_t));
    if (info == NULL) {
        ALOGE("malloc info failed\n");
        goto error;
    }
    pthread_mutex_init(&info->mutex, NULL);

TimeBegin(2);
    status = vip_create_network(nbg, 0, VIP_CREATE_NETWORK_FROM_FILE, &info->network);
TimeEnd(2, "  vip_create_network %s: ", nbg);
    _CHECK_STATUS(status, error);
TimeBegin(2);
    status = load_param(info);
TimeEnd(2, "  load_param %s: ", nbg);
    _CHECK_STATUS(status, error);
TimeBegin(2);
    status = vip_prepare_network(info->network);
TimeEnd(2, "  prepare network %s: ", nbg);
    _CHECK_STATUS(status, error);
TimeBegin(2);
    status = set_network_io(info);
TimeEnd(2, "  set network io %s: ", nbg);
    _CHECK_STATUS(status, error);

    goto exit;

error:
    if (info) {
        vip_finish_network(info->network);
        __destroy_network(info);
        pthread_mutex_destroy(&info->mutex);
        free(info);
        info = NULL;
    }

exit:
TimeEnd(1, "%s total: ", __func__);
    return info;
}

void awnn_set_input_buffers(Awnn_Context_t *info, void **input_buffers) {
    info->user_input_buffers = input_buffers;

    int i = 0;
    pthread_mutex_lock(&info->mutex);
TimeBegin(1);
TimeBegin(2);
    for (i = 0; i < info->input_count; i++) {
        void *data = vip_map_buffer(info->input_buffers[i]);
        vip_uint32_t buff_size = vip_get_buffer_size(info->input_buffers[i]);
        ALOGD("memcpy(%p, %p, %u)", data, info->user_input_buffers[i], buff_size);
        memcpy(data, info->user_input_buffers[i], buff_size);
        vip_unmap_buffer(info->input_buffers[i]);
    }
TimeEnd(2, "  load_input_data: ");

TimeBegin(2);
     /* it is only necessary to call vip_flush_buffer() after set vpmdENABLE_FLUSH_CPU_CACHE to 2 */
    for (i = 0; i < info->input_count; i++) {
        if ((vip_flush_buffer(info->input_buffers[i], VIP_BUFFER_OPER_TYPE_FLUSH)) != VIP_SUCCESS) {
                ALOGE("flush input%d cache failed.\n", i);
        }
    }
TimeEnd(2, "  vip_flush_buffer input: ");
TimeEnd(1, "%s total: ", __func__);
    pthread_mutex_unlock(&info->mutex);
}

float **awnn_get_output_buffers(Awnn_Context_t *info) {
    return info->user_output_buffers;
}

void *awnn_get_output_buffer(Awnn_Context_t *info, int i) {
    return vip_map_buffer(info->output_buffers[i]);
}

void awnn_run(Awnn_Context_t* info) {
    vip_status_e status;
    int i = 0;
    int j = 0;
    pthread_mutex_lock(&info->mutex);

TimeBegin(1);

TimeBegin(2);
    status = vip_run_network(info->network);
    if (status != VIP_SUCCESS) {
        ALOGE("fail to run network, status=%d\n", status);
        goto exit;
    }
TimeEnd(2, "  vip_run_network: ");

TimeBegin(2);
    for (i = 0; i < info->output_count; i++) {
        if ((vip_flush_buffer(info->output_buffers[i], VIP_BUFFER_OPER_TYPE_INVALIDATE)) != VIP_SUCCESS){
            ALOGE("flush output%d cache failed.\n", i);
        }
    }
TimeEnd(2, "  vip_flush_buffer output: ");

TimeBegin(2);
    vip_uint32_t elements;
    for (i = 0; i < info->output_count; i++) {
        int buff_size = vip_get_buffer_size(info->output_buffers[i]);
        void *out_data = vip_map_buffer(info->output_buffers[i]);
        vip_buffer_create_params_t *param = &info->output_params[i].vip_param;
        elements = info->output_params[i].elements;
        float *fp_data = info->user_output_buffers[i];

        if (param->data_format == VIP_BUFFER_FORMAT_FP32) {
TimeBegin(3);
            memcpy(fp_data, out_data, buff_size);
TimeEnd(3, "    fp32 %d memcpy: ", buff_size);
        } else if (param->data_format == VIP_BUFFER_FORMAT_FP16) {
TimeBegin(3);
            short *data = (short*)malloc(buff_size);
            memcpy(data, out_data, buff_size);
TimeEnd(3, "    fp16 memcpy: ");
            for (j = 0; j < elements; j++) {
                fp_data[j] = fp16_to_fp32(*(data + j));
            }
            free(data);
        } else if (param->data_format == VIP_BUFFER_FORMAT_UINT8
                || param->data_format == VIP_BUFFER_FORMAT_INT8) {
TimeBegin(3);
            unsigned char *data = (unsigned char*)malloc(buff_size);
            memcpy(data, out_data, buff_size);
TimeEnd(3, "    int8/uint8 %d memcpy: ", buff_size);
            for (j = 0; j < elements; j++) {
                fp_data[j] = info->quantize_maps[i][*(data + j)];
            }
            free(data);
        } else if (param->data_format == VIP_BUFFER_FORMAT_INT16) {
TimeBegin(3);
            short *data = (short*)malloc(buff_size);
            memcpy(data, out_data, buff_size);
TimeEnd(3, "    int16 memcpy: ");
            for (j = 0; j < elements; j++) {
                fp_data[j] = int16_to_fp32(*(data + j), param->quant_data.dfp.fixed_point_pos);
            }
            free(data);
        }
        vip_unmap_buffer(info->output_buffers[i]);
    }
TimeEnd(2, "  tensor to fp: ");

exit:
TimeEnd(1, "%s total: ", __func__);

    pthread_mutex_unlock(&info->mutex);
}

void awnn_destroy(Awnn_Context_t *info) {
TimeBegin(1);
    if (info) {
        vip_finish_network(info->network);
        __destroy_network(info);
        pthread_mutex_destroy(&info->mutex);
        free(info);
    }

TimeEnd(1, "%s total: ", __func__);
}

void awnn_dump_io(Awnn_Context_t *info, const char *path) {
TimeBegin(1);
    if (info) {
        int i, j;
        char name[256];
        for (i = 0; i < info->input_count; i++) {
            sprintf(name, "%s.input_%d.txt", path, i);
            FILE *fp = fopen(name, "wb");
            if (fp != NULL) {
                for (j = 0; j < info->input_params[i].elements; j++) {
                    fprintf(fp, "%u\n", ((unsigned char **)info->user_input_buffers)[i][j]);
                }
                fclose(fp);
                fp = NULL;
            }
        }
        for (i = 0; i < info->output_count; i++) {
            sprintf(name, "%s.output_%d.txt", path, i);
            FILE *fp = fopen(name, "wb");
            if (fp != NULL) {
                for (j = 0; j < info->output_params[i].elements; j++) {
                    fprintf(fp, "%f\n", info->user_output_buffers[i][j]);
                }
                fclose(fp);
                fp = NULL;
            }
        }
    }
TimeEnd(1, "%s total: ", __func__);
}

