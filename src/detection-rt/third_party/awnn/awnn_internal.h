
#include <vip_lite.h>
#include <sys/time.h>
#include <pthread.h>
#include <stdio.h>
//#include <log/log.h>
#ifdef AWNN_LOG
#define ALOGD(...) do { fprintf(stderr, __VA_ARGS__); } while (0)
#define ALOGE(...) do { fprintf(stderr, __VA_ARGS__); } while (0)
#else
#define ALOGD(...) ((void)0)
#define ALOGE(...) ((void)0)
#endif

typedef struct Awnn_params {
    vip_buffer_create_params_t vip_param;
    /* awnn addition param */
    vip_char_t   name[256];
    vip_uint32_t elements;
} Awnn_params_t;

typedef struct Awnn_Context {
    pthread_mutex_t mutex;
    /* VIP lite buffer objects. */
    vip_network     network;
    vip_uint32_t    input_count;
    vip_uint32_t    output_count;
    vip_buffer     *input_buffers;
    vip_buffer     *output_buffers;
    Awnn_params_t  *input_params;
    Awnn_params_t  *output_params;
    vip_float_t   **quantize_maps;
    void          **user_input_buffers;
    float         **user_output_buffers;
} Awnn_Context_t;


#define _CHECK_STATUS( stat, lbl) do {\
	if( VIP_SUCCESS != stat) {\
		ALOGD("Error: %s, %s at %d\n", __FILE__, __FUNCTION__, __LINE__); \
		goto lbl;\
	}\
} while(0)

#define TIME_SLOTS   10
static vip_uint64_t time_begin[TIME_SLOTS];
static vip_uint64_t time_end[TIME_SLOTS];
static vip_uint64_t GetTime()
{
    struct timeval time;
    gettimeofday(&time, NULL);
    return (vip_uint64_t)(time.tv_usec + time.tv_sec * 1000000LL);
}

#define TimeBegin(id) \
do { \
    time_begin[id] = GetTime(); \
} while (0)

#define TimeEnd(id, PREFIX, ...) \
do { \
    time_end[id] = GetTime(); \
    ALOGD(PREFIX"%.2f ms.\n", ##__VA_ARGS__, (float)(time_end[id] - time_begin[id]) / 1000); \
} while (0)
