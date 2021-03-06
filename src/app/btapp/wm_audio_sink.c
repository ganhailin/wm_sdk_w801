/*****************************************************************************
**
**  Name:           wm_audio_sink.c
**
**  Description:    This file contains the sample functions for bluetooth audio sink application
**
*****************************************************************************/

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include "wm_bt_config.h"

#if (WM_BTA_AV_SINK_INCLUDED == CFG_ON)

#include "wm_bt_av.h"
#include "wm_audio_sink.h"
#include "wm_bt_util.h"
#include "wm_osal.h"
#include "wm_i2s.h"
#include "wm_gpio_afsel.h"

#if (WM_AUDIO_BOARD_INCLUDED == CFG_ON)
#include "audio.h"
#endif

static uint16_t g_sample_rate = 44100;
static uint8_t  g_bit_width   = 16;
static uint8_t  g_chan_count = 2;
static uint8_t  g_playing_state = 0;

#define i2sblocksize (1024*2)
#define loopblocksize (8192)
static uint32_t i2sdata[i2sblocksize];
static int16_t loopbuffer[loopblocksize];
static wm_dma_handler_type hdma_tx;
volatile int currentaudiobuf=0;
volatile int currentpt_r=0;
volatile int currentpt_w=0;
tls_os_queue_t * audiodataqueue=NULL;
tls_os_queue_t * audiosizequeue=NULL;
volatile int interceptpt=0;
const int inter_c=125001*2;
static void i2sDmaSendCpltCallback(wm_dma_handler_type *hdma)
{
    int i=0;
    for(;i<i2sblocksize/2;i++) {
        *(i2sdata + i2sblocksize / 2+i) = loopbuffer[currentpt_r]*65536;
        if(currentpt_r!=currentpt_w) {
            currentpt_r++;
            interceptpt++;
            if(interceptpt==inter_c)
            {
                interceptpt=0;
                currentpt_r+=2;
            }
            if(currentpt_r>=loopblocksize)
                currentpt_r=0;
        }else {
            for (; i < i2sblocksize / 2; i++) {
                *(i2sdata + i) = loopbuffer[currentpt_r] * 65536;
            }
        }
    }
//    printf("%d\n",currentaudiobuf);
//    memcpy(i2sdata+i2sblocksize/2,
//           i2sdatabuf+currentaudiobuf*i2sblocksize/2,
//           i2sblocksize/2*4);
}
static void i2sDmaSendHalfCpltCallback(wm_dma_handler_type *hdma)
{
    int i=0;
    for(;i<i2sblocksize/2;i++) {
        *(i2sdata + i) = loopbuffer[currentpt_r]*65536;
        if(currentpt_r!=currentpt_w) {
            currentpt_r++;
            interceptpt++;
            if(interceptpt==inter_c)
            {
                interceptpt=0;
                currentpt_r+=2;
            }
            if(currentpt_r>=loopblocksize)
                currentpt_r=0;
        }else {
            for (; i < i2sblocksize / 2; i++) {
                *(i2sdata + i) = loopbuffer[currentpt_r] * 65536;
            }
        }
    }
//    printf("%d\n",currentaudiobuf);
//    memcpy(i2sdata,
//           i2sdatabuf+currentaudiobuf*i2sblocksize/2,
//           i2sblocksize/2*4);
}

//_Noreturn static void audio_task(void* parm){
//    while(1){
//        int datasize;
//        uint16_t* datap;
//        tls_os_queue_receive(audiosizequeue,&datasize,0,0);
//        tls_os_queue_receive(audiodataqueue,&datap,0,0);
//        printf("package:%d\n",datasize);
//        int offset=(1-currentaudiobuf)*i2sblocksize/2;
//        uint32_t *p=i2sdatabuf+offset;
//        currentaudiobuf=!currentaudiobuf;
//        free(datap);
//    }
//}
static void audio_init()
{
    memset(&hdma_tx, 0, sizeof(wm_dma_handler_type));
    memset(i2sdata, 0, sizeof(i2sdata));
    wm_i2s_do_config(WM_IO_PB_11);
    wm_i2s_ws_config(WM_IO_PB_09);
    wm_i2s_ck_config(WM_IO_PB_08);
    I2S_InitDef opts = { I2S_MODE_MASTER,
                         I2S_CTRL_STEREO,
                         I2S_RIGHT_CHANNEL,
                         I2S_Standard,
                         I2S_DataFormat_32,
                         44100,
                         44100*256 };

    wm_i2s_port_init(&opts);
    wm_i2s_register_callback(NULL);

    hdma_tx.XferCpltCallback = i2sDmaSendCpltCallback;
    hdma_tx.XferHalfCpltCallback = i2sDmaSendHalfCpltCallback;
    wm_i2s_transmit_dma(&hdma_tx, (uint16_t *)i2sdata, i2sblocksize*(sizeof(uint32_t)/sizeof(uint16_t)) );
    tls_os_queue_create(&audiodataqueue,10);
    tls_os_queue_create(&audiosizequeue,10);
//    tls_os_task_t audiotask;
//    void* audiostack= malloc(1024);
//    tls_os_task_create(&audiotask,
//                       "audiotask",
//                       audio_task,NULL,
//                       (u8*)audiostack,1024,
//                       1,0);
}


/**This function is the pcm output function, type is 0(PCM)*/
#if (WM_AUDIO_BOARD_INCLUDED == CFG_ON)
static uint32_t Stereo2Mono(void *audio_buf, uint32_t len, int LR)
{
    if (!audio_buf || !len || len % 4) {
        printf( "%s arg err\n", __func__);
        return 0;
    }
    int16_t *buf = audio_buf;
    uint32_t i = 0;
    LR = LR ? 1 : 0;

    for (i = 0; i < len / 4; i++) {
        buf[i] = buf[i * 2 + LR];
    }
    return len / 2;
}

int btif_co_avk_data_incoming(uint8_t type, uint8_t *p_data,uint16_t length)
{
    uint16_t fif0_len = 0;
    
    Stereo2Mono(p_data, length, 1);

    fif0_len = FifoSpaceLen();

    if(fif0_len < length/2)
    {
        printf("overwritten, fifo_space(%d), write to len(%d)\r\n", fif0_len, length/2);
    }

    FifoWrite(p_data, length/2);

    if(g_playing_state)
    {
        if(FifoDataLen()>4*1024)
        {
            PlayStart(g_sample_rate, g_bit_width, g_chan_count);
            g_playing_state = 0;
        }
    }
    
}
#else
#define		SAMPLE_RATE			43860U
#define     ORIGIN_RATE         44100U
#define		SAMPLE_RATE_DIV			731U
#define     ORIGIN_RATE_DIV         735U
//#define     FIXPOINT
#ifdef FIXPOINT
#define     OFFSETBIT               2
    #define     OFFSETFACTOR            (1<<OFFSETBIT)
#if OFFSETBIT>15
    #warning "sample calc will overflow!!change OFFSETBIT"
#endif
#if (SAMPLE_RATE_DIV*ORIGIN_RATE_DIV*SAMPLE_RATE_DIV)>0xffffffff
    #error "index calc will overflow!!change ORIGIN_RATE_DIV and SAMPLE_RATE_DIV"
#else
    #if (SAMPLE_RATE_DIV*SAMPLE_RATE_DIV*ORIGIN_RATE_DIV*OFFSETFACTOR)>0x7fffffff
        #error "offset calc will overflow!!change ORIGIN_RATE_DIV and SAMPLE_RATE_DIV and OFFSETBIT"
    #endif
#endif
#endif

#define	BLOCK_SIZE 4096

static bool resample_ch0(int16_t datain,int16_t*dataout){
    static uint32_t samplecount;
    static uint32_t currentindex;
    static int16_t x01,x05,x02;
    static int16_t x11,x15,x12;
    static int16_t x21,x25,x22;
    int16_t x32=datain;
    int16_t x31=x32/2;
    int16_t x35=x31/2;

    uint32_t index1=samplecount*SAMPLE_RATE_DIV/ORIGIN_RATE_DIV;
#ifdef FIXPOINT
    int32_t t=index1*ORIGIN_RATE_DIV*OFFSETFACTOR/SAMPLE_RATE_DIV+1*OFFSETFACTOR-samplecount*OFFSETFACTOR;
#else
    float t=(float)index1*ORIGIN_RATE_DIV/SAMPLE_RATE_DIV+1-samplecount;
#endif
    if(++samplecount==ORIGIN_RATE_DIV*SAMPLE_RATE_DIV)
        samplecount=0;
    if(index1==currentindex)
        goto skip;
    currentindex=index1;

    int16_t c1 = x25-x05;
    int16_t c2 = x01 -x15 -x35 +x22 -x12;
    int16_t c3 = x35 -x05 +x11 -x21 +x15-x25;
#ifdef FIXPOINT
    int32_t sample=((int32_t)(((((c3*t)+c2*OFFSETFACTOR)/OFFSETFACTOR*t)+c1*OFFSETFACTOR)/OFFSETFACTOR*t)+x11*OFFSETFACTOR)/OFFSETFACTOR*2;
#else
    float samplef=((((((c3*t)+c2)*t)+c1)*t)+x11)*2;
    int32_t sample=samplef;
#endif
    if(sample>0x7fff)
        sample=0x7fff;
    if(sample<-0x8000)
        sample=-0x8000;
    *dataout=sample;
    x01=x11;
    x02=x12;
    x05=x15;
    x11=x21;
    x12=x22;
    x15=x25;
    x21=x31;
    x22=x32;
    x25=x35;
    return true;
    skip:
    x01=x11;
    x02=x12;
    x05=x15;
    x11=x21;
    x12=x22;
    x15=x25;
    x21=x31;
    x22=x32;
    x25=x35;
    return false;
}

static bool resample_ch1(int16_t datain,int16_t*dataout){
    static uint32_t samplecount;
    static uint32_t currentindex;
    static int16_t x01,x05,x02;
    static int16_t x11,x15,x12;
    static int16_t x21,x25,x22;
    int16_t x32=datain;
    int16_t x31=x32/2;
    int16_t x35=x31/2;

    uint32_t index1=samplecount*SAMPLE_RATE_DIV/ORIGIN_RATE_DIV;
#ifdef FIXPOINT
    int32_t t=index1*ORIGIN_RATE_DIV*OFFSETFACTOR/SAMPLE_RATE_DIV+1*OFFSETFACTOR-samplecount*OFFSETFACTOR;
#else
    float t=(float)index1*ORIGIN_RATE_DIV/SAMPLE_RATE_DIV+1-samplecount;
#endif
    if(++samplecount==ORIGIN_RATE_DIV*SAMPLE_RATE_DIV)
        samplecount=0;
    if(index1==currentindex)
        goto skip;
    currentindex=index1;

    int16_t c1 = x25-x05;
    int16_t c2 = x01 -x15 -x35 +x22 -x12;
    int16_t c3 = x35 -x05 +x11 -x21 +x15-x25;
#ifdef FIXPOINT
    int32_t sample=((int32_t)(((((c3*t)+c2*OFFSETFACTOR)/OFFSETFACTOR*t)+c1*OFFSETFACTOR)/OFFSETFACTOR*t)+x11*OFFSETFACTOR)/OFFSETFACTOR*2;
#else
    float samplef=((((((c3*t)+c2)*t)+c1)*t)+x11)*2;
    int32_t sample=samplef;
#endif
    if(sample>0x7fff)
        sample=0x7fff;
    if(sample<-0x8000)
        sample=-0x8000;
    *dataout=sample;
    x01=x11;
    x02=x12;
    x05=x15;
    x11=x21;
    x12=x22;
    x15=x25;
    x21=x31;
    x22=x32;
    x25=x35;
    return true;
    skip:
    x01=x11;
    x02=x12;
    x05=x15;
    x11=x21;
    x12=x22;
    x15=x25;
    x21=x31;
    x22=x32;
    x25=x35;
    return false;
}

int btif_co_avk_data_incoming(uint8_t type, uint8_t *p_data,uint16_t length)
{
    int16_t *datap=(int16_t*)p_data;
    for(int i=0;i<length/(g_bit_width/8);i+=2){
        currentpt_w+=resample_ch0(datap[i],&(loopbuffer[currentpt_w]));
        if(currentpt_w>=loopblocksize)
            currentpt_w=0;
        currentpt_w+=resample_ch1(datap[i+1],&(loopbuffer[currentpt_w]));
        if(currentpt_w>=loopblocksize)
            currentpt_w=0;
//        i2sdatabuf[currentpt++]=datap[i]*65536;
//        if(currentpt>=i2sblocksize/2){
//            currentaudiobuf=0;
//            if(currentpt>=i2sblocksize)
//            {
//                currentpt=0;
//            }
//        }else
//            currentaudiobuf=1;
    }
//    int offset=(1-currentaudiobuf)*i2sblocksize/2;
//    uint32_t *p=i2sdatabuf+offset;
//    currentaudiobuf=!currentaudiobuf;

//    void* datacp= malloc(length);
//    if(datacp){
//        memcpy(datacp,p_data,length);
//        tls_os_queue_send(audiodataqueue,datacp,0);
//        tls_os_queue_send(audiosizequeue,(void*)length,0);
//    }
}
#endif
static void bta2dp_connection_state_callback(tls_btav_connection_state_t state, tls_bt_addr_t *bd_addr)
{
    switch(state)
    {
        case WM_BTAV_CONNECTION_STATE_DISCONNECTED:
            TLS_BT_APPL_TRACE_DEBUG("BTAV_CONNECTION_STATE_DISCONNECTED\r\n");
            //sbc_ABV_buffer_reset();
            break;

        case WM_BTAV_CONNECTION_STATE_CONNECTING:
            TLS_BT_APPL_TRACE_DEBUG("BTAV_CONNECTION_STATE_CONNECTING\r\n");
            break;

        case WM_BTAV_CONNECTION_STATE_CONNECTED:
            TLS_BT_APPL_TRACE_DEBUG("BTAV_CONNECTION_STATE_CONNECTED\r\n");
            break;

        case WM_BTAV_CONNECTION_STATE_DISCONNECTING:
            TLS_BT_APPL_TRACE_DEBUG("BTAV_CONNECTION_STATE_DISCONNECTING\r\n");
            break;

        default:
            TLS_BT_APPL_TRACE_DEBUG("UNKNOWN BTAV_AUDIO_STATE...\r\n");
    }
}


static void bta2dp_audio_state_callback(tls_btav_audio_state_t state, tls_bt_addr_t *bd_addr)
{
    switch(state)
    {
        case WM_BTAV_AUDIO_STATE_STARTED:
            TLS_BT_APPL_TRACE_DEBUG("BTAV_AUDIO_STATE_STARTED\r\n");
            //sbc_ABV_buffer_reset();
            //VolumeControl(16);
            //PlayStart(g_sample_rate, g_bit_width);
            g_playing_state = 1;
            break;

        case WM_BTAV_AUDIO_STATE_STOPPED:
            TLS_BT_APPL_TRACE_DEBUG("BTAV_AUDIO_STATE_STOPPED\r\n");
			#if (WM_AUDIO_BOARD_INCLUDED == CFG_ON)
            PlayStop();
			#endif
            break;

        case WM_BTAV_AUDIO_STATE_REMOTE_SUSPEND:
            TLS_BT_APPL_TRACE_DEBUG("BTAV_AUDIO_STATE_REMOTE_SUSPEND\r\n");
			#if (WM_AUDIO_BOARD_INCLUDED == CFG_ON)
            PlayStop();
			#endif
            break;

        default:
            TLS_BT_APPL_TRACE_DEBUG("UNKNOWN BTAV_AUDIO_STATE...\r\n");
    }
}
static void bta2dp_audio_config_callback(tls_bt_addr_t *bd_addr, uint32_t sample_rate, uint8_t channel_count)
{
    TLS_BT_APPL_TRACE_DEBUG("CBACK:%02x:%02x:%02x:%02x:%02x:%02x::sample_rate=%d, channel_count=%d\r\n",
                bd_addr->address[0], bd_addr->address[1], bd_addr->address[2], bd_addr->address[3], bd_addr->address[4], bd_addr->address[5], sample_rate, channel_count);
    g_sample_rate = sample_rate;    
}
static void bta2dp_audio_payload_callback(tls_bt_addr_t *bd_addr, uint8_t format, uint8_t *p_data, uint16_t length)
{
	//TLS_BT_APPL_TRACE_DEBUG("CBACK(%s): length=%d\r\n", __FUNCTION__, length);
}

static void btavrcp_remote_features_callback(tls_bt_addr_t *bd_addr, tls_btrc_remote_features_t features)
{
    TLS_BT_APPL_TRACE_DEBUG("CBACK(%s): features:%d\r\n", __FUNCTION__, features);
}
static void btavrcp_get_play_status_callback()
{
    TLS_BT_APPL_TRACE_DEBUG("CBACK(%s): \r\n", __FUNCTION__);
}
static void btavrcp_get_element_attr_callback(uint8_t num_attr, tls_btrc_media_attr_t *p_attrs)
{
    TLS_BT_APPL_TRACE_DEBUG("CBACK(%s): num_attr:%d, param:%d\r\n", __FUNCTION__, num_attr);
}
static void btavrcp_register_notification_callback(tls_btrc_event_id_t event_id, uint32_t param)
{
    TLS_BT_APPL_TRACE_DEBUG("CBACK(%s): event_id:%d, param:%d\r\n", __FUNCTION__, event_id, param);
}

static void btavrcp_volume_change_callback(uint8_t volume, uint8_t ctype)
{
    TLS_BT_APPL_TRACE_DEBUG("CBACK: volume:%d, type:%d\r\n", volume, ctype);
}
static void btavrcp_passthrough_command_callback(int id, int pressed)
{
    TLS_BT_APPL_TRACE_DEBUG("CBACK(%s): id:%d, pressed:%d\r\n", __FUNCTION__, id, pressed);
}



static void btavrcp_passthrough_response_callback(int id, int pressed)
{
}

static void btavrcp_connection_state_callback(bool state, tls_bt_addr_t *bd_addr)
{
    TLS_BT_APPL_TRACE_DEBUG("CBACK:%02x:%02x:%02x:%02x:%02x:%02x::state:%d\r\n",
                bd_addr->address[0], bd_addr->address[1], bd_addr->address[2], bd_addr->address[3], bd_addr->address[4], bd_addr->address[5], state);
}

static void wm_a2dp_sink_callback(tls_bt_av_evt_t evt, tls_bt_av_msg_t *msg)
{
	switch(evt)
	{
		case WMBT_A2DP_CONNECTION_STATE_EVT:
			bta2dp_connection_state_callback(msg->av_connection_state.stat, msg->av_connection_state.bd_addr);
			break;
		case WMBT_A2DP_AUDIO_STATE_EVT:
			bta2dp_audio_state_callback(msg->av_audio_state.stat, msg->av_audio_state.bd_addr);
			break;
		case WMBT_A2DP_AUDIO_CONFIG_EVT:
			bta2dp_audio_config_callback(msg->av_audio_config.bd_addr, msg->av_audio_config.sample_rate, msg->av_audio_config.channel_count);
			break;
		case WMBT_A2DP_AUDIO_PAYLOAD_EVT:
			bta2dp_audio_payload_callback(msg->av_audio_payload.bd_addr,msg->av_audio_payload.audio_format,msg->av_audio_payload.payload, msg->av_audio_payload.payload_length);
			break;
	}
}

static void wm_btrc_callback(tls_btrc_evt_t evt, tls_btrc_msg_t *msg)
{
	switch(evt)
	{
		case WM_BTRC_REMOTE_FEATURE_EVT:
			btavrcp_remote_features_callback(msg->remote_features.bd_addr, msg->remote_features.features);
			break;
		case WM_BTRC_GET_PLAY_STATUS_EVT:
			btavrcp_get_play_status_callback();
			break;
		case WM_BTRC_GET_ELEMENT_ATTR_EVT:
			btavrcp_get_element_attr_callback(msg->get_element_attr.num_attr, msg->get_element_attr.p_attrs);
			break;
		case WM_BTRC_REGISTER_NOTIFICATION_EVT:
			btavrcp_register_notification_callback(msg->register_notification.event_id, msg->register_notification.param);
			break;
		case WM_BTRC_VOLUME_CHANGED_EVT:
			btavrcp_volume_change_callback(msg->volume_change.ctype, msg->volume_change.volume);
			break;
		case WM_BTRC_PASSTHROUGH_CMD_EVT:
			btavrcp_passthrough_command_callback(msg->passthrough_cmd.id, msg->passthrough_cmd.key_state);
			break;
		default:
			TLS_BT_APPL_TRACE_VERBOSE("unhandled wm_btrc_callback, evt=%d\r\n", evt);
			break;
	}
}


static void wm_btrc_ctrl_callback(tls_btrc_ctrl_evt_t evt, tls_btrc_ctrl_msg_t *msg)
{
	switch(evt)
	{
		case WM_BTRC_PASSTHROUGH_CMD_EVT:
			btavrcp_passthrough_response_callback(msg->passthrough_rsp.id, msg->passthrough_rsp.key_state);
			break;
		default:
			TLS_BT_APPL_TRACE_VERBOSE("unhandled wm_btrc_ctrl_callback, evt=%d\r\n", evt);
			break;	
	}
}

tls_bt_status_t tls_bt_enable_a2dp_sink()
{
	tls_bt_status_t status;

	status = tls_bt_av_sink_init(wm_a2dp_sink_callback);
	if(status != TLS_BT_STATUS_SUCCESS)
	{
		TLS_BT_APPL_TRACE_ERROR("tls_bt_av_sink_init failed, status=%d\r\n", status);
		return status;
	}

	status = tls_btrc_init(wm_btrc_callback);
	if(status != TLS_BT_STATUS_SUCCESS)
	{
		TLS_BT_APPL_TRACE_ERROR("tls_btrc_init failed, status=%d\r\n", status);
		tls_bt_av_sink_deinit();
		return status;
	}
	
	status = tls_btrc_ctrl_init(wm_btrc_ctrl_callback);
	if(status != TLS_BT_STATUS_SUCCESS)
	{
		TLS_BT_APPL_TRACE_ERROR("tls_btrc_init failed, status=%d\r\n", status);
		tls_bt_av_sink_deinit();
		tls_btrc_deinit();
		return status;
	}	
	#if (WM_AUDIO_BOARD_INCLUDED == CFG_ON)
    AudioInit(12*1024);
	#endif
    audio_init();
	return status;
}

tls_bt_status_t tls_bt_disable_a2dp_sink()
{
	tls_bt_av_sink_deinit();
	tls_btrc_deinit();
	tls_btrc_ctrl_deinit();

	return TLS_BT_STATUS_SUCCESS;
}

#endif

