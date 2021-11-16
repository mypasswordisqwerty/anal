//
//  bluepill.c
//  sigrok4DSL
//
//  Created by john on 22.04.2021.
//

#include "libsigrok.h"
#include "libsigrok-internal.h"
#include <sys/fcntl.h>
#include <sys/poll.h>
#include <unistd.h>
#include <termios.h>
#include "bpread.h"


#define DEVICE "/dev/tty.usbmodem4976124F34301"

#define LOG_PREFIX "pills: "

static struct sr_dev_mode devmode = {LOGIC, "Logic Analyzer", "LA", "la", "la.svg"};
SR_PRIV struct sr_dev_driver pills_driver_info;
static struct sr_dev_driver *di = &pills_driver_info;

static const uint64_t samplerates[] = {
    SR_MHZ(36),
};

static const int32_t sessions[] = {
    SR_CONF_SAMPLERATE,
    SR_CONF_LIMIT_SAMPLES,
};

static int init(struct sr_context *sr_ctx)
{
    bpread.samplerate = SR_MHZ(36);
    bpread.samples = SR_MHZ(36);
    bpread.msec = 1000;
    bpread.fd = -1;
    sr_log_loglevel_set(SR_LOG_INFO);
    sr_info("init");
    return std_hw_init(sr_ctx, di, LOG_PREFIX);
}

static void probe_init(struct sr_dev_inst *sdi)
{
    GSList *l;

    for (l = sdi->channels; l; l = l->next) {
        struct sr_channel *probe = (struct sr_channel *)l->data;
        probe->bits = 1;
        probe->vdiv = 1000;
        probe->vfactor = 1;
        probe->coupling = SR_DC_COUPLING;
        probe->trig_value = (1 << (probe->bits - 1));
        probe->hw_offset = (1 << (probe->bits - 1));
        probe->offset = probe->index;

        probe->map_default = TRUE;
        probe->map_unit = "V";
        probe->map_min = -(probe->vdiv * probe->vfactor * DS_CONF_DSO_VDIVS / 2000.0);
        probe->map_max = probe->vdiv * probe->vfactor * DS_CONF_DSO_VDIVS / 2000.0;
    }
}

static GSList *scan(GSList *options){
    sr_info("scan");
    struct sr_dev_inst *sdi;
    struct drv_context *drvc;
    GSList *devices = NULL;
    drvc = di->priv;

    sdi = sr_dev_inst_new(LOGIC, 0, SR_ST_INITIALIZING, "bjfn", "pills", "0.1");
    if (!sdi) {
        sr_err("Device instance creation failed.");
        return NULL;
    }
    sdi->priv = NULL;
    sdi->driver = di;

    struct sr_channel *probe;
    if ((probe = sr_channel_new(0, SR_CHANNEL_LOGIC, TRUE, "B3"))){
        sdi->channels = g_slist_append(sdi->channels, probe);
    }
    if ((probe = sr_channel_new(1, SR_CHANNEL_LOGIC, TRUE, "B4"))){
        sdi->channels = g_slist_append(sdi->channels, probe);
    }
    if ((probe = sr_channel_new(2, SR_CHANNEL_LOGIC, TRUE, "B5 3.3"))){
        sdi->channels = g_slist_append(sdi->channels, probe);
    }
    if ((probe = sr_channel_new(3, SR_CHANNEL_LOGIC, TRUE, "B6"))){
        sdi->channels = g_slist_append(sdi->channels, probe);
    }
    if ((probe = sr_channel_new(4, SR_CHANNEL_LOGIC, TRUE, "B7"))){
        sdi->channels = g_slist_append(sdi->channels, probe);
    }
    if ((probe = sr_channel_new(5, SR_CHANNEL_LOGIC, TRUE, "B8"))){
        sdi->channels = g_slist_append(sdi->channels, probe);
    }
    if ((probe = sr_channel_new(6, SR_CHANNEL_LOGIC, TRUE, "B9"))){
        sdi->channels = g_slist_append(sdi->channels, probe);
    }
    if ((probe = sr_channel_new(7, SR_CHANNEL_LOGIC, TRUE, "B10"))){
        sdi->channels = g_slist_append(sdi->channels, probe);
    }
    if ((probe = sr_channel_new(8, SR_CHANNEL_LOGIC, TRUE, "B11"))){
        sdi->channels = g_slist_append(sdi->channels, probe);
    }

    probe_init(sdi);

    devices = g_slist_append(devices, sdi);
    drvc->instances = g_slist_append(drvc->instances, sdi);
    return devices;
}

static GSList *dev_list(void)
{
    sr_dbg("devlist");
    return ((struct drv_context *)(di->priv))->instances;
}

static const GSList *dev_mode_list(const struct sr_dev_inst *sdi)
{
    sr_info("modelist");
    GSList *l = NULL;
    l = g_slist_append(l, &devmode);
    return l;
}

static int clear_instances(void) {
    sr_info("clear instances");
    return SR_OK;
}

static int configure(int fd){
    struct termios tty;
    if (tcgetattr (fd, &tty) != 0)
    {
        sr_err ("error %d from tcgetattr: %s", errno, strerror(errno));
        return FALSE;
    }
    cfsetospeed (&tty, B115200);
    cfsetispeed (&tty, B115200);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        sr_err("error %d from tcsetattr: %s", errno, strerror(errno));
        return FALSE;
    }
    return TRUE;
}

static int dev_open(struct sr_dev_inst *sdi){
    sr_info("open");
    bpread.fd = open(DEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (bpread.fd < 0){
        sr_err("Error %i from open: %s", errno, strerror(errno));
        return SR_ERR;
    }
    if(!configure(bpread.fd)){
        close(bpread.fd);
        return SR_ERR;
    }

    sdi->status = SR_ST_ACTIVE;
    return SR_OK;
}
static int dev_close(struct sr_dev_inst *sdi){
    sr_info("close");
    if (bpread.fd>=0) {
        close(bpread.fd);
        bpread.fd = -1;
    }
    sdi->status = SR_ST_INACTIVE;
    return SR_OK;
}
static int cleanup(void) {
    sr_info("cleanup");
    struct drv_context *drvc;

    if (!(drvc = di->priv))
        return SR_OK;
    g_slist_free(drvc->instances);
    drvc->instances = NULL;

    return SR_OK;
}

static int config_get(int id, GVariant **data, const struct sr_dev_inst *sdi,
                      const struct sr_channel *ch,
                      const struct sr_channel_group *cg) {
    sr_dbg("cfg get %d", id);
    switch (id) {
        case SR_CONF_SAMPLERATE:
            *data = g_variant_new_uint64(bpread.samplerate);
            break;
        case SR_CONF_LIMIT_SAMPLES:
            *data = g_variant_new_uint64(bpread.samples);
            break;
        case SR_CONF_LIMIT_MSEC:
            *data = g_variant_new_uint64(bpread.msec);
            break;
        case SR_CONF_HW_DEPTH:
            *data = g_variant_new_uint64(SR_Gn(36));
            break;
        default:
            return SR_ERR_NA;
    }
    return SR_OK;
}
static int config_set(int id, GVariant *data, struct sr_dev_inst *sdi,
                      struct sr_channel *ch,
                      struct sr_channel_group *cg){
    sr_dbg("cfg set %d", id);
    switch (id) {
        case SR_CONF_SAMPLERATE:
            bpread.samplerate = g_variant_get_uint64(data);
            break;
        case SR_CONF_LIMIT_SAMPLES:
            bpread.samples = g_variant_get_uint64(data);
            break;
        case SR_CONF_LIMIT_MSEC:
            bpread.msec = g_variant_get_uint64(data);
            break;
    }
    //return SR_ERR_NA;
    return SR_OK;
}

static int config_list(int key, GVariant **data, const struct sr_dev_inst *sdi,
                       const struct sr_channel_group *cg){
    sr_dbg("cfg list");
    GVariant *gvar;
    GVariantBuilder gvb;
    switch (key) {
        case SR_CONF_SAMPLERATE:
            g_variant_builder_init(&gvb, G_VARIANT_TYPE("a{sv}"));
            gvar = g_variant_new_from_data(G_VARIANT_TYPE("at"),
                                           samplerates, sizeof(uint64_t), TRUE, NULL, NULL);
            g_variant_builder_add(&gvb, "{sv}", "samplerates", gvar);
            *data = g_variant_builder_end(&gvb);
            break;
        case SR_CONF_DEVICE_SESSIONS:
            *data = g_variant_new_from_data(G_VARIANT_TYPE("ai"),
                    sessions, ARRAY_SIZE(sessions)*sizeof(int32_t), TRUE, NULL, NULL);
            break;
        default:
            return SR_ERR_NA;
    }
    return SR_OK;
}

void file_clean(int fd){
    struct pollfd pfd = {fd, POLLIN};
    char buf[1024];
    while(poll(&pfd, 1, 1)) {
        while(read(fd, buf, 1024)>0);
        pfd.revents = 0;
    }
}

static int dev_acquisition_start(struct sr_dev_inst *sdi, void *cb_data) {
    sr_info("acq start");
    if (sdi->status != SR_ST_ACTIVE)
        return SR_ERR_DEV_CLOSED;
    file_clean(bpread.fd);
    bpread_start();
    sr_session_source_add(bpread.fd, G_IO_IN, 100, bpread_receive_data, sdi);
    std_session_send_df_header(sdi, LOG_PREFIX);
    return SR_OK;
}

static int dev_acquisition_stop(const struct sr_dev_inst *sdi, void *cb_data) {
    sr_info("acq stop");
    bpread_stop();
    sr_session_source_remove(bpread.fd);
    struct sr_datafeed_packet packet;
    packet.type = SR_DF_END;
    packet.status = SR_PKT_OK;
    sr_session_send(sdi, &packet);
    return SR_OK;
}

static int dev_status_get(const struct sr_dev_inst *sdi, struct sr_status *status, gboolean prg){
    sr_dbg("status");
    return SR_ERR;
}


SR_PRIV struct sr_dev_driver pills_driver_info = {
    .name = "pills",
    .longname = "STM32 Pills driver",
    .api_version = 1,
    .init = init,
    .cleanup = cleanup,
    .scan = scan,
    .dev_list = dev_list,
    .dev_mode_list = dev_mode_list,
    .dev_clear = clear_instances,
    .config_get = config_get,
    .config_set = config_set,
    .config_list = config_list,
    .dev_open = dev_open,
    .dev_close = dev_close,
    .dev_status_get = dev_status_get,
    .dev_acquisition_start = dev_acquisition_start,
    .dev_acquisition_stop = dev_acquisition_stop,
    .priv = NULL,
};
