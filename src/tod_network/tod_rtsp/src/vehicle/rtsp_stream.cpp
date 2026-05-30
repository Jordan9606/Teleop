/**
 * @file rtsp_stream.cpp
 * @brief This file defines the `RtspStream` class for managing video streaming over RTSP using GStreamer. The class provides functionality to set up an RTSP stream, handle video configurations, and manage video encoding, including bitrate adjustments, cropping, and scaling. It also supports receiving image data via ROS2 messages and dynamically reconfiguring stream settings.
 * 
 * The `RtspStream` class integrates with ROS2 to receive image data, configure GStreamer pipelines, and publish the video stream over RTSP. The class supports dynamic reconfiguration of video settings, including bitrate, resolution, and cropping, and includes mechanisms for managing video data through callbacks and maintaining stream activity. It ensures that stream settings are applied properly and provides methods to check and update stream configurations.
 * 
 * @version 1.0
 *
 * @copyright TUMFTM 2024
 */
#include "rtsp_stream.hpp"

using namespace tod_rtsp;

RtspStream::RtspStream(
    const std::string name,
    std::string ip,
    std::string port,
    int bitrate,
    videoConfig video_config,
    const float inactivity_timeout,
    std::shared_ptr<rclcpp::Logger> logger)
    : name_(name),
      ip_(ip),
      port_(port),
      inactivity_timeout_(inactivity_timeout),  // Correct initialization
      logger_(logger), // Proper logger initialization
      video_config_(std::make_shared<videoConfig>(video_config)), // Initialize video_config_ with a copy of video_config
      latest_image_(std::make_shared<sensor_msgs::msg::Image>()),  // Initialize latest_image_
      pipeline_config_{.bitrate = bitrate}, // Initialize pipeline_config
      gst_data_request_(false),  // Atomic flags start as false
      ros2_new_data_(false),
      is_active_(false) {  // Initialize to the current time

    gst_last_request_ = std::chrono::system_clock::now();
    latest_image_->width = video_config.width;
    latest_image_->height = video_config.height;
    latest_image_->step = (video_config.raw_step != -1) ? video_config.raw_step : video_config.width;
    latest_image_->encoding = "i420";
}



void RtspStream::factory_gst_video_pipeline(GstRTSPMountPoints *_gstMounts){
    RCLCPP_INFO(*this->logger_, "Factory gst video pipeline  %s", this->name_.c_str());

    std::string h264_launch_string =
        "( appsrc name=mysrc is-live=true ! videoconvert ! queue ! videocrop name=mycrop !"
        " videoscale ! capsfilter name=myscale !"
        " capsfilter caps=video/x-raw,format=I420 !"
        " x264enc name=myenc "
        " tune=zerolatency speed-preset=superfast"
        " sliced-threads=true byte-stream=true threads=1"
        " key-int-max=15 intra-refresh=true !"
        " identity name=idH264 signal-handoffs=true ! queue! "
        " h264parse ! rtph264pay name=pay0 pt=96 )";

    // create a pipeline that produces streams
    this->factory = gst_rtsp_media_factory_new();
    // configure this pipeline
    gst_rtsp_media_factory_set_launch(this->factory, h264_launch_string.c_str());
    // shared media: one persistent pipeline per stream — prevents on_unprepared race on fast reconnect
    gst_rtsp_media_factory_set_shared(this->factory, TRUE);
    // callback when client connects
    g_signal_connect(this->factory, "media-configure", (GCallback)static_gst_media_configure, this);
    // attach factory to url
    gst_rtsp_mount_points_add_factory(_gstMounts, this->name_.c_str(), this->factory);
    std::string url{"rtsp://" + this->ip_ + ":" + this->port_ + this->name_}; 
    RCLCPP_INFO(*this->logger_, "Stream of size (%dx%d) and encoding %s available at %s", 
        this->video_config_->width, 
        this->video_config_->height, 
        this->latest_image_->encoding.c_str(), url.c_str());
}

void RtspStream::refresh(GstRTSPMountPoints *gstMounts){
    if(this->ros2_new_data_){
        if (!this->factory){
            RCLCPP_INFO(*this->logger_, "Factory: %s",this->name_.c_str());
            this->factory_gst_video_pipeline(gstMounts);
        }
        if (this->gst_data_request_ && this->is_active_) //! Not the best solution - log message upon connecting is weird now 
        {
            this->push_data();
        }
        else if(is_inactive()){
            RCLCPP_INFO(*this->logger_, "Stream %s is paused due to inactivity", this->name_.c_str());
            std::lock_guard lock(this->mutex_);
            this->is_active_ = false;           
        }
    }
}

void RtspStream::reset(){
    std::lock_guard lock(this->mutex_);
    if (!this->videocrop_ || !this->scalingFilter_) return;
    g_object_set(G_OBJECT(this->videocrop_), "top", 0, "bottom", 0, "left", 0, "right", 0, nullptr);
    GstCaps *caps = gst_caps_new_simple("video/x-raw",
        "width", G_TYPE_INT, this->video_config_->width,
        "height", G_TYPE_INT, this->video_config_->height, nullptr);
    g_object_set(G_OBJECT(this->scalingFilter_), "caps", caps, nullptr);
    gst_caps_unref(caps);
}


void RtspStream::ros2_image_callback(const sensor_msgs::msg::Image::ConstPtr &msg){
    std::lock_guard lock(this->mutex_);
    if (this->ros2_new_data_ && !this->is_active_) {
        return; // no unnecessary copies
    }
    this->latest_image_ = std::make_shared<sensor_msgs::msg::Image>(*msg);;
    this->ros2_new_data_ = true;
}

bool RtspStream::set_bitrate(int bitrate){
    std::lock_guard lock(this->mutex_);
    bool success = false;
    this->pipeline_config_.bitrate = bitrate; 
    if (this->encoder_) {       
        g_object_set(G_OBJECT(this->encoder_), "bitrate", this->pipeline_config_.bitrate, nullptr);
        RCLCPP_INFO(*this->logger_, "Updated bitrate for %s in set_bitrate_for_streams to %d", this->name_.c_str(), this->pipeline_config_.bitrate);
        success = true;
    }
    else{
        RCLCPP_ERROR(*this->logger_, "encoder_ not initialized for %s, bitrate not set", this->name_.c_str());
    }
    return success;
}

int RtspStream::get_bitrate(){
    return this->pipeline_config_.bitrate;
}

bool RtspStream::is_stream(std::string camera_name){
    return camera_name == this->name_;
}

bool RtspStream::update_config(videoConfig config){
    if (!this->videocrop_ || !this->scalingFilter_){ // catch error when gstreamer elements not available
        RCLCPP_INFO(*this->logger_, "Not cropping/scaling as no videocrop_/videoscale for %s initialized", this->name_.c_str());
        return false;
    }
    std::lock_guard lock(this->mutex_);
    const int fullWidth = this->video_config_->width;
    const int fullHeight = this->video_config_->height;
    const int minPxSize = 16;

    // saturate set values according to full width and height
    // also make sure there are only even numbers of pixels
    config.width = std::clamp(config.width - config.width % 8, minPxSize, fullWidth);
    config.height = std::clamp(config.height - config.height % 8, minPxSize, fullHeight);
    config.offset_width = std::clamp(config.offset_width - config.offset_width % 8, 0, fullWidth - minPxSize);
    config.offset_height = std::clamp(config.offset_height - config.offset_height % 8, 0, fullHeight - minPxSize);

    // correct width/height/offset if through new width offset, image goes beyond full width
    config.width = std::min(config.width, fullWidth - config.offset_width);
    config.height = std::min(config.height, fullHeight - config.offset_height);
    config.offset_width = std::min(config.offset_width, fullWidth - config.width);
    config.offset_height = std::min(config.offset_height, fullHeight - config.height);

    // final cropping
    const int leftCrop = config.offset_width;
    const int rightCrop = fullWidth - config.width - config.offset_width;
    const int bottomCrop = config.offset_height;
    const int topCrop = fullHeight - config.height - config.offset_height;

    // final scaling - round to integer and even pixel numbers multiple of 8
    std::string scalingStr = config.scaling_factor;
    if (scalingStr.size() < 2) {
        RCLCPP_ERROR(*this->logger_, "Invalid scaling_factor '%s' for %s — must be at least 2 chars",
                     scalingStr.c_str(), this->name_.c_str());
        return false;
    }
    // scaling[1] is 'p' in new_config, replace with '.' to cast as double
    scalingStr.at(1) = '.';
    const double scalingFactor = std::stod(scalingStr);
    int actual_width = std::max(minPxSize, static_cast<int>(config.width * scalingFactor));
    actual_width -= actual_width % 8;
    int actual_height = std::max(minPxSize, static_cast<int>(config.height * scalingFactor));
    actual_height -= actual_height % 8;
    // try to set values in gstreamer
    try {
        g_object_set(G_OBJECT(this->videocrop_),
                    "top", topCrop,
                    "bottom", bottomCrop,
                    "left", leftCrop,
                    "right", rightCrop, 
            nullptr);        

        GstCaps *new_caps = gst_caps_new_simple("video/x-raw",
            "width", G_TYPE_INT, actual_width,
            "height", G_TYPE_INT, actual_height, nullptr);
        g_object_set(G_OBJECT(this->scalingFilter_), "caps", new_caps, nullptr);
        gst_caps_unref(new_caps);

        RCLCPP_INFO(*this->logger_, "set (aw,ah,w,h,ow,oh) = (%d,%d,%d,%d,%d,%d) at scaling factor = %f for %s", 
                actual_width, actual_height, config.width, config.height, config.offset_width, config.offset_height, scalingFactor, this->name_.c_str());
        return true;
    }
    catch (const std::exception &e){
        RCLCPP_ERROR(*this->logger_, "Error in updateConfig: %s", e.what());
    }
    return false;
}

bool RtspStream::update_activity(bool paused){
    std::lock_guard lock(this->mutex_);
    this->is_active_ = !paused;        
    return true;
}

//PRIVATE: ___________________________________________________

void RtspStream::gst_media_configure(GstRTSPMediaFactory *factory, GstRTSPMedia *media){
    GstElement *element = gst_rtsp_media_get_element(media);

    this->encoder_ = gst_bin_get_by_name_recurse_up(GST_BIN(element), "myenc");
    if (!this->encoder_) {
        RCLCPP_ERROR(*this->logger_, "Pipeline element 'myenc' not found for %s", this->name_.c_str());
        gst_object_unref(element);
        return;
    }
    g_object_set(G_OBJECT(this->encoder_), "bitrate", this->pipeline_config_.bitrate, nullptr);

    this->videocrop_ = gst_bin_get_by_name_recurse_up(GST_BIN(element), "mycrop");
    if (!this->videocrop_) {
        RCLCPP_ERROR(*this->logger_, "Pipeline element 'mycrop' not found for %s", this->name_.c_str());
        gst_object_unref(this->encoder_); this->encoder_ = nullptr;
        gst_object_unref(element);
        return;
    }
    g_object_set(G_OBJECT(this->videocrop_), "top", 0, "bottom", 0, "left", 0, "right", 0, nullptr);

    this->scalingFilter_ = gst_bin_get_by_name_recurse_up(GST_BIN(element), "myscale");
    if (!this->scalingFilter_) {
        RCLCPP_ERROR(*this->logger_, "Pipeline element 'myscale' not found for %s", this->name_.c_str());
        gst_object_unref(this->encoder_); this->encoder_ = nullptr;
        gst_object_unref(this->videocrop_); this->videocrop_ = nullptr;
        gst_object_unref(element);
        return;
    }
    {
        GstCaps *caps = gst_caps_new_simple("video/x-raw",
            "width", G_TYPE_INT, this->video_config_->width,
            "height", G_TYPE_INT, this->video_config_->height, nullptr);
        g_object_set(G_OBJECT(this->scalingFilter_), "caps", caps, nullptr);
        gst_caps_unref(caps);
    }

    // select image format as received in ros image callback
    std::string format = get_gst_encoding(this->latest_image_->encoding);

    this->appsrc_ = gst_bin_get_by_name_recurse_up(GST_BIN(element), "mysrc");
    if (!this->appsrc_) {
        RCLCPP_ERROR(*this->logger_, "Pipeline element 'mysrc' not found for %s", this->name_.c_str());
        gst_object_unref(this->encoder_); this->encoder_ = nullptr;
        gst_object_unref(this->videocrop_); this->videocrop_ = nullptr;
        gst_object_unref(this->scalingFilter_); this->scalingFilter_ = nullptr;
        gst_object_unref(element);
        return;
    }
    {
        GstCaps *caps = gst_caps_new_simple("video/x-raw",
            "format", G_TYPE_STRING, format.c_str(),
            "width", G_TYPE_INT, this->video_config_->width,
            "height", G_TYPE_INT, this->video_config_->height, nullptr);
        g_object_set(G_OBJECT(this->appsrc_),
                     "stream-type", GST_APP_STREAM_TYPE_STREAM,
                     "format", GST_FORMAT_TIME,
                     "is-live", TRUE,
                     "do-timestamp", TRUE,
                     "caps", caps, nullptr);
        gst_caps_unref(caps);
    }

    // install the callback that will be called when a buffer is needed
    g_signal_connect(this->appsrc_, "need-data", (GCallback)static_gst_need_data, this);

    GstElement *rtph264pay = gst_bin_get_by_name(GST_BIN(element), "pay0");
    if (!rtph264pay) {
        RCLCPP_ERROR(*this->logger_, "Pipeline element 'pay0' not found for %s", this->name_.c_str());
        gst_object_unref(this->appsrc_); this->appsrc_ = nullptr;
        gst_object_unref(this->encoder_); this->encoder_ = nullptr;
        gst_object_unref(this->videocrop_); this->videocrop_ = nullptr;
        gst_object_unref(this->scalingFilter_); this->scalingFilter_ = nullptr;
        gst_object_unref(element);
        return;
    }
    GstPad *src_pad = gst_element_get_static_pad(rtph264pay, "src");
    if (!src_pad) {
        RCLCPP_ERROR(*this->logger_, "'src' pad not found in pay0 for %s", this->name_.c_str());
        gst_object_unref(rtph264pay);
    } else {
        gst_pad_add_probe(src_pad, GST_PAD_PROBE_TYPE_BUFFER, (GstPadProbeCallback)add_rtp_timestamp_probe, this, nullptr);
        gst_object_unref(src_pad);
        gst_object_unref(rtph264pay);
    }

    gst_object_unref(this->appsrc_);
    gst_object_unref(this->encoder_);
    gst_object_unref(this->videocrop_);
    gst_object_unref(this->scalingFilter_);
    gst_object_unref(element);

    g_signal_connect(media, "unprepared", (GCallback)static_on_unprepared, this);

    std::lock_guard lock(this->mutex_);
    this->is_active_ = true;
    RCLCPP_INFO(*this->logger_, "Client connected for %s in GStreamer image format %s", this->name_.c_str(), format.c_str());
}

void RtspStream::on_unprepared(GstRTSPMedia */*media*/) {
    std::lock_guard lock(this->mutex_);
    // Pipeline is being torn down — null element pointers so push_data/reset don't access freed memory
    this->appsrc_ = nullptr;
    this->encoder_ = nullptr;
    this->videocrop_ = nullptr;
    this->scalingFilter_ = nullptr;
    this->gst_data_request_ = false;
    this->is_active_ = false;
}

void RtspStream::static_on_unprepared(GstRTSPMedia *media, RtspStream *stream) {
    stream->on_unprepared(media);
}

void RtspStream::static_gst_media_configure(GstRTSPMediaFactory *factory, GstRTSPMedia *media, RtspStream *stream){
    stream->gst_media_configure(factory, media);
}

void RtspStream::gst_need_data(GstElement *appsrc_, guint unused){
    std::lock_guard lock(this->mutex_);
    this->gst_data_request_ = true;
    // Only use ROS image stamp if it's non-zero — an uninitialized stamp (epoch) would make
    // is_inactive() return true immediately, pausing the stream before any data arrives
    if (this->latest_image_->header.stamp.sec != 0 || this->latest_image_->header.stamp.nanosec != 0) {
        this->gst_last_request_ = std::chrono::system_clock::time_point(
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::seconds(this->latest_image_->header.stamp.sec))
            + std::chrono::nanoseconds(this->latest_image_->header.stamp.nanosec));
    } else {
        this->gst_last_request_ = std::chrono::system_clock::now();
    }
    this->gst_last_request_ns_.store(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            this->gst_last_request_.time_since_epoch()).count());
    if (!this->is_active_)
    {
        RCLCPP_INFO(*this->logger_, "Stream %s is playing", this->name_.c_str());
        this->is_active_ = true;
    }
}

void RtspStream::static_gst_need_data(GstElement *appsrc_, guint unused, RtspStream *stream){
    stream->gst_need_data(appsrc_, unused);
}

void RtspStream::push_data(){
    std::lock_guard lock(this->mutex_);
    if (!this->appsrc_) return;
    if (this->latest_image_->data.empty()) return;
    // put image data to buffer and push to pipeline
    GstBuffer *buffer = gst_buffer_new_wrapped_full(
        (GstMemoryFlags)0, (gpointer)&(this->latest_image_->data).at(0),
        this->latest_image_->data.size(),
        0, this->latest_image_->data.size(), nullptr, nullptr);
    GstFlowReturn ret;
    try{
        g_signal_emit_by_name(this->appsrc_, "push-buffer", buffer, &ret);
    }catch(...){
        RCLCPP_INFO(*this->logger_, "Error push data %s", this->name_.c_str());
    }
    gst_buffer_unref(buffer);
    //reset flags to receive new data    
    this->gst_data_request_ = this->ros2_new_data_ = false;
}

void RtspStream::static_push_data(RtspStream *stream){
    stream->push_data();
}

bool RtspStream::is_inactive(){
    auto last_req = std::chrono::system_clock::time_point(
        std::chrono::nanoseconds(this->gst_last_request_ns_.load()));
    bool timeout = std::chrono::system_clock::now() >=
        (last_req + std::chrono::duration<double>(inactivity_timeout_));
    return timeout && this->is_active_;
}

GstPadProbeReturn RtspStream::add_rtp_timestamp_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
    auto* stream = static_cast<RtspStream*>(user_data);
    GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
    if (buffer && (info->type & GST_PAD_PROBE_TYPE_BUFFER)) {
        GstRTPBuffer rtp_buffer = GST_RTP_BUFFER_INIT;
        if (gst_rtp_buffer_map(buffer, GST_MAP_READWRITE, &rtp_buffer)) {
            guint8 ext_id = 1;
            guint8 appbits = 1;
            guint64 custom_timestamp = static_cast<guint64>(stream->gst_last_request_ns_.load());
            if (custom_timestamp) {
                gboolean success = gst_rtp_buffer_add_extension_twobytes_header(
                    &rtp_buffer, appbits, ext_id,
                    &custom_timestamp, sizeof(custom_timestamp));
                if (!success) g_warning("Failed to add RTP header extension.");
            }
            gst_rtp_buffer_unmap(&rtp_buffer);
        }
    }
    return GST_PAD_PROBE_OK;
}


std::string RtspStream::get_gst_encoding(std::string image_encoding){
    // treating special MONO16 encoding case
    if (image_encoding == sensor_msgs::image_encodings::MONO16) {
        RCLCPP_WARN(*this->logger_, "WARNING! image format == Mono16, using UYVY instead");
    }
    std::string format = "MONO8"; // Default value
    auto it = this->color_encoding_map_.find(image_encoding);
    if (it != this->color_encoding_map_.end()) format = it->second;          
    else std::cerr << "UNSUPPORTED IMAGE ENCODING: "<< image_encoding << " | defaulting to MONO8" << std::endl;
    return format;
}
