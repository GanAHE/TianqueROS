/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include "gazebo/sensors/DepthCameraSensor.hh"
#include "gazebo_gst_camera_plugin.h"

#include <math.h>
#include <string>
#include <iostream>
#include <thread>
#include <time.h>
#include "Int32.pb.h"

#include <opencv2/opencv.hpp>

using namespace std;
using namespace gazebo;
using namespace cv;

GZ_REGISTER_SENSOR_PLUGIN(GstCameraPlugin)


static void cb_need_data(GstElement *appsrc, guint unused_size, gpointer user_data) {
  GstCameraPlugin *plugin = (GstCameraPlugin*)user_data;
  plugin->gstCallback(appsrc);
}

void GstCameraPlugin::gstCallback(GstElement *appsrc) {

  frameBufferMutex.lock();

  while (!frameBuffer) {
	/* can happen if not initialized yet */
    frameBufferMutex.unlock();
    usleep(10000);
    frameBufferMutex.lock();
  }

  GST_BUFFER_PTS(frameBuffer) = gstTimestamp;
  GST_BUFFER_DURATION(frameBuffer) = gst_util_uint64_scale_int (1, GST_SECOND, (int)rate);
  gstTimestamp += GST_BUFFER_DURATION(frameBuffer);

  GstFlowReturn ret;
  g_signal_emit_by_name(appsrc, "push-buffer", frameBuffer, &ret);

  frameBufferMutex.unlock();

  if (ret != GST_FLOW_OK) {
    /* something wrong, stop pushing */
    gzerr << "g_signal_emit_by_name failed" << endl;
    g_main_loop_quit(mainLoop);
  }
}

static void* start_thread(void* param) {
  GstCameraPlugin* plugin = (GstCameraPlugin*)param;
  plugin->startGstThread();
  return nullptr;
}

/////////////////////////////////////////////////
void GstCameraPlugin::startGstThread() {

  gst_init(0, 0);

  mainLoop = g_main_loop_new(NULL, FALSE);
  if (!mainLoop) {
    gzerr << "Create loop failed. \n";
    return;
  }

  GstElement* pipeline = gst_pipeline_new("sender");
  if (!pipeline) {
    gzerr << "ERR: Create pipeline failed. \n";
    return;
  }

  GstElement* dataSrc = gst_element_factory_make("appsrc", "AppSrc");
  GstElement* testSrc = gst_element_factory_make("videotestsrc", "FileSrc");
  GstElement* conv  = gst_element_factory_make("videoconvert", "Convert");
  GstElement* encoder = gst_element_factory_make("x264enc", "AvcEncoder");
  GstElement* parser  = gst_element_factory_make("h264parse", "Parser");
  GstElement* payload = gst_element_factory_make("rtph264pay", "PayLoad");
  GstElement* sink  = gst_element_factory_make("udpsink", "UdpSink");
  if (!dataSrc || !testSrc || !conv || !encoder || !parser || !payload || !sink) {
    gzerr << "ERR: Create elements failed. \n";
    return;
  }

// gzerr <<"width"<< this->width<<"\n";
// gzerr <<"height"<< this->height<<"\n";
// gzerr <<"rate"<< this->rate<<"\n";

  // Config src
  g_object_set(G_OBJECT(dataSrc), "caps",
      gst_caps_new_simple ("video/x-raw",
      "format", G_TYPE_STRING, "I420",
      "width", G_TYPE_INT, this->width,
      "height", G_TYPE_INT, this->height,
      "framerate", GST_TYPE_FRACTION, (unsigned int)this->rate, 1,
      NULL),
      "is-live", TRUE,
      NULL);

  // Config encoder
  g_object_set(G_OBJECT(encoder), "bitrate", 800, NULL);
  g_object_set(G_OBJECT(encoder), "speed-preset", 2, NULL); //lower = faster, 6=medium
  //g_object_set(G_OBJECT(encoder), "tune", "zerolatency", NULL);
  //g_object_set(G_OBJECT(encoder), "low-latency", 1, NULL);
  //g_object_set(G_OBJECT(encoder), "control-rate", 2, NULL);

  // Config payload
  g_object_set(G_OBJECT(payload), "config-interval", 1, NULL);

  // Config udpsink
  g_object_set(G_OBJECT(sink), "host", this->udpHost.c_str(), NULL);
  g_object_set(G_OBJECT(sink), "port", this->udpPort, NULL);
  //g_object_set(G_OBJECT(sink), "sync", false, NULL);
  //g_object_set(G_OBJECT(sink), "async", false, NULL);

  // Connect all elements to pipeline
  gst_bin_add_many(GST_BIN(pipeline), dataSrc, conv, encoder, parser, payload, sink, NULL);

  // Link all elements
  if (gst_element_link_many(dataSrc, conv, encoder, parser, payload, sink, NULL) != TRUE) {
    gzerr << "ERR: Link all the elements failed. \n";
    return;
  }

  // Set up appsrc
  g_object_set(G_OBJECT(dataSrc), "stream-type", 0, "format", GST_FORMAT_TIME, NULL);
  g_signal_connect(dataSrc, "need-data", G_CALLBACK(cb_need_data), this);

  // Start
  gst_element_set_state(pipeline, GST_STATE_PLAYING);
  g_main_loop_run(mainLoop);

  // Clean up
  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(GST_OBJECT(pipeline));
  g_main_loop_unref(mainLoop);
  mainLoop = nullptr;

}

/////////////////////////////////////////////////
void GstCameraPlugin::stopGstThread()
{
  if(mainLoop)
    g_main_loop_quit(mainLoop);

}

/////////////////////////////////////////////////
GstCameraPlugin::GstCameraPlugin()
: SensorPlugin(), width(0), height(0), depth(0), frameBuffer(nullptr), mainLoop(nullptr),
  gstTimestamp(0), mIsActive(false)
{
}

/////////////////////////////////////////////////
GstCameraPlugin::~GstCameraPlugin()
{
  this->parentSensor.reset();
  this->camera.reset();
  if (mainLoop) {
    g_main_loop_quit(mainLoop);
  }
  std::lock_guard<std::mutex> guard(frameBufferMutex);
  if (frameBuffer) {
	  gst_buffer_unref(frameBuffer);
	  frameBuffer = nullptr;
  }
}

/////////////////////////////////////////////////
void GstCameraPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
  if (!sensor)
    gzerr << "Invalid sensor pointer.\n";

  this->parentSensor =
#if GAZEBO_MAJOR_VERSION >= 7
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
#else
    boost::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
#endif

  if (!this->parentSensor)
  {
    gzerr << "GstCameraPlugin requires a CameraSensor.\n";
#if GAZEBO_MAJOR_VERSION >= 7
    if (std::dynamic_pointer_cast<sensors::DepthCameraSensor>(sensor))
#else
    if (boost::dynamic_pointer_cast<sensors::DepthCameraSensor>(sensor))
#endif
      gzmsg << "It is a depth camera sensor\n";
  }

#if GAZEBO_MAJOR_VERSION >= 7
  this->camera = this->parentSensor->Camera();
#else
  this->camera = this->parentSensor->GetCamera();
#endif

  if (!this->parentSensor)
  {
    gzerr << "GstCameraPlugin not attached to a camera sensor\n";
    return;
  }

#if GAZEBO_MAJOR_VERSION >= 7
  this->width = this->camera->ImageWidth();
  this->height = this->camera->ImageHeight();
  this->depth = this->camera->ImageDepth();
  this->format = this->camera->ImageFormat();
  this->rate = this->camera->RenderRate();
#else
  this->width = this->camera->GetImageWidth();
  this->height = this->camera->GetImageHeight();
  this->depth = this->camera->GetImageDepth();
  this->format = this->camera->GetImageFormat();
  this->rate = this->camera->GetRenderRate();
#endif

 if (!isfinite(this->rate)) {
   this->rate =  60.0;
 }

  if (sdf->HasElement("robotNamespace"))
    namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzwarn << "[gazebo_gst_camera_plugin] Please specify a robotNamespace.\n";

  this->udpHost = "127.0.0.1";
  if (sdf->HasElement("udpHost")) {
	this->udpHost = sdf->GetElement("udpHost")->Get<string>();
  }
	
  this->udpPort = 5600;
  if (sdf->HasElement("udpPort")) {
	this->udpPort = sdf->GetElement("udpPort")->Get<int>();
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  // Listen to Gazebo topic
  mVideoSub = node_handle_->Subscribe<msgs::Int>(mTopicName, &GstCameraPlugin::cbVideoStream, this);

  // And start by default
  startStreaming();

}

/////////////////////////////////////////////////
void GstCameraPlugin::cbVideoStream(const boost::shared_ptr<const msgs::Int> &_msg)
{
  gzwarn << "Video Streaming callback: " << _msg->data() << "\n";
  int enable = _msg->data();
  if(enable)
    startStreaming();
  else
    stopStreaming();
}

/////////////////////////////////////////////////
void GstCameraPlugin::startStreaming()
{
  if(!mIsActive) {
    this->newFrameConnection = this->camera->ConnectNewImageFrame(
        boost::bind(&GstCameraPlugin::OnNewFrame, this, _1, this->width, this->height, this->depth, this->format));

    this->parentSensor->SetActive(true);

    /* start the gstreamer event loop */
    pthread_create(&mThreadId, NULL, start_thread, this);
    mIsActive = true;
  }

}

/////////////////////////////////////////////////
void GstCameraPlugin::stopStreaming()
{
  if(mIsActive) {
    stopGstThread();

    pthread_join(mThreadId, NULL);

    this->parentSensor->SetActive(false);

    this->newFrameConnection->~Connection();
    mIsActive = false;
  }

}

/////////////////////////////////////////////////
void GstCameraPlugin::OnNewFrame(const unsigned char * image,
                              unsigned int width,
                              unsigned int height,
                              unsigned int depth,
                              const std::string &format)
{

#if GAZEBO_MAJOR_VERSION >= 7
  image = this->camera->ImageData(0);
#else
  image = this->camera->GetImageData(0);
#endif

  std::lock_guard<std::mutex> guard(frameBufferMutex);

  if (frameBuffer) {
    gst_buffer_unref(frameBuffer);
  }

  // Alloc buffer
  guint size = width * height * 1.5;
  frameBuffer = gst_buffer_new_allocate(NULL, size, NULL);

  GstMapInfo mapInfo;
  if (gst_buffer_map(frameBuffer, &mapInfo, GST_MAP_WRITE)) {

    // Color Conversion from RGB to YUV
    Mat frame = Mat(height, width, CV_8UC3);
    Mat frameYUV = Mat(height, width, CV_8UC3);
    frame.data = (uchar*)image;
    cvtColor(frame, frameYUV, COLOR_RGB2YUV_I420);

    memcpy(mapInfo.data, frameYUV.data, size);
    gst_buffer_unmap(frameBuffer, &mapInfo);
  } else {
	  gzerr << "gst_buffer_map failed"<<endl;
  }
}
