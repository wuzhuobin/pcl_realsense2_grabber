// me
#include "realsense2_grabber.h"
#include "realsense2_frame_wrapper.h"
#include "realsense2_preset.h"
// boost
#include <boost/thread.hpp>
// realsense2
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
// pcl
#include <pcl/io/image_depth.h>
#include <pcl/io/image_ir.h>
#include <pcl/io/image.h>
#include <pcl/io/image_rgb24.h>
pcl::io::RealSense2Grabber::RealSense2Grabber(const std::string & device_id, const Mode& mode):
	rs2_pipeline(new rs2::pipeline),
	rs2_config(new rs2::config),
	running(false)
{
	this->image_signal = this->createSignal<sig_cb_realsense2_image>();
	this->ir_image_signal = this->createSignal<sig_cb_realsense2_ir_image>();
	this->depth_image_signal = this->createSignal<sig_cb_realsense2_depth_image>();
	this->point_cloud_signal = this->createSignal<sig_cb_realsense2_point_cloud>();
	this->point_cloud_rgb_signal = this->createSignal<sig_cb_realsense2_point_cloud_rgb>();
	rs2_config->disable_all_streams();
	switch (mode.frame_rate)
	{
	case Mode::rs2_rs400_frame_rate::FPS6Hz:
		this->frame_rate = 6;
		break;
	case Mode::rs2_rs400_frame_rate::FPS15Hz:
		this->frame_rate = 15;
		break;
	case Mode::rs2_rs400_frame_rate::FPS30Hz:
		this->frame_rate = 30;
	default:
		break;
	}
	switch (mode.resolution)
	{
	case Mode::rs2_rs400_resolution::H1280X720:
		this->image_width_ = 1280;
		this->image_height_ = 720;
		break;
	case Mode::rs2_rs400_resolution::H848X480:
		this->image_width_ = 848;
		this->image_height_ = 480;
		break;
	case Mode::rs2_rs400_resolution::M640X480:
		this->image_width_ = 640;
		this->image_height_ = 480;
		break;
	case Mode::rs2_rs400_resolution::M640X360:
		this->image_width_ = 640;
		this->image_height_ = 360;
		break;
	case Mode::rs2_rs400_resolution::L480X270:
		this->image_width_ = 480;
		this->image_height_ = 270;
		break;
	case Mode::rs2_rs400_resolution::L424X240:
		this->image_width_ = 424;
		this->image_height_ = 240;
		break;
	default:
		break;
	}
	switch (mode.preset)
	{
	case rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_DENSITY:
		switch (mode.resolution)
		{
		case Mode::rs2_rs400_resolution::L480X270:
		case Mode::rs2_rs400_resolution::L424X240:
			this->preset_json = realsense2::LOW_RES_HIGH_DENSITY_PRESET;
			break;
		case Mode::rs2_rs400_resolution::M640X480:
		case Mode::rs2_rs400_resolution::M640X360:
			this->preset_json = realsense2::MED_RES_HIGH_DENSITY_PRESET;
			break;
		case Mode::rs2_rs400_resolution::H1280X720:
		case Mode::rs2_rs400_resolution::H848X480:
			this->preset_json = realsense2::HIGH_RES_HIGH_DENSITY_PRESET;
			break;
		}
		break;
	case rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_MEDIUM_DENSITY:
		switch (mode.resolution)
		{
		case Mode::rs2_rs400_resolution::L480X270:
		case Mode::rs2_rs400_resolution::L424X240:
			this->preset_json = realsense2::LOW_RES_MED_DENSITY_PRESET;
			break;
		case Mode::rs2_rs400_resolution::M640X480:
		case Mode::rs2_rs400_resolution::M640X360:
			this->preset_json = realsense2::MED_RES_MED_DENSITY_PRESET;
			break;
		case Mode::rs2_rs400_resolution::H1280X720:
		case Mode::rs2_rs400_resolution::H848X480:
			this->preset_json = realsense2::HIGH_RES_MED_DENSITY_PRESET;
			break;
		}
		break;
	case rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY:
		switch (mode.resolution)
		{
		case Mode::rs2_rs400_resolution::L480X270:
		case Mode::rs2_rs400_resolution::L424X240:
			this->preset_json = realsense2::LOW_RES_HIGH_ACCURACY_PRESET;
			break;
		case Mode::rs2_rs400_resolution::M640X480:
		case Mode::rs2_rs400_resolution::M640X360:
			this->preset_json = realsense2::MED_RES_HIGH_ACCURACY_PRESET;
			break;
		case Mode::rs2_rs400_resolution::H1280X720:
		case Mode::rs2_rs400_resolution::H848X480:
			this->preset_json = realsense2::HIGH_RES_HIGH_ACCURACY_PRESET;
			break;
		}
		break;
	case rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_DEFAULT:
	default:
		this->preset_json = realsense2::DEFAULT_PRESET_D435;
		break;
	}
}

pcl::io::RealSense2Grabber::~RealSense2Grabber()
{
	this->stop();
	this->disconnect_all_slots<sig_cb_realsense2_image>();
	this->disconnect_all_slots<sig_cb_realsense2_ir_image>();
	this->disconnect_all_slots<sig_cb_realsense2_depth_image>();
	this->disconnect_all_slots<sig_cb_realsense2_point_cloud>();
	this->disconnect_all_slots<sig_cb_realsense2_point_cloud_rgb>();
}

void pcl::io::RealSense2Grabber::start()
{
	this->running = true;
	this->block_signals();
	this->rs2_pipeline->start(*this->rs2_config);
	rs2::pipeline_profile profile = this->rs2_pipeline->get_active_profile();
	rs400::advanced_mode device = profile.get_device().as<rs400::advanced_mode>();
	//device.get
	if (!device.is_enabled()) {
		std::cerr << "Not enabled. \n";
		device.toggle_advanced_mode(true);
	}
	device.load_json(this->preset_json);
	std::cerr << device.serialize_json() << '\n';

	this->process_thread.reset(new boost::thread{
		boost::bind(&RealSense2Grabber::process, this)
	});
	this->unblock_signals();
}

void pcl::io::RealSense2Grabber::stop()
{
	this->running = false;
	this->process_thread->join();
	try {
		//this->rs2_pipeline-
		this->rs2_pipeline->stop();
	}
	catch (rs2::wrong_api_call_sequence_error &e) {
		std::cerr << "Error catched. wrong_api_call_sequence_error. \n";
		std::cerr << e.what() << '\n';
	}
}

void pcl::io::RealSense2Grabber::checkImageStreamRequired()
{
	if (this->num_slots<sig_cb_realsense2_image>() > 0 ||
		this->num_slots<sig_cb_realsense2_point_cloud_rgb>() > 0) {
		//this->rs2_config->enable_stream(RS2_STREAM_COLOR, this->image_width_, this->image_height_, this->frame_rate);
	}
	else {
		this->rs2_config->disable_stream(RS2_STREAM_COLOR);
	}
}

void pcl::io::RealSense2Grabber::checkDepthStreamRequired()
{
	if (this->num_slots<sig_cb_realsense2_depth_image>() > 0 ||
		this->num_slots<sig_cb_realsense2_point_cloud>() > 0 ||
		this->num_slots<sig_cb_realsense2_point_cloud_rgb>() > 0) {
		//this->rs2_config->enable_stream(RS2_STREAM_DEPTH, this->image_width_, this->image_height_, this->frame_rate);
	}
	else {
		this->rs2_config->disable_stream(RS2_STREAM_DEPTH);
	}
}

void pcl::io::RealSense2Grabber::checkIRStreamRequired()
{
	if (this->num_slots<sig_cb_realsense2_ir_image>() > 0 ||
		this->num_slots<sig_cb_realsense2_point_cloud>() > 0 ||
		this->num_slots<sig_cb_realsense2_point_cloud_rgb>() > 0) {
		//this->rs2_config->enable_stream(RS2_STREAM_INFRARED, 0, this->image_width_, this->image_height_, RS2_FORMAT_ANY, this->frame_rate);
		//this->rs2_config->enable_stream(RS2_STREAM_INFRARED, 1, this->image_width_, this->image_height_, RS2_FORMAT_ANY, this->frame_rate);
	}
	else {
		this->rs2_config->disable_stream(RS2_STREAM_INFRARED, 0);
		this->rs2_config->disable_stream(RS2_STREAM_INFRARED, 1);
	}
}

void pcl::io::RealSense2Grabber::imageCallback(rs2::frameset & frames)
{
	if (this->image_signal->num_slots() > 0) {
		Image::Timestamp t_callback = Image::Clock::now();
		rs2::video_frame frame = frames.get_color_frame();
		FrameWrapper::Ptr frameWrapper = boost::make_shared<pcl::io::realsense2::RealSense2FrameWrapper>(frame);
		boost::shared_ptr<Image> image = boost::make_shared<ImageRGB24>(frameWrapper, t_callback);
		(*this->image_signal)(image);
	}
}

void pcl::io::RealSense2Grabber::depthCallback(rs2::frameset & frames)
{
	if (this->depth_image_signal->num_slots() > 0) {
		Image::Timestamp t_callback = Image::Clock::now();
		rs2::depth_frame depth = frames.get_depth_frame();
		//auto depth_stream = this->rs2_pipeline->get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
		FrameWrapper::Ptr framWrapper = boost::make_shared<pcl::io::realsense2::RealSense2FrameWrapper>(depth);
		boost::shared_ptr<DepthImage> image = boost::make_shared<DepthImage>(framWrapper, 0, 0, 0, 0);
		(*this->depth_image_signal)(image);
	}
	if (this->point_cloud_signal->num_slots() > 0) {
		rs2::depth_frame depth = frames.get_depth_frame();
		PointCloud<PointXYZ>::Ptr cloud = this->convertToXYZPointCloud(depth);
		(*this->point_cloud_signal)(cloud);
	}
}

void pcl::io::RealSense2Grabber::irCallback(rs2::frameset & frames)
{
	if (this->ir_image_signal->num_slots() > 0) {
		Image::Timestamp t_callback = Image::Clock::now();
		rs2::video_frame ir = frames.get_infrared_frame();
		FrameWrapper::Ptr frameWrapper = boost::make_shared<pcl::io::realsense2::RealSense2FrameWrapper>(ir);
		boost::shared_ptr<IRImage> ir_image = boost::make_shared<IRImage>(frameWrapper, t_callback);
		(*this->ir_image_signal)(ir_image);
	}
}

void pcl::io::RealSense2Grabber::signalsChanged()
{
	// reevaluate which streams are required
	this->checkImageStreamRequired();
	this->checkDepthStreamRequired();
	this->checkIRStreamRequired();
	//if (ir_required_ && image_required_)
	//	PCL_THROW_EXCEPTION(pcl::IOException, "Can not provide IR stream and RGB stream at the same time.");

	//checkImageAndDepthSynchronizationRequired();
	if (this->running)
	{
		this->stop();
		this->start();
	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr pcl::io::RealSense2Grabber::convertToXYZPointCloud(const rs2::depth_frame &frame)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	rs2::pointcloud rs2_point_cloud;
	//rs2_point_cloud.calculate(depth);
	rs2::points points = rs2_point_cloud.calculate(frame);
	auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }
	//std::cerr << __FUNCTION__ << '\n';
    return cloud;
}

void pcl::io::RealSense2Grabber::process()
{
	while (this->running)
	{
		rs2::frameset frames = this->rs2_pipeline->wait_for_frames();
		//if (this->image_required_) {
		//}
		//if (this->ir_required_) {
		//}
		//if (this->depth_required_) {
		//}
		this->imageCallback(frames);
		this->irCallback(frames);
		this->depthCallback(frames);

	}
}
