/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* $Id$
*
*/

#define MEASURE_FUNCTION_TIME
#include <pcl/common/time.h> //fps calculations
#include <pcl/common/angles.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/image_depth.h>
#include <pcl/io/image_ir.h>
#include <pcl/io/image.h>
#include <pcl/io/image_rgb24.h>
#include <boost/chrono.hpp>

#include "realsense2_grabber.h"
typedef boost::chrono::high_resolution_clock HRClock;

#define SHOW_FPS 1
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
  do \
{ \
  static unsigned count = 0;\
  static double last = pcl::getTime ();\
  double now = pcl::getTime (); \
  ++count; \
  if (now - last >= 1.0) \
{ \
  std::cout << "Average framerate ("<< _WHAT_ << "): " << double (count)/double (now - last) << " Hz" <<  std::endl; \
  count = 0; \
  last = now; \
} \
}while (false)
#else
#define FPS_CALC (_WHAT_) \
  do \
{ \
}while (false)
#endif

void
printHelp(int, char **argv)
{
	using pcl::console::print_error;
	using pcl::console::print_info;

	print_error("Syntax is: %s [((<device_id> | <path-to-oni-file>) [-depthmode <mode>] [-imagemode <mode>] [-xyz] | -l [<device_id>]| -h | --help)]\n", argv[0]);
	print_info("%s -h | --help : shows this help\n", argv[0]);
	print_info("%s -xyz : use only XYZ values and ignore RGB components (this flag is required for use with ASUS Xtion Pro) \n", argv[0]);
	print_info("%s -l : list all available devices\n", argv[0]);
	print_info("%s -l <device-id> :list all available modes for specified device\n", argv[0]);
	print_info("\t\t<device_id> may be \"#1\", \"#2\", ... for the first, second etc device in the list\n");
#ifndef _WIN32
	print_info("\t\t                   bus@address for the device connected to a specific usb-bus / address combination\n");
	print_info("\t\t                   <serial-number>\n");
#endif
	print_info("\n\nexamples:\n");
	print_info("%s \"#1\"\n", argv[0]);
	print_info("\t\t uses the first device.\n");
	print_info("%s  \"./temp/test.oni\"\n", argv[0]);
	print_info("\t\t uses the oni-player device to play back oni file given by path.\n");
	print_info("%s -l\n", argv[0]);
	print_info("\t\t list all available devices.\n");
	print_info("%s -l \"#2\"\n", argv[0]);
	print_info("\t\t list all available modes for the second device.\n");
#ifndef _WIN32
	print_info("%s A00361800903049A\n", argv[0]);
	print_info("\t\t uses the device with the serial number \'A00361800903049A\'.\n");
	print_info("%s 1@16\n", argv[0]);
	print_info("\t\t uses the device on address 16 at USB bus 1.\n");
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointType>
class RealSense2Viewer
{
public:
	typedef pcl::PointCloud<PointType> Cloud;
	typedef typename Cloud::ConstPtr CloudConstPtr;

	RealSense2Viewer(pcl::io::RealSense2Grabber& grabber)
		: cloud_viewer_(new pcl::visualization::PCLVisualizer("PCL RealSense2 cloud"))
		, image_viewer_()
		, ir_image_viewer_()
		, depth_image_viewer_()
		, grabber_(grabber)
		, rgb_data_(0), rgb_data_size_(0)
	{
	}

	void
		cloud_callback(const CloudConstPtr& cloud)
	{
		FPS_CALC("cloud callback");
		boost::mutex::scoped_lock lock(cloud_mutex_);
		cloud_ = cloud;
	}

	void
		image_callback(const boost::shared_ptr<pcl::io::RealSense2Grabber::Image>& image)
	{
		FPS_CALC("image callback");
		boost::mutex::scoped_lock lock(image_mutex_);
		image_ = image;

		if (image->getEncoding() != pcl::io::RealSense2Grabber::Image::RGB)
		{
			if (rgb_data_size_ < image->getWidth() * image->getHeight())
			{
				if (rgb_data_)
					delete[] rgb_data_;
				rgb_data_size_ = image->getWidth() * image->getHeight();
				rgb_data_ = new unsigned char[rgb_data_size_ * 3];
			}
			image_->fillRGB(image_->getWidth(), image_->getHeight(), rgb_data_);
		}
	}

	void
		ir_image_callback(const boost::shared_ptr<pcl::io::RealSense2Grabber::IRImage>& image)
	{
		FPS_CALC("image callback");
		boost::mutex::scoped_lock lock(ir_image_mutex_);
		ir_image_ = image;
	}
	void
		depth_image_callback(const boost::shared_ptr<pcl::io::RealSense2Grabber::DepthImage>& image)
	{
		FPS_CALC("image callback");
		boost::mutex::scoped_lock lock(depth_image_mutex_);
		depth_image_ = image;
	}
	void
		keyboard_callback(const pcl::visualization::KeyboardEvent& event, void*)
	{
		if (event.getKeyCode())
			cout << "the key \'" << event.getKeyCode() << "\' (" << event.getKeyCode() << ") was";
		else
			cout << "the special key \'" << event.getKeySym() << "\' was";
		if (event.keyDown())
			cout << " pressed" << endl;
		else
			cout << " released" << endl;
	}

	void
		mouse_callback(const pcl::visualization::MouseEvent& mouse_event, void*)
	{
		if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton)
		{
			cout << "left button pressed @ " << mouse_event.getX() << " , " << mouse_event.getY() << endl;
		}
	}

	/**
	* @brief starts the main loop
	*/
	void
		run()
	{
		cloud_viewer_->registerMouseCallback(&RealSense2Viewer::mouse_callback, *this);
		cloud_viewer_->registerKeyboardCallback(&RealSense2Viewer::keyboard_callback, *this);
		cloud_viewer_->setCameraFieldOfView(1.02259994f);
		boost::function<void(const CloudConstPtr&) > cloud_cb = boost::bind(&RealSense2Viewer::cloud_callback, this, _1);
		boost::signals2::connection cloud_connection = grabber_.registerCallback(cloud_cb);

		boost::signals2::connection image_connection;
		if (grabber_.providesCallback<void(const boost::shared_ptr< pcl::io::RealSense2Grabber::Image>&)>())
		{
			image_viewer_.reset(new pcl::visualization::ImageViewer("PCL OpenNI image"));
			image_viewer_->registerMouseCallback(&RealSense2Viewer::mouse_callback, *this);
			image_viewer_->registerKeyboardCallback(&RealSense2Viewer::keyboard_callback, *this);
			boost::function<void(const boost::shared_ptr<pcl::io::RealSense2Grabber::Image>&) > image_cb = boost::bind(&RealSense2Viewer::image_callback, this, _1);
			image_connection = grabber_.registerCallback(image_cb);
		}

		boost::signals2::connection ir_image_connection;
		if (grabber_.providesCallback<void(const boost::shared_ptr<pcl::io::RealSense2Grabber::IRImage>&)>())
		{
			ir_image_viewer_.reset(new pcl::visualization::ImageViewer("PCL OpenNI ir image"));
			ir_image_viewer_->registerMouseCallback(&RealSense2Viewer::mouse_callback, *this);
			ir_image_viewer_->registerKeyboardCallback(&RealSense2Viewer::keyboard_callback, *this);
			boost::function<void(const boost::shared_ptr<pcl::io::RealSense2Grabber::IRImage>&) > image_cb = boost::bind(&RealSense2Viewer::ir_image_callback, this, _1);
			// pcl ir image is 16bit unsigned short while realsense can only support 8 bit
			//ir_image_connection = grabber_.registerCallback(image_cb);
		}
		boost::signals2::connection depth_image_connection;
		if (grabber_.providesCallback<void(const boost::shared_ptr<pcl::io::RealSense2Grabber::DepthImage>&)>())
		{
			depth_image_viewer_.reset(new pcl::visualization::ImageViewer("PCL OpenNI ir image"));
			depth_image_viewer_->registerMouseCallback(&RealSense2Viewer::mouse_callback, *this);
			depth_image_viewer_->registerKeyboardCallback(&RealSense2Viewer::keyboard_callback, *this);
			boost::function<void(const boost::shared_ptr<pcl::io::RealSense2Grabber::DepthImage>&) > image_cb = boost::bind(&RealSense2Viewer::depth_image_callback, this, _1);
			depth_image_connection = grabber_.registerCallback(image_cb);
		}
		bool image_init = false, cloud_init = false, ir_image_init = false, depth_image_init = false;

		grabber_.start();

		while (true)
		{
			boost::shared_ptr<pcl::io::RealSense2Grabber::Image> image;
			boost::shared_ptr<pcl::io::RealSense2Grabber::IRImage> ir_image;
			boost::shared_ptr<pcl::io::RealSense2Grabber::DepthImage> depth_image;
			CloudConstPtr cloud;

			cloud_viewer_->spinOnce();

			// See if we can get a cloud
			if (cloud_mutex_.try_lock())
			{
				cloud_.swap(cloud);
				cloud_mutex_.unlock();
			}

			if (cloud)
			{
				FPS_CALC("drawing cloud");

				if (!cloud_init)
				{
					cloud_viewer_->setPosition(0, 0);
					cloud_viewer_->setSize(cloud->width, cloud->height);
					cloud_init = !cloud_init;
				}

				if (!cloud_viewer_->updatePointCloud(cloud, "OpenNICloud"))
				{
					cloud_viewer_->addPointCloud(cloud, "OpenNICloud");
					cloud_viewer_->resetCameraViewpoint("OpenNICloud");
					cloud_viewer_->setCameraPosition(
						0, 0, 0,		// Position
						0, 0, 1,		// Viewpoint
						0, -1, 0);	// Up
				}
			}
			if (cloud_init && this->cloud_viewer_->wasStopped()) {
				break;
			}
			// See if we can get an image
			if (image_mutex_.try_lock())
			{
				image_.swap(image);
				image_mutex_.unlock();
			}


			if (image)
			{
				if (!image_init && cloud && cloud->width != 0)
				{
					image_viewer_->setPosition(cloud->width, 0);
					image_viewer_->setSize(cloud->width, cloud->height);
					image_init = !image_init;
				}

				if (image->getEncoding() == pcl::io::RealSense2Grabber::Image::RGB)
					image_viewer_->addRGBImage((const unsigned char*)image->getData(), image->getWidth(), image->getHeight());
				else
					image_viewer_->addRGBImage(rgb_data_, image->getWidth(), image->getHeight());
				image_viewer_->spinOnce();

			}

			if (image_init && this->image_viewer_->wasStopped()) {
				break;
			}
			// See if we can get an ir_image
			if (ir_image_mutex_.try_lock())
			{
				ir_image_.swap(ir_image);
				ir_image_mutex_.unlock();
			}


			if (ir_image)
			{
				if (/*!ir_image_init && */cloud && cloud->width != 0)
				{
					ir_image_viewer_->setPosition(cloud->width, 0);
					ir_image_viewer_->setSize(cloud->width, cloud->height);
					ir_image_init = !ir_image_init;
					ir_image_viewer_->addShortImage(
						ir_image->getData(),
						ir_image->getWidth(),
						ir_image->getHeight(),
						std::numeric_limits<unsigned char>::min(),
						std::numeric_limits<unsigned char>::max(),
						true);
				}
				ir_image_viewer_->spinOnce();
			}
			if (ir_image_init && this->ir_image_viewer_->wasStopped()) {
				break;
			}
			// See if we can get an depth_image
			if (depth_image_mutex_.try_lock())
			{
				depth_image_.swap(depth_image);
				depth_image_mutex_.unlock();
			}


			if (depth_image)
			{
				if (/*!depth_image_init && */cloud && cloud->width != 0)
				{
					depth_image_viewer_->setPosition(cloud->width, 0);
					depth_image_viewer_->setSize(cloud->width, cloud->height);
					depth_image_init = !depth_image_init;
					depth_image_viewer_->addShortImage(
						depth_image->getData(),
						depth_image->getWidth(),
						depth_image->getHeight(),
						//std::numeric_limits<unsigned char>::min(), 
						//std::numeric_limits<unsigned char>::max(), 
						true);
				}
				depth_image_viewer_->spinOnce();
			}
			if (depth_image_init && this->depth_image_viewer_->wasStopped()) {
				break;
			}
		}

		grabber_.stop();

		cloud_connection.disconnect();
		image_connection.disconnect();
		ir_image_connection.disconnect();
		depth_image_connection.disconnect();
		if (rgb_data_)
			delete[] rgb_data_;
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;
	boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer_;
	boost::shared_ptr<pcl::visualization::ImageViewer> ir_image_viewer_;
	boost::shared_ptr<pcl::visualization::ImageViewer> depth_image_viewer_;

	pcl::io::RealSense2Grabber& grabber_;
	boost::mutex cloud_mutex_;
	boost::mutex image_mutex_;
	boost::mutex ir_image_mutex_;
	boost::mutex depth_image_mutex_;

	CloudConstPtr cloud_;
	boost::shared_ptr<pcl::io::RealSense2Grabber::Image> image_;
	boost::shared_ptr<pcl::io::RealSense2Grabber::IRImage> ir_image_;
	boost::shared_ptr<pcl::io::RealSense2Grabber::DepthImage> depth_image_;
	unsigned char* rgb_data_;
	unsigned rgb_data_size_;
};
//
//// Create the PCLVisualizer object
//boost::shared_ptr<pcl::visualization::PCLVisualizer> cld;
//boost::shared_ptr<pcl::visualization::ImageViewer> img;

/* ---[ */
int
main(int argc, char** argv)
{
	pcl::io::RealSense2Grabber grabber("", pcl::io::RealSense2Grabber::Mode(
		pcl::io::RealSense2Grabber::Mode::rs2_rs400_resolution::H1280X720,
		pcl::io::RealSense2Grabber::Mode::rs2_rs400_frame_rate::FPS30Hz,
		pcl::io::RealSense2Grabber::Mode::rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY
	));
		RealSense2Viewer<pcl::PointXYZ> openni_viewer(grabber);
		openni_viewer.run();
	return (0);
}
/* ]--- */
