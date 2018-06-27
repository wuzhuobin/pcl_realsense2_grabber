#ifndef __PCL_IO_REALSENSE2_GRABBER_H__
#define __PCL_IO_REALSENSE2_GRABBER_H__
#pragma once
// pcl
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace pcl {
	namespace io {
		class DepthImage;
		class IRImage;
		class Image;
	}
}
// boost
#include <boost/shared_ptr.hpp>
#include <boost/signals2.hpp>
namespace boost {
	class thread;
}
// realsense2
#include <librealsense2/h/rs_option.h>
namespace rs2 { 
	class pipeline; 
	class config;
	class frameset;
	class depth_frame;
}
namespace pcl {
	namespace io {
		class RealSense2Grabber : public Grabber {
		public:
			typedef pcl::io::DepthImage DepthImage;
			typedef pcl::io::IRImage IRImage;
			typedef pcl::io::Image Image;
			struct Mode
			{
				typedef enum rs2_rs400_resolution {
					H1280X720 = 0,
					H848X480,
					M640X480,
					M640X360,
					L480X270,
					L424X240
				}rs2_rs400_resolution;
				rs2_rs400_resolution resolution;
				typedef enum rs2_rs400_frame_rate {
					FPS30Hz = 0,
					FPS15Hz,
					FPS6Hz
				}rs2_rs400_frame_rate;
				rs2_rs400_frame_rate frame_rate;
				typedef rs2_rs400_visual_preset rs2_rs400_visual_preset;
				rs2_rs400_visual_preset preset;
				Mode(rs2_rs400_resolution resolution = H1280X720,
					rs2_rs400_frame_rate frame_rate = FPS30Hz,
					rs2_rs400_visual_preset preset = RS2_RS400_VISUAL_PRESET_DEFAULT) :
					resolution(resolution),
					frame_rate(frame_rate),
					preset(preset) {}
			};
			typedef boost::shared_ptr<RealSense2Grabber> Ptr;
			typedef boost::shared_ptr<const RealSense2Grabber> ConstPtr;
			typedef void (sig_cb_realsense2_image)(const boost::shared_ptr<Image>&);
			typedef void (sig_cb_realsense2_ir_image)(const boost::shared_ptr<IRImage>&);
			typedef void (sig_cb_realsense2_depth_image)(const boost::shared_ptr<DepthImage>&);
			typedef void (sig_cb_realsense2_point_cloud)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&);
			typedef void (sig_cb_realsense2_point_cloud_rgb)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> >&);

			RealSense2Grabber(const std::string &device_id = "", const Mode& mode = Mode());
			virtual ~RealSense2Grabber() override;
			virtual void start() override;
			virtual void stop() override;
			virtual std::string getName() const override { return "RealSense2Grabber"; }
			virtual bool isRunning() const override { return this->running; }
			virtual float getFramesPerSecond() const override { return this->frame_rate; }
			boost::shared_ptr<rs2::pipeline> getPipeline() const { return this->rs2_pipeline; }
		protected:
			/** \brief Check if the RGB image stream is required or not. */
			virtual void checkImageStreamRequired();
			/** \brief Check if the depth stream is required or not. */
			virtual void checkDepthStreamRequired();
			/** \brief Check if the IR image stream is required or not. */
			virtual void checkIRStreamRequired();
			virtual void imageCallback(rs2::frameset &frames);
			virtual void depthCallback(rs2::frameset &frames);
			virtual void irCallback(rs2::frameset &frames);
			virtual void signalsChanged() override;
			static pcl::PointCloud<pcl::PointXYZ>::Ptr convertToXYZPointCloud(const rs2::depth_frame &frame);
			virtual void process();
		private:
			boost::shared_ptr<rs2::pipeline> rs2_pipeline;
			boost::shared_ptr<rs2::config> rs2_config;
			//Mode rs2_mode;
			std::string rgb_frame_id_;
			std::string depth_frame_id_;
			unsigned image_width_;
			unsigned image_height_;
			float frame_rate;
			std::string preset_json;

			//bool image_required_;
			//bool depth_required_;
			//bool ir_required_;
			bool sync_required_;
			boost::signals2::signal<sig_cb_realsense2_image> *image_signal;
			boost::signals2::signal<sig_cb_realsense2_ir_image> *ir_image_signal;
			boost::signals2::signal<sig_cb_realsense2_depth_image> *depth_image_signal;
			boost::signals2::signal<sig_cb_realsense2_point_cloud> *point_cloud_signal;
			boost::signals2::signal<sig_cb_realsense2_point_cloud_rgb> *point_cloud_rgb_signal;
			boost::shared_ptr<boost::thread> process_thread;
			bool running;
		};
	}
}
#endif // !__PCL__IO_REALSENSE2_GRABBER_H__
