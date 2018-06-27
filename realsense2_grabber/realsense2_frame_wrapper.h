#ifndef __PCL_IO_REALSENSE2_FRAME_WRAPPER_H__
#define __PCL_IO_REALSENSE2_FRAME_WRAPPER_H__
#pragma once
#include <pcl/io/image_metadata_wrapper.h>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/h/rs_frame.h>
#include <librealsense2/h/rs_types.h>
namespace pcl
{
	namespace io
	{
		namespace realsense2
		{
			class RealSense2FrameWrapper : public pcl::io::FrameWrapper
			{
			public:
				RealSense2FrameWrapper(rs2::video_frame metadata)
					: metadata_(metadata)
				{}

				virtual inline const void*
					getData() const
				{
					return (metadata_.get_data());
				}

				// not sure
				virtual inline unsigned
					getDataSize() const
				{
					return metadata_.get_stride_in_bytes() * metadata_.get_height();
				}

				virtual inline unsigned
					getWidth() const
				{
					return (metadata_.get_width());
				}

				virtual inline unsigned
					getHeight() const
				{
					return (metadata_.get_height());
				}

				virtual inline unsigned
					getFrameID() const
				{
					return (metadata_.get_frame_number());
				}

				virtual inline uint64_t
					getTimestamp() const
				{
					return (metadata_.get_timestamp());
				}


				const inline rs2::video_frame&
					getMetaData() const
				{
					return (metadata_);
				}

			private:
				rs2::video_frame metadata_; // Internally reference counted
			};

		} // namespace
	}
}
#endif // __PCL_IO_REALSENSE2_FRAME_WRAPPER_H__
