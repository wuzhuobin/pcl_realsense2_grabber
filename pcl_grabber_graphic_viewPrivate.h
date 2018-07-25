#ifndef __PCL_GRABBER_GRAPHIC_VIEW_PRIVATE_H__
#define __PCL_GRABBER_GRAPHIC_VIEW_PRIVATE_H__
#pragma once
// me 
#include "pcl_grabber_graphic_view.h"
#include "realsense2_grabber.h"
// qt 
#include <QImage>
#include <QPixmap>
// boost
#include <boost/chrono.hpp>
// pcl
#include <pcl/io/image.h>
class pcl_grabber_graphic_viewPrivate 
{
public: 
	Q_DECLARE_PUBLIC(pcl_grabber_graphic_view);
	typedef typename pcl::io::RealSense2Grabber::Image ImageType;
	pcl_grabber_graphic_viewPrivate(pcl_grabber_graphic_view *q);
	pcl_grabber_graphic_view *q_ptr;
	void image_callback(const boost::shared_ptr<ImageType> &image);
	QImage cachedImage;
	QPixmap cachedPixMap;
	boost::shared_mutex image_mutex;
	boost::signals2::connection image_connection;
	pcl::Grabber *grabber;
};

inline pcl_grabber_graphic_viewPrivate::pcl_grabber_graphic_viewPrivate(pcl_grabber_graphic_view *q):
	q_ptr(q),
	grabber(nullptr)
{
}

inline void pcl_grabber_graphic_viewPrivate::image_callback(const boost::shared_ptr<ImageType> &image)
{
	Q_Q(pcl_grabber_graphic_view);
	boost::unique_lock<boost::shared_mutex>(this->image_mutex);
	if (image->getEncoding() == pcl::io::Image::Encoding::RGB) {
		this->cachedImage = QImage(
			static_cast<const unsigned char*>(image->getData()), image->getWidth(), image->getHeight(), QImage::Format_RGB888);
		this->cachedPixMap.convertFromImage(this->cachedImage);
		QMetaObject::invokeMethod(q, "update");
	}
	else {
		return;
	}

}
#endif // !__PCL_GRABBER_GRAPHIC_VIEW_PRIVATE_H__
