#ifndef __PCL_GRABBER_GRAPHIC_VIEW_H__
#define __PCL_GRABBER_GRAPHIC_VIEW_H__
#pragma once
// qt
#include <QGraphicsView>
// pcl
#include <pcl/io/image.h>
namespace pcl {
	class Grabber;
}
class pcl_grabber_graphic_viewPrivate;
class pcl_grabber_graphic_view : public QGraphicsView
{
	Q_OBJECT;
	Q_DECLARE_PRIVATE(pcl_grabber_graphic_view);
public:
	typedef pcl::io::Image ImageType;
	explicit pcl_grabber_graphic_view(QWidget *parent = nullptr);
	virtual ~pcl_grabber_graphic_view() override;
	void set_grabber(pcl::Grabber *grabber);
public Q_SLOTS:
	void update();
private:
	Q_DISABLE_COPY(pcl_grabber_graphic_view);
	pcl_grabber_graphic_viewPrivate *d_ptr;
};
#endif !__PCL_GRABBER_GRAPHIC_VIEW_H__