// me 
#include "pcl_grabber_graphic_view.h"
#include "pcl_grabber_graphic_viewPrivate.h"
pcl_grabber_graphic_view::pcl_grabber_graphic_view(QWidget * parent):
	QGraphicsView(parent),
	d_ptr(new pcl_grabber_graphic_viewPrivate(this))
{
	Q_D(pcl_grabber_graphic_view);
	this->setScene(new QGraphicsScene(this));
}

pcl_grabber_graphic_view::~pcl_grabber_graphic_view()
{
	this->set_grabber(nullptr);
	delete this->d_ptr;
}

void pcl_grabber_graphic_view::set_grabber(pcl::Grabber *grabber)
{
	Q_D(pcl_grabber_graphic_view);
	if (d->grabber) {
		d->image_connection.disconnect();
	}
	d->grabber = grabber;
	if (d->grabber) {
		boost::function<void(const ImageType::Ptr&)> f = 
			boost::bind(&pcl_grabber_graphic_viewPrivate::image_callback, d, _1);
		d->image_connection = d->grabber->registerCallback(f);
	}
}

void pcl_grabber_graphic_view::update()
{
	Q_D(pcl_grabber_graphic_view);
	boost::shared_lock<boost::shared_mutex>(d->image_mutex);
	this->scene()->clear();
	this->scene()->addPixmap(d_ptr->cachedPixMap);
	this->fitInView(this->scene()->sceneRect());
}