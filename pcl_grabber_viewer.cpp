// me
#include "pcl_grabber_viewer.h"
#include "pcl_grabber_viewerPrivate.h"
//pcl
#include <pcl/io/Grabber.h>
// vtk
#include <vtkRenderWindow.h>
// qt
#include <QDebug>
pcl_grabber_viewer::pcl_grabber_viewer(QWidget * parent)
	:QVTKWidget(parent),
	d_ptr(new pcl_grabber_viewerPrivate(this))
{
	Q_D(pcl_grabber_viewer);
	this->SetRenderWindow(d->visualizer.getRenderWindow());
}

pcl_grabber_viewer::~pcl_grabber_viewer()
{
	delete d_ptr;
}

void pcl_grabber_viewer::set_grabber(pcl::Grabber * grabber)
{
	Q_D(pcl_grabber_viewer);
	if (!d->grabber) {
		d->cloud_connection.disconnect();
	}
	d->grabber = grabber;
	boost::function<void(const CloudType::ConstPtr&)> f =
		boost::bind(&pcl_grabber_viewerPrivate::cloud_callback, d, _1);
	d->cloud_connection = d->grabber->registerCallback(f);
}

pcl::visualization::PCLVisualizer * pcl_grabber_viewer::get_visualizer()
{
	Q_D(pcl_grabber_viewer);
	return &d->visualizer;
}

pcl_grabber_viewer::CloudType::Ptr pcl_grabber_viewer::get_cloud() 
{
	Q_D(pcl_grabber_viewer);
	boost::shared_lock<boost::shared_mutex>(d->cloud_mutex);
	return d->cloud;
}

void pcl_grabber_viewer::new_cloud()
{
}

void pcl_grabber_viewer::update()
{
	Q_D(pcl_grabber_viewer);
	boost::shared_lock<boost::shared_mutex>(d->cloud_mutex);
	if (!d->visualizer.updatePointCloud(d->cloud)) {
		d->visualizer.addPointCloud(d->cloud);
		d->visualizer.resetCameraViewpoint();
		d->visualizer.setCameraPosition(
			0, 0, 0,
			0, 0, 1,
			0, -1, 0
		);
		d->visualizer.resetCamera();
	}
	QVTKWidget::update();
}

void pcl_grabber_viewer::set_x_min(double m)
{
	Q_D(pcl_grabber_viewer);
	boost::unique_lock<boost::shared_mutex>(d->cloud_mutex);
	d->min[0] = m;
	double placed[6]{d->min[0], d->max[0], d->min[1], d->max[1], d->min[2], d->max[2]};
	d->boxRep->PlaceWidget(placed);
}

void pcl_grabber_viewer::set_x_max(double m)
{
	Q_D(pcl_grabber_viewer);
	boost::unique_lock<boost::shared_mutex>(d->cloud_mutex);
	d->max[0] = m;
	double placed[6]{d->min[0], d->max[0], d->min[1], d->max[1], d->min[2], d->max[2]};
	d->boxRep->PlaceWidget(placed);
}

void pcl_grabber_viewer::set_y_min(double m)
{
	Q_D(pcl_grabber_viewer);
	boost::unique_lock<boost::shared_mutex>(d->cloud_mutex);
	d->min[1] = m;
	double placed[6]{d->min[0], d->max[0], d->min[1], d->max[1], d->min[2], d->max[2]};
	d->boxRep->PlaceWidget(placed);
}

void pcl_grabber_viewer::set_y_max(double m)
{
	Q_D(pcl_grabber_viewer);
	boost::unique_lock<boost::shared_mutex>(d->cloud_mutex);
	d->max[1] = m;
	double placed[6]{d->min[0], d->max[0], d->min[1], d->max[1], d->min[2], d->max[2]};
	d->boxRep->PlaceWidget(placed);
}

void pcl_grabber_viewer::set_z_min(double m)
{
	Q_D(pcl_grabber_viewer);
	boost::unique_lock<boost::shared_mutex>(d->cloud_mutex);
	d->min[2] = m;
	double placed[6]{d->min[0], d->max[0], d->min[1], d->max[1], d->min[2], d->max[2]};
	d->boxRep->PlaceWidget(placed);
}

void pcl_grabber_viewer::set_z_max(double m)
{
	Q_D(pcl_grabber_viewer);
	boost::unique_lock<boost::shared_mutex>(d->cloud_mutex);
	d->max[2] = m;
	double placed[6]{d->min[0], d->max[0], d->min[1], d->max[1], d->min[2], d->max[2]};
	d->boxRep->PlaceWidget(placed);
}
