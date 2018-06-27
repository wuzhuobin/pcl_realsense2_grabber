#ifndef __PCL_GRABBER_VIEWER_PRIVATE_H__
#define __PCL_GRABBER_VIEWER_PRIVATE_H__
#pragma once
// pcl
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/Grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
// me 
#include "pcl_grabber_viewer.h"
// boost
#include <boost/chrono.hpp>
// vtk
#include <vtkBoxWidget2.h>
#include <vtkSmartPointer.h>
#include <vtkBoxRepresentation.h>
class pcl_grabber_viewerPrivate
{
public:
	Q_DECLARE_PUBLIC(pcl_grabber_viewer);
	typedef typename pcl_grabber_viewer::PointType PointType;
	typedef typename pcl_grabber_viewer::CloudType CloudType;
	pcl_grabber_viewerPrivate(pcl_grabber_viewer *q);
	void cloud_callback(const CloudType::ConstPtr &cloud);
	pcl_grabber_viewer *q_ptr;
	pcl::visualization::PCLVisualizer visualizer;
	pcl::Grabber *grabber;
	CloudType::Ptr cloud;
	pcl::CropBox<PointType> crop_box;
	Eigen::Vector4f max;
	Eigen::Vector4f min;
	boost::shared_mutex cloud_mutex;
	boost::signals2::connection cloud_connection;
	vtkSmartPointer<vtkBoxWidget2> boxWidget;
	vtkSmartPointer<vtkBoxRepresentation> boxRep;
};

inline pcl_grabber_viewerPrivate::pcl_grabber_viewerPrivate(pcl_grabber_viewer *q)
	:q_ptr(q),
	visualizer("", false),
	grabber(nullptr),
	max(1, 1, 1, 1),
	min(-1, -1, -1, 1),
	boxWidget(vtkSmartPointer<vtkBoxWidget2>::New()), 
	boxRep(vtkSmartPointer<vtkBoxRepresentation>::New())
{
	q->SetRenderWindow(this->visualizer.getRenderWindow());
	this->visualizer.addOrientationMarkerWidgetAxes(q->GetInteractor());
	double placed[6]{ 0, 0, 0, 0, 0, 0 };
	this->boxRep->PlaceWidget(placed);
	this->boxRep->HandlesOff();
	this->boxRep->SetPlaceFactor(1);
	//this->boxRep->SetOutlineFaceWires(true);
	this->boxWidget->SetRepresentation(this->boxRep);
	this->boxWidget->SetTranslationEnabled(false);
	this->boxWidget->SetScalingEnabled(false);
	this->boxWidget->SetRotationEnabled(false);
	this->boxWidget->SetMoveFacesEnabled(false);
	this->boxWidget->SetInteractor(q->GetInteractor());
	this->boxWidget->SetEnabled(true);

}
inline void pcl_grabber_viewerPrivate::cloud_callback(const pcl_grabber_viewer::CloudType::ConstPtr & cloud)
{
	Q_Q(pcl_grabber_viewer);
	boost::unique_lock<boost::shared_mutex>(this->cloud_mutex);
	CloudType::Ptr _cloud(new CloudType);
	pcl::transformPointCloud(*cloud, *_cloud, Eigen::Affine3d(Eigen::Scaling(1000.0)));
	pcl::CropBox<PointType> crop_box;
	crop_box.setMax(this->max);
	crop_box.setMin(this->min);
	crop_box.setInputCloud(_cloud);
	//crop_box.setKeepOrganized(true);
	crop_box.setKeepOrganized(false);
	crop_box.filter(*_cloud);
	this->cloud = _cloud;

	QMetaObject::invokeMethod(q, "update");
}
#endif // !__PCL_GRABBER_VIEWER_PRIVATE_H__
