#ifndef __PCL_GRABBER_VIEWER_H__
#define __PCL_GRABBER_VIEWER_H__
#pragma once
// vtk
#include <QVTKWidget.h>
// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace pcl { 
	class Grabber;
	//struct PointXYZ;
	//template<typename PointT>
	//class PointCloud;
	namespace visualization { class PCLVisualizer; }
}
// me
class pcl_grabber_viewerPrivate;
class pcl_grabber_viewer : public QVTKWidget
{
	Q_OBJECT;
	Q_DECLARE_PRIVATE(pcl_grabber_viewer);
public:
	typedef pcl::PointXYZ PointType;
	typedef typename pcl::PointCloud<PointType> CloudType;
	explicit pcl_grabber_viewer(QWidget *parent = nullptr);
	virtual ~pcl_grabber_viewer() override;
	void set_grabber(pcl::Grabber *grabber);
	pcl::visualization::PCLVisualizer* get_visualizer();
	CloudType::Ptr get_cloud();
	void new_cloud();
public Q_SLOTS:
	void update();
	void set_x_min(double m);
	void set_x_max(double m);
	void set_y_min(double m);
	void set_y_max(double m);
	void set_z_min(double m);
	void set_z_max(double m);
private:
	Q_DISABLE_COPY(pcl_grabber_viewer);
	pcl_grabber_viewerPrivate *d_ptr;
};

#endif // !__PCL_GRABBER_VIEWER_H__
