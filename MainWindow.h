#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#pragma once
// qt
#include <QMainWindow>
// pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
// me j
namespace Ui {
class MainWindow;
}
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
	typedef pcl::PointXYZ PointType;
	typedef typename pcl::PointCloud<pcl::PointXYZ> CloudType;
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
public Q_SLOTS:
	void capture();
	void load();
	void test();
private:
    Ui::MainWindow *ui;
	CloudType::Ptr capturedCloud;
	CloudType::Ptr loadedCloud;
	//CloudType::Ptr alignedCloud;
	pcl::PointCloud<pcl::PointNormal>::Ptr alignedCloud;
};

#endif // MAINWINDOW_H
