// me 
#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "pcl_grabber_viewer.h"
#include "realsense2_grabber.h"
// pcl
#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/io/vtk_lib_io.h>
// vtk
#include <vtkPolyDataReader.h>
#include <vtkPLYWriter.h>
// qt 
#include <QFileDialog>
const std::string CAPTURE_CLOUD("capture_cloud");
const std::string LOADED_CLOUD("loaded_cloud");
const std::string ALIGNED_CLOUD("aligned_cloud");
//pcl::io::OpenNI2Grabber grabber;
//pcl::io::RealSense2Grabber grabber;
MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent),
    ui(new Ui::MainWindow), 
	grabber(new pcl::io::RealSense2Grabber("", pcl::io::RealSense2Grabber::Mode(
		pcl::io::RealSense2Grabber::Mode::rs2_rs400_resolution::H1280X720,
		pcl::io::RealSense2Grabber::Mode::rs2_rs400_frame_rate::FPS30Hz,
		pcl::io::RealSense2Grabber::Mode::rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY)))
{
    this->ui->setupUi(this);
	this->grabber->start();
	connect(this->ui->doubleSpinBoxMinX, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
		this->ui->grabber_viewer, &pcl_grabber_viewer::set_x_min);
	connect(this->ui->doubleSpinBoxMinY, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
		this->ui->grabber_viewer, &pcl_grabber_viewer::set_y_min);
	connect(this->ui->doubleSpinBoxMinZ, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
		this->ui->grabber_viewer, &pcl_grabber_viewer::set_z_min);
	connect(this->ui->doubleSpinBoxMaxX, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
		this->ui->grabber_viewer, &pcl_grabber_viewer::set_x_max);
	connect(this->ui->doubleSpinBoxMaxY, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
		this->ui->grabber_viewer, &pcl_grabber_viewer::set_y_max);
	connect(this->ui->doubleSpinBoxMaxZ, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
		this->ui->grabber_viewer, &pcl_grabber_viewer::set_z_max);
	this->ui->grabber_viewer->set_x_max(9999);
	this->ui->grabber_viewer->set_y_max(9999);
	this->ui->grabber_viewer->set_z_max(9999);
	this->ui->grabber_viewer->set_x_min(-9999);
	this->ui->grabber_viewer->set_y_min(-9999);
	this->ui->grabber_viewer->set_z_min(-9999);
	this->ui->doubleSpinBoxMinX->setValue(-500);
	this->ui->doubleSpinBoxMaxX->setValue(500);
	this->ui->doubleSpinBoxMinY->setValue(-500);
	this->ui->doubleSpinBoxMaxY->setValue(500);
	this->ui->doubleSpinBoxMinZ->setValue(500);
	this->ui->doubleSpinBoxMaxZ->setValue(1000);
	this->ui->grabber_viewer->set_grabber(grabber);
	this->ui->graphicsView->set_grabber(grabber);
	connect(this->ui->action_Capture, &QAction::triggered,
		this, &MainWindow::capture);
	connect(this->ui->action_Load, &QAction::triggered,
		this, &MainWindow::load);
	connect(this->ui->action_Test, &QAction::triggered,
		this, &MainWindow::test);
	//this->ui->graphicsView->setScene(new QGraphicsScene(this));
	//this->ui->graphicsView->scene()->addPixmap(this->image);
	//boost::function<void(const boost::shared_ptr<pcl::io::Image>&)> f =
	//	[this](const boost::shared_ptr<pcl::io::Image>& image) {
	//	//const pcl::io::ImageRGB24::Ptr imageRGB = static_cast<const pcl::io::ImageRGB24::Ptr>(image);
	//	this->cacheImage = QImage(static_cast<const unsigned char*>(image->getData()), image->getWidth(), image->getHeight (), QImage::Format_RGB888);
	//	this->image.convertFromImage(this->cacheImage);
	//	this->ui->graphicsView->scene()->clear();
	//	this->ui->graphicsView->scene()->addPixmap(this->image);
	//	this->ui->graphicsView->fitInView(this->ui->graphicsView->scene()->sceneRect());
	//};
	//this->grabber->registerCallback(f);
}

MainWindow::~MainWindow()
{
    delete ui;
	this->grabber->stop();
	delete this->grabber;
}

void MainWindow::load()
{
	pcl_grabber_viewer *viewer = this->ui->grabber_viewer;
	pcl::visualization::PCLVisualizer *visualizer = this->ui->grabber_viewer->get_visualizer();
	vtkSmartPointer<vtkPolyDataReader> reader = vtkSmartPointer<vtkPolyDataReader> ::New();
	reader->SetFileName("C:/Users/jieji/Desktop/clippedFace.vtk");
	reader->Update();
	this->loadedCloud.reset(new CloudType);
	pcl::io::vtkPolyDataToPointCloud<PointType>(reader->GetOutput(), *this->loadedCloud);
	pcl::visualization::PointCloudColorHandlerCustom<PointType> colorHandler(loadedCloud, 0, 255, 0);
	if (!visualizer->updatePointCloud(this->loadedCloud, colorHandler, LOADED_CLOUD)) {
		visualizer->addPointCloud(this->loadedCloud, colorHandler, LOADED_CLOUD);
	}
}

#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/impl/normal_3d_omp.hpp>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/impl/fpfh_omp.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/sample_consensus_prerejective.h>
void MainWindow::test()
{
	pcl_grabber_viewer *viewer = this->ui->grabber_viewer;
	pcl::visualization::PCLVisualizer *visualizer = this->ui->grabber_viewer->get_visualizer();
	this->alignedCloud.reset(new pcl::PointCloud<pcl::PointNormal>);
	const float leaf = 3.0f;
	pcl::PointCloud<pcl::PointNormal>::Ptr capturedCloudNormal(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr loadedCloudNormal(new pcl::PointCloud<pcl::PointNormal>);
	pcl::copyPointCloud(*this->capturedCloud, *capturedCloudNormal);
	pcl::copyPointCloud(*this->loadedCloud, *loadedCloudNormal);
	cerr << "Down sampling\n";
	//pcl::VoxelGrid<pcl::PointNormal> capturedGrid;
	//capturedGrid.setInputCloud(capturedCloudNormal);
	//capturedGrid.setLeafSize(leaf, leaf, leaf);
	//capturedGrid.filter(*capturedCloudNormal);
	pcl::VoxelGrid<pcl::PointNormal> loadedGrid;
	loadedGrid.setInputCloud(loadedCloudNormal);
	loadedGrid.setLeafSize(leaf, leaf, leaf);
	loadedGrid.filter(*loadedCloudNormal);
	cerr << "Normal Estimation\n";
	pcl::NormalEstimationOMP<pcl::PointNormal, pcl::PointNormal> normalEstimationOMP_source;
	normalEstimationOMP_source.setRadiusSearch(leaf * 3);
	normalEstimationOMP_source.setInputCloud(capturedCloudNormal);
	normalEstimationOMP_source.compute(*capturedCloudNormal);
	cerr << "capturedCloudNormal" << '\n';
	cerr << *capturedCloudNormal << '\n';
	pcl::NormalEstimationOMP<pcl::PointNormal, pcl::PointNormal> normalEstimationOMP_target;
	normalEstimationOMP_target.setRadiusSearch(leaf * 3);
	normalEstimationOMP_target.setInputCloud(loadedCloudNormal);
	normalEstimationOMP_target.compute(*loadedCloudNormal);
	cerr << "loadedCloudNormal\n";
	cerr << *loadedCloudNormal << '\n';
	cerr << "featuring\n";
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr capturedFeatured(new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fpfh_estimationOMP_source;
	fpfh_estimationOMP_source.setRadiusSearch(leaf * 3);
	fpfh_estimationOMP_source.setInputCloud(capturedCloudNormal);
	fpfh_estimationOMP_source.setInputNormals(capturedCloudNormal);
	fpfh_estimationOMP_source.compute(*capturedFeatured);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr loadedFeature(new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fpfh_estimationOMP_target;
	fpfh_estimationOMP_target.setRadiusSearch(leaf * 3);
	fpfh_estimationOMP_target.setInputCloud(loadedCloudNormal);
	fpfh_estimationOMP_target.setInputNormals(loadedCloudNormal);
	fpfh_estimationOMP_target.compute(*loadedFeature);
	cerr << "capturedCloudNormal\n";
	cerr << *capturedCloudNormal << '\n';
	cerr << "catpuredFeature\n";
	cerr << *capturedFeatured << '\n';
	cerr << "loadedCloudNormal\n";
	cerr << *loadedCloudNormal << '\n';
	cerr << "loaededFeature\n";
	cerr << *loadedFeature << '\n';
	cerr << "Ransc\n";
	pcl::SampleConsensusPrerejective<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> sampleConsensusPrerejective;
	sampleConsensusPrerejective.setInputSource(capturedCloudNormal);
	sampleConsensusPrerejective.setSourceFeatures(capturedFeatured);
	sampleConsensusPrerejective.setInputTarget(loadedCloudNormal);
	sampleConsensusPrerejective.setTargetFeatures(loadedFeature);
	sampleConsensusPrerejective.setMaximumIterations(70000); // Number of RANSAC iterations
	sampleConsensusPrerejective.setNumberOfSamples(3); // Number of points to sample for generating/prerejecting a pose
	sampleConsensusPrerejective.setCorrespondenceRandomness(3); // Number of nearest features to use
	sampleConsensusPrerejective.setSimilarityThreshold(0.8f); // Polygonal edge length similarity threshold
	sampleConsensusPrerejective.setMaxCorrespondenceDistance(2.5f * leaf); // Inlier threshold
	sampleConsensusPrerejective.setInlierFraction(0.4f); // Required inlier fraction for accepting a pose hypothesis
	sampleConsensusPrerejective.align(*this->alignedCloud);
	cerr << "RMS" << sqrt(sampleConsensusPrerejective.getFitnessScore()) << '\n';
	cerr << "Matrix: \n" << sampleConsensusPrerejective.getFinalTransformation() << '\n';
	cerr << "Icp\n";
	pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
	icp.setInputCloud(this->alignedCloud);
	icp.setInputTarget(loadedCloudNormal);
	icp.setMaximumIterations(1000);
	icp.align(*this->alignedCloud);
	std::cerr << "RMS: " << sqrt(icp.getFitnessScore()) << '\n';
	cerr << "Matrix: \n" << icp.getFinalTransformation() << '\n';
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> colorHandler(this->alignedCloud, 0, 0, 255); 
	if (!visualizer->updatePointCloud(this->alignedCloud, colorHandler, ALIGNED_CLOUD)) {
		visualizer->addPointCloud(this->alignedCloud, colorHandler, ALIGNED_CLOUD);
	}
}

void MainWindow::capture()
{
	pcl_grabber_viewer *viewer = this->ui->grabber_viewer;
	pcl::visualization::PCLVisualizer *visualizer = this->ui->grabber_viewer->get_visualizer();
	this->capturedCloud = viewer->get_cloud();
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save Point Cloud"), QString(), QString());
	if (fileName.isEmpty()) {
		return;
	}
	fileName = QFileInfo(fileName).filePath();
	pcl::io::savePCDFile((fileName + ".pcd").toStdString(),  *this->capturedCloud);
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	pcl::io::pointCloudTovtkPolyData(*this->capturedCloud, polyData);
	vtkSmartPointer<vtkPolyDataWriter> polyDataWriter = vtkSmartPointer<vtkPolyDataWriter>::New();
	polyDataWriter->SetFileName((fileName + ".vtk").toStdString().c_str());
	polyDataWriter->SetInputData(polyData);
	polyDataWriter->Write();
	vtkSmartPointer<vtkPLYWriter> plyWriter = vtkSmartPointer<vtkPLYWriter>::New();
	plyWriter->SetFileName((fileName + ".ply").toStdString().c_str());
	plyWriter->SetInputData(polyData);
	plyWriter->Write();
	pcl::visualization::PointCloudColorHandlerCustom<PointType> colorHandler(this->capturedCloud, 255, 0, 0); 
	if (!visualizer->updatePointCloud(this->capturedCloud, colorHandler, CAPTURE_CLOUD)) {
		visualizer->addPointCloud(this->capturedCloud, colorHandler, CAPTURE_CLOUD);
	} 
} 