// me
#include "test.hpp"
// pcl
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/impl/normal_3d_omp.hpp>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/impl/fpfh_omp.hpp>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/icp.h>
// vtk
#include <vtkPolyDataWriter.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
int ransc(int argc, char **argv) {
	using namespace pcl;
	cerr << "loading" << '\n';
	vtkObject::GlobalWarningDisplayOff();
	PolygonMesh sourcePolygonMesh;
	io::loadPolygonFileVTK("C:/Users/jieji/Desktop/aaa.vtk", sourcePolygonMesh);
	PointCloud<PointNormal>::Ptr sourceCloud(new PointCloud<PointNormal>);
	fromPCLPointCloud2(sourcePolygonMesh.cloud, *sourceCloud);
	//io::loadPCDFile("C:/Users/jieji/Desktop/aaa.pcd", *sourceCloud);
	PolygonMesh targetPolygonMesh;
	io::loadPolygonFileVTK("C:/Users/jieji/Desktop/clippedFace.vtk", targetPolygonMesh);
	//PointCloud<PointNormal> targetCloud;
	PointCloud<PointNormal>::Ptr targetCloud(new PointCloud<PointNormal>);
	fromPCLPointCloud2(targetPolygonMesh.cloud, *targetCloud);
	// Downsample
	cerr << "Downsampling" << '\n';
	//VoxelGrid<PointNormal> gridSource;
	//gridSource.setLeafSize(leaf, leaf, leaf);
	//gridSource.setInputCloud(sourceCloud);
	//gridSource.filter(*sourceCloud);
	VoxelGrid<PointNormal> gridTarget;
	gridTarget.setLeafSize(leaf, leaf, leaf);
	gridTarget.setInputCloud(targetCloud);
	gridTarget.filter(*targetCloud);
	// Estimate normals for scene
	cerr << "Estimate normals" << '\n';
	pcl::NormalEstimationOMP<PointNormal, PointNormal> normalEstimationOMP_source;
	normalEstimationOMP_source.setRadiusSearch(2 * leaf);
	normalEstimationOMP_source.setInputCloud(sourceCloud);
	normalEstimationOMP_source.compute(*sourceCloud);
	pcl::NormalEstimationOMP<PointNormal, PointNormal> normalEstimationOMP_target;
	normalEstimationOMP_target.setRadiusSearch(2 * leaf);
	normalEstimationOMP_target.setInputCloud(targetCloud);
	normalEstimationOMP_target.compute(*targetCloud);
	// Estimate features
	cerr << "Estimate features" << '\n';
	PointCloud<FPFHSignature33>::Ptr sourceFeature(new PointCloud<FPFHSignature33>);
	FPFHEstimationOMP<PointNormal, PointNormal, FPFHSignature33> fpfh_estimationOMP_source;
	fpfh_estimationOMP_source.setRadiusSearch(3 * leaf);
	fpfh_estimationOMP_source.setInputCloud(sourceCloud);
	fpfh_estimationOMP_source.setInputNormals(sourceCloud);
	fpfh_estimationOMP_source.compute(*sourceFeature);
	PointCloud<FPFHSignature33>::Ptr targetFeature(new PointCloud<FPFHSignature33>);
	FPFHEstimationOMP<PointNormal, PointNormal, FPFHSignature33> fpfh_estimationOMP_target;
	fpfh_estimationOMP_target.setRadiusSearch(3 * leaf);
	fpfh_estimationOMP_target.setInputCloud(targetCloud);
	fpfh_estimationOMP_target.setInputNormals(targetCloud);
	fpfh_estimationOMP_target.compute(*targetFeature);
	cerr << *sourceCloud << '\n';
	cerr << *targetCloud << '\n';
	// Perform alignment
	cerr << "Perform alignment" << '\n';
	PointCloud<PointNormal> aligned;
	SampleConsensusPrerejective<PointNormal, PointNormal, FPFHSignature33> sampleConsensusPrerejective;
	sampleConsensusPrerejective.setInputSource(sourceCloud);
	sampleConsensusPrerejective.setSourceFeatures(sourceFeature);
	sampleConsensusPrerejective.setInputTarget(targetCloud);
	sampleConsensusPrerejective.setTargetFeatures(targetFeature);
	sampleConsensusPrerejective.setMaximumIterations(50000); // Number of RANSAC iterations
	sampleConsensusPrerejective.setNumberOfSamples(3); // Number of points to sample for generating/prerejecting a pose
	sampleConsensusPrerejective.setCorrespondenceRandomness(5); // Number of nearest features to use
	sampleConsensusPrerejective.setSimilarityThreshold(0.9f); // Polygonal edge length similarity threshold
	sampleConsensusPrerejective.setMaxCorrespondenceDistance(2.5f * leaf); // Inlier threshold
	sampleConsensusPrerejective.setInlierFraction(0.25f); // Required inlier fraction for accepting a pose hypothesis
	sampleConsensusPrerejective.align(aligned);
	cerr << "RMS: " << sampleConsensusPrerejective.getFitnessScore() << '\n';
	cerr << "Matrix" << '\n' << sampleConsensusPrerejective.getFinalTransformation() << '\n';
	vtkSmartPointer<vtkPolyData> polyData =
		vtkSmartPointer<vtkPolyData>::New();
	pcl::io::pointCloudTovtkPolyData(aligned, polyData);
	vtkSmartPointer<vtkPolyDataWriter> wr =
		vtkSmartPointer<vtkPolyDataWriter>::New();
	wr->SetInputData(polyData);
	wr->SetFileName("C:/Users/jieji/Desktop/aligned.vtk");
	wr->Write();
	cin.get();
	return 0;
}

int icp(int argc, char **argv) {

	using namespace pcl;
	cerr << "loading" << '\n';
	vtkObject::GlobalWarningDisplayOff();
	//PolygonMesh sourcePolygonMesh;
	//io::loadPolygonFileVTK("C:/Users/jieji/Desktop/phantom_frame_clipped_scaled_transform.vtk", sourcePolygonMesh);
	//PointCloud<PointNormal> sourceCloud;
	PointCloud<PointNormal>::Ptr sourceCloud(new PointCloud<PointNormal>);
	//fromPCLPointCloud2(sourcePolygonMesh.cloud, *sourceCloud);
	io::loadPCDFile("C:/Users/jieji/Desktop/aaa.pcd", *sourceCloud);
	PolygonMesh targetPolygonMesh;
	io::loadPolygonFileVTK("C:/Users/jieji/Desktop/clippedFace.vtk", targetPolygonMesh);
	//PointCloud<PointNormal> targetCloud;
	PointCloud<PointNormal>::Ptr targetCloud(new PointCloud<PointNormal>);
	fromPCLPointCloud2(targetPolygonMesh.cloud, *targetCloud);
	// icp
	cerr << "ICP..." << '\n';
	PointCloud<PointNormal> alignCloud;
	IterativeClosestPoint<PointNormal, PointNormal> icp;
	icp.setInputCloud(sourceCloud);
	icp.setInputTarget(targetCloud);
	icp.setMaximumIterations(200);
	icp.align(alignCloud);
	cerr << "RMS: " << icp.getFitnessScore() << '\n';
	io::savePLYFile("C:/Users/jieji/Desktop/aligned.ply", alignCloud);
	cerr << "Finish" << '\n';
	cin.get();
	//feature(argc, argv);
	return 0;
}