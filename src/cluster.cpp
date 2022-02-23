
#define PCL_NO_PRECOMPILE
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <cmath>
#include <math.h>

ros::Publisher pub;

struct EIGEN_ALIGN16 PointXYZIR
{
    PCL_ADD_POINT4D;              // quad-word XYZ
    float intensity;              ///< laser intensity reading
    std::uint16_t ring;           ///< laser ring number
    PCL_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring))

// Point Cloud Pointer
template <typename PointT>
using PCPtr = typename pcl::PointCloud<PointT>::Ptr;
// Kd Tree
template <typename PointT>
using KDTreePtr = typename pcl::search::KdTree<PointT>::Ptr;

int countingPoints(double coneHeight, double coneWidth, double horRes, double vertRes, double distance)
{
    double verticalPoints = coneHeight / (2 * distance * tan(vertRes / 2));
    double horizontalPoints = coneWidth / (2 * distance * tan(horRes / 2));
    double numberOfPoints = 0.5 * verticalPoints * horizontalPoints;
    return int(numberOfPoints);
}
template <typename PointT>
PCPtr<PointT> cluster_section(const PCPtr<PointT> input)
{
    // Creating the KdTree object for the search method of the extraction
    KDTreePtr<PointT> tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(input);

    // Setting up the cluster extraction
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;

    ec.setClusterTolerance(0.8);
    ec.setMinClusterSize(1);
    ec.setMaxClusterSize(250);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input);
    ec.extract(cluster_indices);

    // This part is only for testing, by changing each cluster to a randomly generated colour
    // TODO: Improve visualisation of clusters by adding ROS markers
    PCPtr<PointT> total_cloud(new pcl::PointCloud<PointT>);

    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        PCPtr<PointT> cluster_cloud(new pcl::PointCloud<PointT>);
        for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cluster_cloud->push_back(input->points[*pit]);
        }
        std::cout << "PointCloud representing the Cluster: " << cluster_cloud->size() << " data points." << std::endl;
        cluster_cloud->width = cluster_cloud->points.size();
        cluster_cloud->height = 1;
        cluster_cloud->is_dense = true;
        // Add cone recon here

        // Geometric filter
        // pcl::PointXYZ coneCentre;
        // pcl::computeCentroid(*cluster_cloud, coneCentre);
        // double distance = sqrt(pow(coneCentre.x, 2) + pow(coneCentre.y, 2) + pow(coneCentre.z, 2));
        // double coneHeight = 0.23;
        // double coneWidth = 0.33;
        // double horRes = 0.1 * (3.14 / 180);
        // double vertRes = 0.4 * (3.14 / 180);
        // int numOfPoints = countingPoints(coneHeight, coneWidth, horRes, vertRes, distance);
        // std::cout << "number of expected points = " << numOfPoints << std::endl;
        // std::cout << "distance to cone  = " << distance << std::endl;

        *total_cloud += *cluster_cloud;
        return total_cloud;
    }
}
void dataReadAndConvert(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    // Point Clouds for Pre- and Post-Clustered Data
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);

    // Convert to the PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);
    // Convert from PointCloud2 to PointCloud
    pcl::PointCloud<PointXYZIR>::Ptr input(new pcl::PointCloud<PointXYZIR>);
    pcl::fromPCLPointCloud2(*cloud, *input);
    pcl::PointCloud<PointXYZIR>::Ptr total_cloud = cluster_section<PointXYZIR>(input);
    total_cloud->header.frame_id = "velodyne";

    //  convert to ROS data type
    pcl::toROSMsg(*total_cloud, *output);

    // publish the output data
    pub.publish(output);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "pcl_cluster");
    ros::NodeHandle nh;

    // a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("input", 1, dataReadAndConvert);

    // a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("/cluster_output", 1);

    ros::spin();
}