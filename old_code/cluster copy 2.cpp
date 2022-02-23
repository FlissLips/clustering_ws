
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <string>

#include <pcl/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>

ros::Publisher pub;
void cluster_section(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{

    // Point Clouds for Pre- and Post-Clustered Data
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);

    // Convert to the PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);
    // Convert from PointCloud2 to PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud, *input);

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(input);

    // Setting up the cluster extraction
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    ec.setClusterTolerance(0.8); // 2cm
    ec.setMinClusterSize(1);
    ec.setMaxClusterSize(250);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input);
    ec.extract(cluster_indices);

    // This part is only for testing, by changing each cluster to a randomly generated colour
    // TODO: Improve visualisation of clusters by adding ROS markers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::uint8_t r = std::rand() % 256, g = std::rand() % 256, b = std::rand() % 256;
        for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            pcl::PointXYZ cloud_cluster;
            pcl::PointXYZRGB cloud_colour;
            cloud_cluster = input->points[*pit];
            cloud_colour.x = cloud_cluster.x;
            cloud_colour.y = cloud_cluster.y;
            cloud_colour.z = cloud_cluster.z;
            std::uint32_t rgb = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
            cloud_colour.rgb = *reinterpret_cast<float *>(&rgb);
            cluster_cloud->push_back(cloud_colour);
        }
        
        *clustered_cloud += *cluster_cloud;
        clustered_cloud->header.frame_id = "velodyne";
  }
    // Testing markers
    // markers();
    //  convert to ROS data type
    pcl::toROSMsg(*clustered_cloud, *output);

    // publish the output data
    pub.publish(output);
}
void (){

}
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "pcl_cluster");
    ros::NodeHandle nh;

    // a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("input", 1, cluster_section);

    // a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("/cluster_output", 1);

    // a ROS publisher for the marker array
    //   visualMarkerPub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    ros::spin();
}