/**
  *
  * @author Marco Carraro, carraromarco89@gmail.com
  * @date January 2016
  *
  * */

#include <memory>
#include <mutex>
#include <thread>

#include <pcl/console/parse.h>
#include <pcl/io/grabber.h>
#include <pcl/io/image_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

std::mutex cloud_mutex_ground_plane;
bool new_cloud_available_flag = false;
PointCloudT::Ptr cloud_ground_plane (new PointCloudT);

void print_help()
{
  std::cout << "\nPCLZF simple viewer (help):" << std::endl;
  std::cout << "\t -lzf        \t<path_to_folder_with_pclzf_files> "
               "(default: current dir)" << std::endl;
  std::cout << "\t -lzf_fps    \t<int> \tfps for replaying the lzf-files "
               "(default: 30)" << std::endl;
}

/**
 * Cloud callback for Ground Plane People Detector
 */
void
cloud_cb (const PointCloudT::ConstPtr &callback_cloud,
          PointCloudT::Ptr& cloud,
          bool* new_cloud_available_flag)
{

  cloud_mutex_ground_plane.lock ();    // for not overwriting the point cloud
  // from another thread
  pcl::copyPointCloud (*callback_cloud, *cloud);

  *new_cloud_available_flag = true;
  cloud_mutex_ground_plane.unlock ();
}


int main(int argc, char** argv)
{
  print_help();

  std::string lzf_dir("");
  int lzf_fps = 30;

  pcl::console::parse_argument (argc, argv, "-lzf", lzf_dir);
  pcl::console::parse_argument (argc, argv, "-lzf_fps", lzf_fps);

  std::shared_ptr<pcl::Grabber> capture;
  capture.reset (new pcl::ImageGrabber<pcl::PointXYZRGBA> (lzf_dir, lzf_fps,
                                                           false, true));

  pcl::PCDGrabberBase* ispcd = dynamic_cast<pcl::PCDGrabberBase*> (&(*capture));
  boost::function<void
      (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
      boost::bind (&cloud_cb,
                   _1,
                   cloud_ground_plane,
                   &new_cloud_available_flag);

  capture->registerCallback (f);
  capture->start ();

  pcl::visualization::PCLVisualizer viewer("PCLZF Viewer");

  while(!viewer.wasStopped())
  {

    while(!new_cloud_available_flag)
      std::this_thread::sleep_for (std::chrono::milliseconds (1));


    new_cloud_available_flag = false;
    cloud_mutex_ground_plane.try_lock ();

    viewer.removeAllPointClouds ();
    viewer.addPointCloud(cloud_ground_plane,"cloud");
    viewer.spinOnce ();

    cloud_mutex_ground_plane.unlock ();
  }

  capture->stop ();
  return 0;
}
