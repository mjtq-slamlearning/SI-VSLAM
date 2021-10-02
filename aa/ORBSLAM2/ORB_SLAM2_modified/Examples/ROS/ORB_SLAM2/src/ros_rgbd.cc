#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<fstream>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include "../../../include/System.h"
#include "../../../include/pointcloudmapping.h"
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

using namespace std;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps,  vector<string> &fileSemantic);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::NodeHandle n;

    ros::start();
    
    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<string> fileSemantic;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps, fileSemantic);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    //声明点云发布
    ros::Publisher pubCloud =  n.advertise<sensor_msgs::PointCloud2>("/cloud", 2);
    
    // Main loop
    cv::Mat imRGB, imD, imSemantic;
    for(int ni=0; ni<nImages; ni++)
    {   
        // Read image and depthmap from file

        cout << ni << endl;
        
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],-1);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],-1);
        //imSemantic = cv::imread(string(argv[3])+"/segmentation_masks/"+ vstrImageFilenamesRGB[ni].substr(4,11));
        imSemantic = cv::imread(string(argv[3])+"/"+fileSemantic[ni], -1);

        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,tframe, imSemantic);

        PointCloud::Ptr p = SLAM.mpPointCloudMapping->getPc();

        sensor_msgs::PointCloud2 cloudMsgTemp;
        pcl::toROSMsg(*p, cloudMsgTemp);
        cloudMsgTemp.header.stamp = ros::Time::now();
        cloudMsgTemp.header.frame_id = "/map";
        pubCloud.publish(cloudMsgTemp);

        //cout << "......................" << endl;
        //cout << p->points.size() << endl;

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    ros::spin();

    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    ros::shutdown();

    return 0;
}


void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps, vector<string> &fileSemantic)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);
            string sSe;
            ss >> t;
            ss >> sSe;
            fileSemantic.push_back(sSe);

        }
    }
}

