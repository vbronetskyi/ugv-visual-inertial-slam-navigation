/**
 * Offline RGB-D-Inertial ORB-SLAM3 runner.
 * Reads pre-extracted RGB, Depth, and IMU files (TUM-style format).
 * Usage: ./rgbd_inertial_offline vocab settings data_dir associations imu_file
 */
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <System.h>

using namespace std;

void LoadImages(const string &strAssociationFilename,
                vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD,
                vector<double> &vTimestamps);

void LoadIMU(const string &strImuPath,
             vector<double> &vTimeStamps,
             vector<cv::Point3f> &vAcc,
             vector<cv::Point3f> &vGyro);

int main(int argc, char **argv)
{
    if(argc != 6) {
        cerr << "Usage: ./rgbd_inertial_offline vocab settings data_dir associations imu_file" << endl;
        return 1;
    }

    string vocabPath = argv[1];
    string settingsPath = argv[2];
    string dataDir = argv[3];
    string assocPath = argv[4];
    string imuPath = argv[5];

    // Load images
    vector<string> vstrImageFilenamesRGB, vstrImageFilenamesD;
    vector<double> vTimestampsImg;
    LoadImages(assocPath, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestampsImg);

    int nImages = vstrImageFilenamesRGB.size();
    if(nImages == 0) {
        cerr << "No images found!" << endl;
        return 1;
    }
    cout << "Images: " << nImages << endl;

    // Load IMU
    vector<double> vTimestampsIMU;
    vector<cv::Point3f> vAcc, vGyro;
    LoadIMU(imuPath, vTimestampsIMU, vAcc, vGyro);
    cout << "IMU samples: " << vTimestampsIMU.size() << endl;

    // Create SLAM system (RGB-D + IMU)
    ORB_SLAM3::System SLAM(vocabPath, settingsPath, ORB_SLAM3::System::IMU_RGBD, false);

    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    int imu_idx = 0;

    cout << "Processing " << nImages << " frames..." << endl;

    for(int ni = 0; ni < nImages; ni++) {
        // Read images
        cv::Mat imRGB = cv::imread(dataDir + "/" + vstrImageFilenamesRGB[ni], cv::IMREAD_UNCHANGED);
        cv::Mat imD = cv::imread(dataDir + "/" + vstrImageFilenamesD[ni], cv::IMREAD_UNCHANGED);
        double tframe = vTimestampsImg[ni];

        if(imRGB.empty() || imD.empty()) {
            cerr << "Failed to load image pair " << ni << endl;
            continue;
        }

        // Collect IMU measurements between previous and current frame
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        double t_prev = (ni > 0) ? vTimestampsImg[ni-1] : tframe - 0.11;

        while(imu_idx < (int)vTimestampsIMU.size() && vTimestampsIMU[imu_idx] <= tframe) {
            if(vTimestampsIMU[imu_idx] > t_prev) {
                vImuMeas.push_back(ORB_SLAM3::IMU::Point(
                    vAcc[imu_idx].x, vAcc[imu_idx].y, vAcc[imu_idx].z,
                    vGyro[imu_idx].x, vGyro[imu_idx].y, vGyro[imu_idx].z,
                    vTimestampsIMU[imu_idx]));
            }
            imu_idx++;
        }

        auto t1 = chrono::steady_clock::now();
        SLAM.TrackRGBD(imRGB, imD, tframe, vImuMeas);
        auto t2 = chrono::steady_clock::now();
        double ttrack = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
        vTimesTrack[ni] = ttrack;

        if(ni % 100 == 0)
            cout << "  Frame " << ni << "/" << nImages << " (" << vImuMeas.size() << " IMU)" << endl;

        // Wait to simulate real-time - needed for IMU preintegration threads
        double T = 0;
        if(ni < nImages - 1)
            T = vTimestampsImg[ni+1] - tframe;
        else if(ni > 0)
            T = tframe - vTimestampsImg[ni-1];
        if(ttrack < T)
            usleep((T-ttrack)*1e6);
    }

    cout << "Shutting down..." << endl;
    SLAM.Shutdown();

    // Save trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    cout << "Trajectories saved." << endl;

    // Tracking time stats
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for(auto t : vTimesTrack) totaltime += t;
    cout << "Median tracking time: " << vTimesTrack[nImages/2] << "s" << endl;
    cout << "Mean tracking time: " << totaltime/nImages << "s" << endl;

    return 0;
}

void LoadImages(const string &strAssociationFilename,
                vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD,
                vector<double> &vTimestamps)
{
    ifstream fAssociation(strAssociationFilename);
    while(!fAssociation.eof()) {
        string s;
        getline(fAssociation, s);
        if(!s.empty() && s[0] != '#') {
            stringstream ss(s);
            double t;
            string sRGB, sD;
            double td;
            ss >> t >> sRGB >> td >> sD;
            vTimestamps.push_back(t);
            vstrImageFilenamesRGB.push_back(sRGB);
            vstrImageFilenamesD.push_back(sD);
        }
    }
}

void LoadIMU(const string &strImuPath,
             vector<double> &vTimeStamps,
             vector<cv::Point3f> &vAcc,
             vector<cv::Point3f> &vGyro)
{
    ifstream fImu(strImuPath);
    while(!fImu.eof()) {
        string s;
        getline(fImu, s);
        if(!s.empty() && s[0] != '#') {
            stringstream ss(s);
            double t, gx, gy, gz, ax, ay, az;
            ss >> t >> gx >> gy >> gz >> ax >> ay >> az;
            vTimeStamps.push_back(t);
            vGyro.push_back(cv::Point3f(gx, gy, gz));
            vAcc.push_back(cv::Point3f(ax, ay, az));
        }
    }
}
