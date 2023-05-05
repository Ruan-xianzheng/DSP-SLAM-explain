/**
* This file is part of https://github.com/JingwenWang95/DSP-SLAM
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<opencv2/core/core.hpp>

#include"System.h"

using namespace std;
//图片加载函数声明 相较于orb-slam2增加了fps
void LoadImages(const string &strSequence, const float &fps, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    /*终端输入的命令：./dsp_slam_mono （可执行文件）
    Vocabulary/ORBvoc.bin（词典文件） 
    configs/redwood_09374.yaml （配置文件）
    data/09374/ （数据集路径）
    map/redwood_09374/ （存放结果的路径）*/
    if(argc != 5)
    {
        cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_settings path_to_sequence path_to_saved_trajectory" << endl;
        return 1;
    }
    //读取指定路径下的配置文件，将其存放到fSettings对象中
    cv::FileStorage fSettings(string(argv[2]), cv::FileStorage::READ);
    //从配置文件中读取名为"Camera.fps"的参数的值，即相机帧率
    float fps = fSettings["Camera.fps"];

    // Retrieve paths to images
    //按顺序存放需要读取的图像序列、时间戳
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;    
    //按照存放数据集文件路径，将每张图片数据集的路径存到vstrImageFilenames中，相应的时间戳存到vTimestamps容器中
    LoadImages(string(argv[3]), fps, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    /*初始化ORB-SLAM2系统 
    argv[1]是Vocabulary/ORBvoc.txt（词典文件）argv[2]是配置文件 argv[3]是数据集路径
    该语句的作用是创建一个ORB-SLAM2系统对象 
    并初始化其配置。同时指定了系统的运行模式为MONOCULAR
    */    
    ORB_SLAM2::System SLAM(argv[1], argv[2], argv[3], ORB_SLAM2::System::MONOCULAR);

    // Vector for tracking time statistics
    // 创建一个容器保存跟踪时间
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni = 0; ni < nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        //计算相邻两个关键帧之间的时间差T
        double T = 0.0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<size_t>((T- ttrack)*1e6)));
        }

//        if (SLAM.GetTrackingState() == ORB_SLAM2::Tracking::OK)
//            SLAM.SaveMapCurrentFrame(string(argv[4]), ni);
    }
    //保存整张地图
    SLAM.SaveEntireMap(string(argv[4]));

    cv::waitKey(0);

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    return 0;
}

void LoadImages(const string &strPathToSequence, const float &fps, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;//fTimes是输入文件流格式的变量，用来读取文件中的数据
    string strPathTimeFile = strPathToSequence + "/times.txt";//定义时间戳所在的路径
    fTimes.open(strPathTimeFile.c_str());//将时间戳文件转化成c++能够识别的文件，并与fTimes变量关联
    float dt = 1. / fps; //相机获取图像然后完全读出该图像所需时间 1/15.0 = 0.06666
    float t = 0.;
    //一直读取，直到文件结束 
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);//获取fTimes文件中每一行数据，并传递给s变量
        //如果没有空行 则将时间戳存放在vTimestamps容器中
        if(!s.empty())
        {
            vTimestamps.push_back(t); 
            t += dt;  //怎么感觉这里的t有问题
        }
    }
    //定义单目数据集的地址
    string strPrefixLeft = strPathToSequence + "/image_0/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);//重新设置vstrImageFilenames容器大小

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
        vTimestamps.push_back(t);
        t += dt;
    }
}


