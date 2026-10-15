#include "landing.hpp"
#include "vision.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
namespace fs=std::filesystem;
void usage() {
 std::cout << "Usage:\n"
 << "  drone demo [trials]    Simulated PID landings, CSV statistics\n"
 << "  drone marker           Generate results/marker.png\n"
 << "  drone detect image.jpg config/camera.yaml [marker_size_m]\n"
 << "  drone webcam config/camera.yaml [marker_size_m]\n"
 << "  drone calibrate \"calibration/*.jpg\" config/camera.yaml [cols rows square_m]\n";
}
int main(int argc,char** argv) {
 try {
 fs::create_directories("results");
 if(argc<2){usage();return 0;}
 std::string cmd=argv[1];
 if(cmd=="demo") {
  int trials=argc>2?std::stoi(argv[2]):30;
  if(trials<1 || trials>10000){std::cerr<<"Trials must be 1..10000\n";return 1;}
  std::ofstream csv("results/trials.csv");
  csv<<"trial,final_error_m,overshoot_m,settling_time_s,dropped_frames,noise_std_m,drop_probability,target_displacement_m\n";
  double errorSum=0,maxError=0;
  for(int i=0;i<trials;++i){
   double noise=0.01+(i%5)*0.02, drop=(i%4)*0.05, displace=0.5+(i%3)*0.5;
   auto r=runTrial(100+i,noise,drop,displace,0.8,0.02,0.25,i==0?"results/trajectory.csv":nullptr);
   csv<<i<<','<<r.finalError<<','<<r.overshoot<<','<<r.settlingSeconds<<','<<r.dropped<<','<<noise<<','<<drop<<','<<displace<<'\n';
   errorSum+=r.finalError;maxError=std::max(maxError,r.finalError);
  }
  std::cout<<"Completed "<<trials<<" 2D PID kinematic simulations\nAverage final landing/alignment error: "
    <<errorSum/trials<<" m\nMaximum final error: "<<maxError<<" m\nSaved results/trials.csv and results/trajectory.csv\n";
  std::cout<<"NOTE: This is not a real flight or SITL landing.\n";
  return 0;
 }
 if(cmd=="marker") {
   if(!makeMarker("results/marker.png")){std::cerr<<"Could not write marker\n";return 1;}
   std::cout<<"Generated results/marker.png (ID 23, DICT_6X6_250)\n";return 0;
 }
 if(cmd=="calibrate") {
  if(argc<4){usage();return 1;}
  int cols=argc>4?std::stoi(argv[4]):9, rows=argc>5?std::stoi(argv[5]):6;
  double square=argc>6?std::stod(argv[6]):0.025;
  return calibrateCameraFromImages(argv[2],argv[3],cols,rows,square)?0:1;
 }
 if(cmd=="detect" || cmd=="webcam"){
  int index=cmd=="detect"?3:2;
  if(argc<=index){usage();return 1;}
  cv::Mat k,d;
  if(!loadCalibration(argv[index],k,d)) {std::cerr<<"Calibration missing/invalid: "<<argv[index]<<"\n";return 1;}
  double size=argc>index+1?std::stod(argv[index+1]):0.15;
  if(cmd=="detect"){
    cv::Mat img=cv::imread(argv[2]),annotated;
    if(img.empty()){std::cerr<<"Cannot open image\n";return 1;}
    auto pose=detectMarker(img,k,d,size,annotated);
    std::cout<<"Detected: "<<pose.detected<<" ID: "<<pose.id<<" Time: "<<pose.processingMs<<" ms\n";
    if(pose.detected)std::cout<<"Marker relative to camera (m): "<<pose.translation<<"\n";
    if(!annotated.empty())cv::imwrite("results/detected.png",annotated);
    return pose.detected?0:2;
  }
  cv::VideoCapture cap(0);
  if(!cap.isOpened()){std::cerr<<"Cannot open webcam\n";return 1;}
  std::ofstream log("results/vision_latency.csv");log<<"frame,detected,latency_ms,x_m,y_m,z_m\n";
  int frame=0;
  for(;;){cv::Mat img,out;cap>>img;if(img.empty())break;
    auto pose=detectMarker(img,k,d,size,out);
    log<<frame++<<','<<pose.detected<<','<<pose.processingMs<<','<<pose.translation[0]<<','<<pose.translation[1]<<','<<pose.translation[2]<<'\n';
    cv::imshow("ArUco detection (press q to quit)",out);
    if((cv::waitKey(1)&255)=='q')break;
  }
  return 0;
 }
 usage();return 1;
 }catch(const std::exception& e){std::cerr<<"Error: "<<e.what()<<'\n';return 1;}
}
