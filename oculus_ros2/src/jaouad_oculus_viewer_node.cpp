#include <iostream>
using namespace std;

#include "ros/ros.h"
#include <oculus_sonar/OculusPing.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>

#include <rosbag/bag.h>
#include <fstream>
#include <numeric>

class SonarPublisher
{
public:
    SonarPublisher()
    {
        pub_ = n_.advertise<sensor_msgs::Image>("/oculus_sonar/image", 1);
        sub_ = n_.subscribe("/oculus_sonar/ping", 1, &SonarPublisher::ping_callback, this);
    }
    void ping_callback(const oculus_sonar::OculusPing &ping);

private:
    sensor_msgs::ImagePtr msg;
    cv::Mat data;
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};

int image_i = 0;
void SonarPublisher::ping_callback(const oculus_sonar::OculusPing &ping)
{
    int width = ping.nBeams;
    int height = ping.nRanges;
    int offset = ping.imageOffset;

    cv::Mat rawDataMat = cv::Mat(height, (width + 4), CV_8U);
    // #pragma omp parallel for collapse(2)
    for (int i = 0, k = offset; i < height; i++)
        for (int j = 0; j < (width + 4); j++)
            rawDataMat.at<uint8_t>(i, j) = ping.data[k++];

    data = cv::Mat(height, width, CV_64F);
#pragma omp parallel for collapse(2)
    for (int i = 0; i < height; i++)
        for (int j = 4; j < width + 4; j++)
            data.at<double>(i, j - 4) = rawDataMat.at<uint8_t>(i, j);

    cv::Mat img = data;
    img = img / 255.0;
    cv::imshow("img", img);

    cv::Mat img_f;
    cv::dft(img, img_f, cv::DFT_COMPLEX_OUTPUT);

    cv::Mat beam(1, img.cols, CV_64F, cv::Scalar::all(0));

    // set the values of the beam
    int num_values = img.cols / 20;
    double values[num_values];
    for (int i = 0; i < num_values / 2; i++)
        values[i] = 24 + i;
    values[num_values / 2] = 70;
    for (int i = num_values / 2 + 1; i < num_values; i++)
        values[i] = values[num_values - i - 1];
    for (int i = 0; i < num_values; i++)
        beam.at<double>(0, i * img.cols / num_values) = values[i];

    // normalize the beam
    cv::Mat psf = (1.0 / cv::sum(beam)[0]) * beam;

    int kw = psf.rows;
    int kh = psf.cols;
    cv::Mat psf_padded = cv::Mat::zeros(img.size(), img.type());
    psf.copyTo(psf_padded(cv::Rect(0, 0, kh, kw)));

    // compute (padded) psf's DFT
    cv::Mat psf_f;
    cv::dft(psf_padded, psf_f, cv::DFT_COMPLEX_OUTPUT, kh);

    cv::Mat psf_f_2;
    cv::pow(psf_f, 2, psf_f_2);
    cv::transform(psf_f_2, psf_f_2, cv::Matx12f(1, 1));

    double noise = 0.001;
    cv::Mat ipsf_f(psf_f.size(), CV_64FC2);
    for (int i = 0; i < psf_f.rows; i++)
    {
        for (int j = 0; j < psf_f.cols; j++)
        {
            // compute element-wise division
            cv::Vec2d val = psf_f.at<cv::Vec2d>(i, j) / (psf_f_2.at<double>(i, j) + noise);
            // store result in ipsf_f
            ipsf_f.at<cv::Vec2d>(i, j) = val;
        }
    }
    cv::Mat result_f;
    cv::mulSpectrums(img_f, ipsf_f, result_f, 0);

    cv::Mat result;
    cv::idft(result_f, result, cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);

    cv::Mat result_shifted_rows(result.size(), result.type());
    int shift = ceil(kw / 2.0);
    // similar to roll with shift = -1
    for (int i = 0; i < result.rows; i++)
        result.row(i).copyTo(result_shifted_rows.row((i + shift) % result_shifted_rows.rows));

    shift = ceil(kh / 2.0);
    for (int i = 0; i < result.cols; i++)
        result_shifted_rows.col(i).copyTo(result.col((i + shift) % result.cols));
    result.setTo(0, result < 0);
    cv::normalize(result, result, 0, 1, cv::NORM_MINMAX);
    cv::imshow("result", result);
    cv::waitKey(1);

    /*std::stringstream file_name;
    file_name << "/home/jaouadros/catkin_ws/src/Sonar_processing_display/tests/results/before/image_" << image_i << ".jpg";
    cv::normalize(img, img, 0, 255, cv::NORM_MINMAX);
    cv::imwrite(file_name.str(), img);
    std::stringstream file_name2;
    cv::normalize(result, result, 0, 255, cv::NORM_MINMAX);
    file_name2 << "/home/jaouadros/catkin_ws/src/Sonar_processing_display/tests/results/after/image_" << image_i << ".jpg";
    cv::imwrite(file_name2.str(), result);
    image_i++;*/
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "oculus_viewer", ros::init_options::NoSigintHandler);
    SonarPublisher pub;

    // Display display;
    ros::Rate loopRate(10);
    while (1)
    {
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}