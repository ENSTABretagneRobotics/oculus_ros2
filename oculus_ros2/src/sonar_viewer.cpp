#include "sonar_viewer.h"

// SonarViewer::SonarViewer(bool *arg)
// {
// }
SonarViewer::SonarViewer(rclcpp::Node *node) : node_(node)
{
    image_publisher_ = node->create_publisher<sensor_msgs::msg::Image>("image", 10);
}
SonarViewer::~SonarViewer() {}

// void SonarViewer::stream_and_filter(const int &width, const int &height, const int &offset, const std::vector<uint8_t> &ping_data, cv::Mat &data)
void SonarViewer::stream_and_filter(const oculus::PingMessage::ConstPtr &ping, cv::Mat &data)
{
    int width = ping->bearing_count();
    int height = ping->range_count();
    int offset = ping->ping_data_offset();

    cv::Mat rawDataMat = cv::Mat(height, (width + 4), CV_8U);
#pragma omp parallel for collapse(2)
    for (int i = 0, k = offset; i < height; i++)
        for (int j = 0; j < (width + 4); j++)
            rawDataMat.at<uint8_t>(i, j) = ping->data()[k++]; // TODO(hugoyvrn) Seems wrong for me

    data = cv::Mat(height, width, CV_64F);
#pragma omp parallel for collapse(2) // TODO(hugoyvrn)
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

void gammaCorrection(const cv::Mat &src, cv::Mat &dst, const float gamma)
{
    float invGamma = 1 / gamma;

    cv::Mat table(1, 256, CV_8U);
    uchar *p = table.ptr();
    for (int i = 0; i < 256; ++i)
    {
        p[i] = (uchar)(pow(i / 255.0, invGamma) * 255);
    }

    LUT(src, table, dst);
}

std::string type2str(int type)
{
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth)
    {
    case CV_8U:
        r = "8U";
        break;
    case CV_8S:
        r = "8S";
        break;
    case CV_16U:
        r = "16U";
        break;
    case CV_16S:
        r = "16S";
        break;
    case CV_32S:
        r = "32S";
        break;
    case CV_32F:
        r = "32F";
        break;
    case CV_64F:
        r = "64F";
        break;
    default:
        r = "User";
        break;
    }

    r += "C";
    r += (chans + '0');

    return r;
}

int grid_presence_counter[36][25] = {0};
int grid_absence_counter[36][25] = {0};
int frames_counter = 0;

void SonarViewer::publish_fan(const oculus_interfaces::msg::Ping &ros_ping_msg) const
{
    // const int offset = ping->ping_data_offset();
    const int offset = 229; // TODO(hugoyvrn)
    publish_fan(ros_ping_msg.n_beams, ros_ping_msg.n_ranges, offset, ros_ping_msg.ping_data, ros_ping_msg.master_mode, ros_ping_msg.range);
}

void SonarViewer::publish_fan(const oculus::PingMessage::ConstPtr &ping) const
{
    publish_fan(ping->bearing_count(), ping->range_count(), ping->ping_data_offset(), ping->data(), ping->master_mode(), ping->range());
}

void SonarViewer::publish_fan(const int &width, const int &height, const int &offset, const std::vector<uint8_t> &ping_data, const int &master_mode, const double &ping_range) const
{
    cv::Mat data;

    cv::Mat rawDataMat = cv::Mat(height, (width + 4), CV_8U); // 413 256+4
    #pragma omp parallel for collapse(2)
    for (int i = 0, k = offset; i < height; i++)
        for (int j = 0; j < (width + 4); j++)
            rawDataMat.at<uint8_t>(i, j) = ping_data[k++]; // TODO(hugoyvrn, seams wront to me)

    int bearing = 40;
    if (master_mode == 1)
        bearing = 65;

    std::vector<double> ranges = linspace(0., (double)ping_range, height);
    int image_width = 2 * std::sin(bearing * M_PI / 180) * ranges.size();
    cv::Mat mat = cv::Mat::zeros(cv::Size(image_width, ranges.size()), CV_8UC3);

    const float ThetaShift = 1.5 * 180;
    const cv::Point origin(image_width / 2, ranges.size());
    for (int r = 0; r < ranges.size(); r++)
    {
        const double range = ranges[r];
        std::vector<cv::Point> pts;
        cv::ellipse2Poly(origin, cv::Size(r, r), ThetaShift, -bearing, bearing, 1, pts);
        std::vector<cv::Point> arc_points; // TODO can the size of arc_points be known!?
        arc_points.push_back(pts[0]);
        for (int k = 0, s = 1; k < (pts.size() - 1); k++)
        {
            cv::LineIterator it(mat, pts[k], pts[k + 1], 4);
            it++; // The first pts is already initiated in arc_points.push_back(pts[0]);
            for (int i = 0; i < it.count; i++, ++it)
            {
                cv::Point pt = it.pos();
                arc_points.push_back(pt);
            }
        }
        cv::Mat data_rows_resized;
        cv::resize(rawDataMat.row(r), data_rows_resized, cv::Size(arc_points.size(), arc_points.size()));
        for (int k = 0; k < arc_points.size(); k++)
            mat.at<cv::Vec3b>(arc_points[k])[1] = data_rows_resized.at<uint8_t>(1, k);

        // Draw cone contours
        mat.at<cv::Vec3b>(arc_points[0]) = cv::Vec3b(0, 0, 255);
        mat.at<cv::Vec3b>(*(arc_points.end() - 1)) = cv::Vec3b(0, 0, 255);
        if (r == (ranges.size() - 1))
            for (int d = 0; d < arc_points.size(); d++)
                mat.at<cv::Vec3b>(arc_points[d]) = cv::Vec3b(0, 0, 255);
    }

    // Publish sonar conic image
    sensor_msgs::msg::Image msg;
    cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", mat).toImageMsg(msg); // TODO(hugoyvrn, "mono8"? Really?)
    std_msgs::msg::Header _header;
    image_publisher_->publish(msg);
}
