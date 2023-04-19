// #include "sonar_viewer.h"

// SonarViewer::SonarViewer()
// {
// }
// // SonarViewer::SonarViewer(OculusSonarNode::SharedPtr node) : node_(node)
// // {
// //     publisher_ = node->create_publisher<sensor_msgs::msg::Image>("fan_image", 10);
// // }
// SonarViewer::~SonarViewer() {}

// void SonarViewer::stream_and_filter(const oculus::PingMessage::ConstPtr &ping)
// {
//     int width = ping->bearing_count();
//     int height = ping->range_count();
//     int offset = 0; // TODO(hugoyvrn)
//     // int offset = ping->imageOffset();

//     cv::Mat rawDataMat = cv::Mat(height, (width + 4), CV_8U);
//     // #pragma omp parallel for collapse(2)
//     for (int i = 0, k = offset; i < height; i++)
//         for (int j = 0; j < (width + 4); j++)
//             rawDataMat.at<uint8_t>(i, j) = ping->data()[k++]; // TODO(hugoyvrn) Seems wrong for me

//     data = cv::Mat(height, width, CV_64F);
// #pragma omp parallel for collapse(2) // TODO(hugoyvrn)
//     for (int i = 0; i < height; i++)
//         for (int j = 4; j < width + 4; j++)
//             data.at<double>(i, j - 4) = rawDataMat.at<uint8_t>(i, j);

//     cv::Mat img = data;
//     img = img / 255.0;
//     cv::imshow("img", img);

//     cv::Mat img_f;
//     cv::dft(img, img_f, cv::DFT_COMPLEX_OUTPUT);

//     cv::Mat beam(1, img.cols, CV_64F, cv::Scalar::all(0));

//     // set the values of the beam
//     int num_values = img.cols / 20;
//     double values[num_values];
//     for (int i = 0; i < num_values / 2; i++)
//         values[i] = 24 + i;
//     values[num_values / 2] = 70;
//     for (int i = num_values / 2 + 1; i < num_values; i++)
//         values[i] = values[num_values - i - 1];
//     for (int i = 0; i < num_values; i++)
//         beam.at<double>(0, i * img.cols / num_values) = values[i];

//     // normalize the beam
//     cv::Mat psf = (1.0 / cv::sum(beam)[0]) * beam;

//     int kw = psf.rows;
//     int kh = psf.cols;
//     cv::Mat psf_padded = cv::Mat::zeros(img.size(), img.type());
//     psf.copyTo(psf_padded(cv::Rect(0, 0, kh, kw)));

//     // compute (padded) psf's DFT
//     cv::Mat psf_f;
//     cv::dft(psf_padded, psf_f, cv::DFT_COMPLEX_OUTPUT, kh);

//     cv::Mat psf_f_2;
//     cv::pow(psf_f, 2, psf_f_2);
//     cv::transform(psf_f_2, psf_f_2, cv::Matx12f(1, 1));

//     double noise = 0.001;
//     cv::Mat ipsf_f(psf_f.size(), CV_64FC2);
//     for (int i = 0; i < psf_f.rows; i++)
//     {
//         for (int j = 0; j < psf_f.cols; j++)
//         {
//             // compute element-wise division
//             cv::Vec2d val = psf_f.at<cv::Vec2d>(i, j) / (psf_f_2.at<double>(i, j) + noise);
//             // store result in ipsf_f
//             ipsf_f.at<cv::Vec2d>(i, j) = val;
//         }
//     }
//     cv::Mat result_f;
//     cv::mulSpectrums(img_f, ipsf_f, result_f, 0);

//     cv::Mat result;
//     cv::idft(result_f, result, cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);

//     cv::Mat result_shifted_rows(result.size(), result.type());
//     int shift = ceil(kw / 2.0);
//     // similar to roll with shift = -1
//     for (int i = 0; i < result.rows; i++)
//         result.row(i).copyTo(result_shifted_rows.row((i + shift) % result_shifted_rows.rows));

//     shift = ceil(kh / 2.0);
//     for (int i = 0; i < result.cols; i++)
//         result_shifted_rows.col(i).copyTo(result.col((i + shift) % result.cols));
//     result.setTo(0, result < 0);
//     cv::normalize(result, result, 0, 1, cv::NORM_MINMAX);
//     cv::imshow("result", result);
//     cv::waitKey(1);

//     /*std::stringstream file_name;
//     file_name << "/home/jaouadros/catkin_ws/src/Sonar_processing_display/tests/results/before/image_" << image_i << ".jpg";
//     cv::normalize(img, img, 0, 255, cv::NORM_MINMAX);
//     cv::imwrite(file_name.str(), img);
//     std::stringstream file_name2;
//     cv::normalize(result, result, 0, 255, cv::NORM_MINMAX);
//     file_name2 << "/home/jaouadros/catkin_ws/src/Sonar_processing_display/tests/results/after/image_" << image_i << ".jpg";
//     cv::imwrite(file_name2.str(), result);
//     image_i++;*/
// }

// void gammaCorrection(const cv::Mat &src, cv::Mat &dst, const float gamma)
// {
//     float invGamma = 1 / gamma;

//     cv::Mat table(1, 256, CV_8U);
//     uchar *p = table.ptr();
//     for (int i = 0; i < 256; ++i)
//     {
//         p[i] = (uchar)(pow(i / 255.0, invGamma) * 255);
//     }

//     LUT(src, table, dst);
// }

// std::string type2str(int type)
// {
//     std::string r;

//     uchar depth = type & CV_MAT_DEPTH_MASK;
//     uchar chans = 1 + (type >> CV_CN_SHIFT);

//     switch (depth)
//     {
//     case CV_8U:
//         r = "8U";
//         break;
//     case CV_8S:
//         r = "8S";
//         break;
//     case CV_16U:
//         r = "16U";
//         break;
//     case CV_16S:
//         r = "16S";
//         break;
//     case CV_32S:
//         r = "32S";
//         break;
//     case CV_32F:
//         r = "32F";
//         break;
//     case CV_64F:
//         r = "64F";
//         break;
//     default:
//         r = "User";
//         break;
//     }

//     r += "C";
//     r += (chans + '0');

//     return r;
// }

// template <typename T>
// std::vector<double> linspace(T start_in, T end_in, int num_in)
// {
//     std::vector<double> linspaced;

//     double start = static_cast<double>(start_in);
//     double end = static_cast<double>(end_in);
//     double num = static_cast<double>(num_in);

//     if (num == 0)
//     {
//         return linspaced;
//     }
//     if (num == 1)
//     {
//         linspaced.push_back(start);
//         return linspaced;
//     }

//     double delta = (end - start) / (num - 1);

//     for (int i = 0; i < num - 1; ++i)
//     {
//         linspaced.push_back(start + delta * i);
//     }
//     linspaced.push_back(end); // I want to ensure that start and end
//                               // are exactly the same as the input
//     return linspaced;
// }

// int grid_presence_counter[36][25] = {0};
// int grid_absence_counter[36][25] = {0};
// int frames_counter = 0;

// sensor_msgs::msg::Image SonarViewer::publish_fan(const oculus::PingMessage::ConstPtr &ping)
// {
//     int width = ping->bearing_count();
//     int height = ping->range_count();
//     int offset = 0; // TODO(hugoyvrn)
//     // int offset = ping->imageOffset();

//     cv::Mat rawDataMat = cv::Mat(height, (width + 4), CV_8U); // 413 256+4
//     // #pragma omp parallel for collapse(2)
//     for (int i = 0, k = offset; i < height; i++)
//         for (int j = 0; j < (width + 4); j++)
//             rawDataMat.at<uint8_t>(i, j) = ping->data[k++]; // TODO(hugoyvrn, seams wront to me)

//     data = cv::Mat(height, width, CV_32F); // 413 256
// #pragma omp parallel for collapse(2)
//     for (int i = 0; i < height; i++)
//         for (int j = 4; j < width + 4; j++)
//             data.at<float>(i, j - 4) = rawDataMat.at<uint8_t>(i, j);

//     bool gain_processing = 1;
//     if (gain_processing)
//     {
//         cv::Mat gainsMat = cv::Mat(height, 4, CV_32F); // 413 4
// #pragma omp parallel for collapse(2)
//         for (int i = 0; i < height; i++)
//             for (int j = 0; j < 4; j++)
//                 gainsMat.at<float>(i, j) = rawDataMat.at<uint8_t>(i, j);

//         cv::Mat gainsOneColMat = cv::Mat(height, 1, CV_32F); // 413 4
//         gainsMat.convertTo(gainsMat, CV_32F);
//         gainsOneColMat = gainsMat.col(0) + std::pow(2, 8) * gainsMat.col(1) + std::pow(2, 16) * gainsMat.col(2) + std::pow(2, 24) * gainsMat.col(3);

//         cv::sqrt(gainsOneColMat, gainsOneColMat);
//         gainsOneColMat = 1 / gainsOneColMat;
//         // #pragma omp parallel
//         for (int i = 0; i < width; i++)
//             data.col(i) = data.col(i).mul(gainsOneColMat);

//         cv::Mat filtered_cropped;
//         data.copyTo(filtered_cropped);
//         for (int i = 0; i < height; i++)
//             cv::sort(filtered_cropped.row(i), filtered_cropped.row(i), cv::SORT_ASCENDING);

//         cv::Rect cropping = cv::Rect(0, 0, (int)width / 2, height);
//         filtered_cropped = filtered_cropped(cropping);

//         for (int i = 0; i < height - 1; i++)
//         {
//             double r = std::sqrt(cv::sum((filtered_cropped.row(i).mul(filtered_cropped.row(i))) / cv::sum(filtered_cropped.row(i + 1).mul(filtered_cropped.row(i + 1))))[0]);
//             data.row(i) *= r;
//         }
//     }

//     data.convertTo(data, CV_8U, 255);
//     gammaCorrection(data, data, 1.7);

//     // cv::Size data_size = data.size();
//     // cv::resize(data, data, data_size/4);
//     // cv::resize(data, data, data_size);
//     /*cv::Mat datav0_resized;
//     cv::resize(data, datav0_resized, data.size()*3);
//     cv::imshow("datav0", datav0_resized);*/

//     cv::GaussianBlur(data, data, cv::Size(7, 7), 0);
//     cv::erode(data, data, cv::Mat());
//     cv::dilate(data, data, cv::Mat());
//     cv::Scalar mean_scalar = cv::mean(data);
//     float mean_data = mean_scalar.val[0];
//     data = (2 * data - 2 * mean_data) * 2;

//     /*cv::Mat data_resized;
//     cv::resize(data, data_resized, data.size()*3);
//     cv::imshow("data", data_resized);*/

//     // Publish sonar image
//     msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", data).toImageMsg();
//     publisher_->publish(msg);

//     /*cv::Mat mat_threshold;
//     cv::adaptiveThreshold(data, mat_threshold, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 9, 3);
//     cv::dilate(mat_threshold, mat_threshold, cv::Mat());
//     cv::dilate(mat_threshold, mat_threshold, cv::Mat());
//     cv::Mat mat_threshold_resized;
//     cv::resize(mat_threshold, mat_threshold_resized, mat_threshold.size()*4);
//     cv::imshow("mat_threshold_resized", mat_threshold_resized*2);*/

//     // data.copyTo(obstacles_mat);
//     cv::Mat obstacles_mat = cv::Mat::zeros(data.size(), CV_8U);
//     int square_size = 10;
//     std::array<uint8_t, 100> vec_pings; // size = square_size^2
//     // std::vector<std::vector<in>> grid_presence_counter{data.rows/square_size, data.cols/square_size};//counts grid being white
//     // std::cout<<(int)(data.rows/square_size)<<" "<<(int)(data.cols/square_size)<<std::endl;
//     for (int i = 100; i < data.rows && 0; i = i + square_size) // start from 100 to filter surface water on ensta dataset
//     {
//         for (int j = 0, all = 0; j < data.cols; j = j + square_size, all = 0)
//         {
//             for (int k = i; k < i + square_size; k++)
//                 for (int l = j; l < j + square_size; l++)
//                     if (k < obstacles_mat.rows && l < obstacles_mat.cols)
//                         vec_pings[all++] = data.at<uint8_t>(k, l);
//             double sum = std::accumulate(std::begin(vec_pings), std::end(vec_pings), 0.0);
//             double m = sum / vec_pings.size();
//             double accum = 0.0;
//             std::for_each(std::begin(vec_pings), std::end(vec_pings), [&](const double d)
//                           { accum += (d - m) * (d - m); });

//             // double stdev = sqrt(accum / (vec_pings.size()-1));
//             double stdev = sqrt((accum - accum / vec_pings.size()) / (vec_pings.size() - 1));
//             double gamma = 0.1;
//             double s = stdev * (1 - gamma) + gamma * m;
//             if (s > 3)
//                 s = 255;
//             else
//                 s = 0;
//             if (s > 0 && all > (vec_pings.size() / 2) && (grid_presence_counter[i / 10][j / 10]++) > 10) // all>(vec_pings.size()/2) : be sure that the size of the square is square_size*square_size (last zones are not full squares due to subdivision)
//             {
//                 for (int k = i; k < i + square_size; k++)
//                     for (int l = j; l < j + square_size; l++)
//                         if (k < obstacles_mat.rows && l < obstacles_mat.cols)
//                             obstacles_mat.at<uint8_t>(k, l) = s;
//             }
//             else if (s == 0 && grid_presence_counter[i / 10][j / 10] > 10)
//                 grid_presence_counter[i / 10][j / 10] = 0;
//             // std::cout<<i<<" "<<j<<std::endl;
//         }
//     }

//     /*cv::Mat obstacles_mat_resized;
//     cv::resize(obstacles_mat, obstacles_mat_resized, obstacles_mat.size()*4);
//     cv::imshow("obstacles_mat resize", obstacles_mat_resized);*/

//     int bearing = 40;
//     if (ping.fireMessage.masterMode == 1)
//         bearing = 65;
//     else
//         bearing = 40;

//     std::vector<double> ranges = linspace(0., (double)ping.fireMessage.range, height);
//     int image_width = 2 * std::sin(bearing * M_PI / 180) * ranges.size();
//     cv::Mat mat = cv::Mat::zeros(cv::Size(image_width, ranges.size()), CV_8UC3);

//     const float ThetaShift = 1.5 * 180;
//     const cv::Point origin(image_width / 2, ranges.size());
//     bool sonar_view = false;
//     // std::cout<<ranges.size()<<std::endl;//362
//     for (int r = 0; r < ranges.size() && sonar_view; r++)
//     {
//         const double range = ranges[r];
//         std::vector<cv::Point> pts;
//         cv::ellipse2Poly(origin, cv::Size(r, r), ThetaShift, -bearing, bearing, 1, pts);
//         // std::cout<<pts.size()<<std::endl;
//         std::vector<cv::Point> arc_points; // TODO can the size of arc_points be known!?
//         arc_points.push_back(pts[0]);
//         for (int k = 0, s = 1; k < (pts.size() - 1); k++)
//         {
//             cv::LineIterator it(mat, pts[k], pts[k + 1], 4);
//             it++; // The first pts is already initiated in arc_points.push_back(pts[0]);
//             for (int i = 0; i < it.count; i++, ++it)
//             {
//                 cv::Point pt = it.pos();
//                 arc_points.push_back(pt);
//             }
//         }
//         cv::Mat data_rows_resized;
//         // cv::resize(data.row(r), data_rows_resized, cv::Size(arc_points.size(),arc_points.size()));
//         cv::resize(obstacles_mat.row(r), data_rows_resized, cv::Size(arc_points.size(), arc_points.size()));
//         // cv::resize(mat_threshold.row(r), data_rows_resized, cv::Size(arc_points.size(),arc_points.size()));
//         for (int k = 0; k < arc_points.size(); k++)
//             // mat.at<uint8_t>(arc_points[k]) = data_rows_resized.at<uint8_t>(1,k);
//             mat.at<cv::Vec3b>(arc_points[k])[1] = data_rows_resized.at<uint8_t>(1, k);

//         // Draw cone contours
//         mat.at<cv::Vec3b>(arc_points[0]) = cv::Vec3b(0, 0, 255);
//         mat.at<cv::Vec3b>(*(arc_points.end() - 1)) = cv::Vec3b(0, 0, 255);
//         if (r == (ranges.size() - 1))
//             for (int d = 0; d < arc_points.size(); d++)
//                 mat.at<cv::Vec3b>(arc_points[d]) = cv::Vec3b(0, 0, 255);
//     }
//     // concatenate - saving video
//     /*cv::Mat concatenate;
//     cv::hconcat(datav0_resized, data, concatenate);
//     cv::flip(mat, mat, 0);
//     cv::resize(mat, mat, cv::Size(mat.cols, data.rows));
//     concatenate.convertTo(concatenate, mat.type());
//     cv::cvtColor(concatenate, concatenate, cv::COLOR_GRAY2BGR);
//     //std::cout<<mat.rows<<" "<<concatenate.rows<<" "<<mat.dims<<" "<<concatenate.dims<<" "<<mat.type()<<" "<<concatenate.type()<<std::endl;
//     cv::hconcat(concatenate, mat, concatenate);
//     cv::resize(concatenate, concatenate, concatenate.size()*3);
//     cv::imshow("concatenate", concatenate);
//     std::string savingName = "/media/jaouadros/My Passport/PhD/Sonar/obstacles_detection_video/" + std::to_string(++frames_counter) + ".png";
//     cv::imwrite(savingName, concatenate);*/

//     // mat.convertTo(mat, CV_8U, 255);
//     // cv::resize(mat, mat, mat.size()*2);
//     // cv::imshow("mat0", mat*2);
//     // mat = (mat-20)*2;
//     // mat.convertTo(mat, CV_8U);
//     // std::cout<<mat<<std::endl;
//     /*std::string ty =  type2str( mat.type() );
//     printf("Matrix: %s %dx%d \n", ty.c_str(), mat.cols, mat.rows );*/

//     /*cv::resize(mat, mat, cv::Size(mat.cols, data.rows)*3);
//     cv::imshow("mat", mat*2);*/
//     // cv::waitKey(1);
// }
