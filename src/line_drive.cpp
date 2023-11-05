#include <ros/ros.h>
#include <xycar_msgs/xycar_motor.h>
#include <sensor_msgs/Image.h>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <vector>
#include <signal.h>
#include <sys/types.h>
#include <unistd.h>

class MovingAverage
{
public:
    MovingAverage(int n)
    {
        samples = n;
        data = std::vector<float>();
        weights = std::vector<int>(n+1);
        for (int i = 0; i < n+1; i++)
        {
            weights[i] = i;
        }
    }

    void add_sample(float new_sample)
    {
        if (data.size() < samples)
        {
            data.push_back(new_sample);
        }
        else
        {
            data.erase(data.begin());
            data.push_back(new_sample);
        }
    }

    float get_mm()
    {
        float sum = std::accumulate(data.begin(), data.end(), 0.0);
        return sum / data.size();
    }

    float get_wmm()
    {
        float s = 0;
        for (int i = 0; i < data.size(); i++)
        {
            s += data[i] * weights[i];
        }
        return s / std::accumulate(weights.begin(), weights.begin() + data.size(), 0.0);
    }

private:
    int samples;
    std::vector<float> data;
    std::vector<int> weights;
};

class PID
{
public:
    PID(float kp, float ki, float kd)
    {
        Kp = kp;
        Ki = ki;
        Kd = kd;
        p_error = 0.0;
        i_error = 0.0;
        d_error = 0.0;
    }
    void pid_control(double cte)
    {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;

    return Kp*p_error + Ki*i_error + Kd*d_error;
    }
}

void signal_handler(int sig, siginfo_t *siginfo, void *context)
{
    system("killall -9 python rosout");
    exit(0);
}

int Width = 640;
int Height = 480;
int Offset = 390;
int Gap = 40;

std::vector<int> image;

void img_callback(const sensor_msgs::ImageConstPtr& data)
{
    global image;
    image = cv_bridge::toCvShare(data, "bgr8")->image;
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
}

// publish xycar_motor msg
void drive(int Angle, int Speed)
{
    global pub;

    xycar_motor msg;
    msg.angle = int(Angle);
    msg.speed = int(Speed);

    pub.publish(msg);
}

// draw lines
cv::Mat draw_lines(cv::Mat img, std::vector<cv::Vec4i> lines)
{
    global Offset;
    for (auto line : lines)
    {
        cv::Point pt1(line[0], line[1] + Offset);
        cv::Point pt2(line[2], line[3] + Offset);
        cv::Scalar color(cv::randu<uchar>(), cv::randu<uchar>(), cv::randu<uchar>());
        cv::line(img, pt1, pt2, color, 2);
    }
    return img;
}

// draw_rectangle
void draw_rectangle(cv::Mat img, int lpos, int rpos, int offset = 0)
{
    int center = (lpos + rpos) / 2;
    //center2 = (lpos2 + rpos2) / 2

    cv::rectangle(img, cv::Point(lpos - 5, 15 + offset),
                       cv::Point(lpos + 5, 25 + offset),
                       cv::Scalar(0, 255, 0), 2);
    cv::rectangle(img, cv::Point(rpos - 5, 15 + offset),
                       cv::Point(rpos + 5, 25 + offset),
                       cv::Scalar(0, 255, 0), 2);
    cv::rectangle(img, cv::Point(center-5, 15 + offset),
                       cv::Point(center+5, 25 + offset),
                       cv::Scalar(0, 255, 0), 2);
    cv::rectangle(img, cv::Point(315, 15 + offset),
                       cv::Point(325, 25 + offset),
                       cv::Scalar(0, 0, 255), 2);
    return img;
}

void divide_left_right(std::vector<std::vector<int>> lines) {
    int low_slope_threshold = 0;
    int high_slope_threshold = 10;

    // calculate slope & filtering with threshold
    std::vector<float> slopes;
    std::vector<std::vector<int>> new_lines;

    for (int i = 0; i < lines.size(); i++) {
        int x1 = lines[i][0];
        int y1 = lines[i][1];
        int x2 = lines[i][2];
        int y2 = lines[i][3];

        if (x2 - x1 == 0) {
            float slope = 0;
        }
        else {
            float slope = float(y2-y1) / float(x2-x1);
        }
        
        if (abs(slope) > low_slope_threshold && abs(slope) < high_slope_threshold) {
            slopes.push_back(slope);
            new_lines.push_back(lines[i]);
        }
    }

    // divide lines left to right
    std::vector<std::vector<int>> left_lines;
    std::vector<std::vector<int>> right_lines;

    for (int j = 0; j < slopes.size(); j++) {
        std::vector<int> Line = new_lines[j];
        float slope = slopes[j];

        int x1 = Line[0];
        int y1 = Line[1];
        int x2 = Line[2];
        int y2 = Line[3];

        if (slope < 0 && x2 < Width/2 - 90) {
            left_lines.push_back(Line);
        }
        else if (slope > 0 && x1 > Width/2 + 90) {
            right_lines.push_back(Line);
        }
    }
}

// get average m, b of lines
std::pair<float, float> get_line_params(std::vector<std::vector<int>> lines)
{
    // sum of x, y, m
    float x_sum = 0.0;
    float y_sum = 0.0;
    float m_sum = 0.0;

    int size = lines.size();
    if (size == 0)
        return std::make_pair(0, 0);

    for (auto line : lines)
    {
        int x1 = line[0];
        int y1 = line[1];
        int x2 = line[2];
        int y2 = line[3];

        x_sum += x1 + x2;
        y_sum += y1 + y2;
        m_sum += float(y2 - y1) / float(x2 - x1);
    }

    float x_avg = x_sum / (size * 2);
    float y_avg = y_sum / (size * 2);
    float m = m_sum / size;
    float b = y_avg - m * x_avg;

    return std::make_pair(m, b);
}

# get lpos, rpos
void get_line_pos(cv::Mat img, std::vector<cv::Vec4i> lines, bool left, bool right) {
    int m, b;
    get_line_params(lines, m, b);

    if (m == 0 && b == 0) {
        if (left) {
            pos = 0;
        }
        if (right) {
            pos = Width;
        }
    }
    else {
        int y = Gap / 2;
        pos = (y - b) / m;

        b += Offset;
        int x1 = (Height - b) / (float)m;
        int x2 = ((Height / 2) - b) / (float)m;

        cv::line(img, cv::Point(x1, Height), cv::Point(x2, (Height / 2)), cv::Scalar(255, 0, 0), 3);
    }

    return img, int(pos);
}

void process_image(cv::Mat frame)
{
    // gray
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    gray = gray - 50;

    double alpha = 1;
    cv::Mat dst = gray + (gray - 128) * alpha;

    // blur
    int kernel_size = 5;
    cv::Mat blur_gray;
    cv::GaussianBlur(dst, blur_gray, cv::Size(kernel_size, kernel_size), 0);

    cv::threshold(blur_gray, blur_gray, 120, 255, cv::THRESH_BINARY);
    //cv2.imshow('binary',blur_gray)

    // canny edge
    int low_threshold = 60;
    int high_threshold = 70;
    cv::Mat edge_img;
    cv::Canny(blur_gray, edge_img, low_threshold, high_threshold);
    // cv2.imshow('canny',edge_img)

    // HoughLinesP
    cv::Mat roi = edge_img(cv::Rect(0, Offset, Width, Gap));
    std::vector<cv::Vec4i> all_lines;
    cv::HoughLinesP(roi, all_lines, 1, CV_PI/180, 35, 35, 10);

    // divide left, right lines
    if (all_lines.empty())
    {
        return 0, 640;
    }
    std::vector<cv::Vec4i> left_lines, right_lines;
    divide_left_right(all_lines, left_lines, right_lines);

    // get center of lines
    int lpos = get_line_pos(frame, left_lines, true);
    int rpos = get_line_pos(frame, right_lines, false);

    // draw lines
    frame = draw_lines(frame, left_lines);
    frame = draw_lines(frame, right_lines);
    cv::line(frame, cv::Point(230, 235), cv::Point(410, 235), cv::Scalar(255,255,255), 2);
    
    // draw rectangle
    frame = draw_rectangle(frame, lpos, rpos, offset=Offset);

    // show image
    // cv::imshow('calibration', frame);
    
    return std::make_pair(lpos, rpos);

void avg_angle(double angle, std::vector<double> angle_list)
{
    std::vector<double> a_list = angle_list;
    
    if (!a_list.empty())
    {
        if (angle * a_list.back() < 0)
        {
            a_list.clear();
            a_list.push_back(angle);
        }
        else
        {
            if (a_list.size() < 3)
            {
                a_list.push_back(angle);
            }
            else
            {
                a_list.push_back(angle);
                a_list.erase(a_list.begin());
            }
        }
    }
    else
    {
        a_list.push_back(angle);
    }
}

double avg_angle(double angle, std::vector<double> angle_list)
{
    std::vector<double> a_list = angle_list;
    
    if (!a_list.empty())
    {
        if (angle * a_list.back() < 0)
        {
            a_list.clear();
            a_list.push_back(angle);
        }
        else
        {
            if (a_list.size() < 3)
            {
                a_list.push_back(angle);
            }
            else
            {
                a_list.push_back(angle);
                a_list.erase(a_list.begin());
            }
        }
    }
    else
    {
        a_list.push_back(angle);
    }
    return a_list;
}

void start(){
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<xycar_motor>("xycar_motor", 1);
    ros::Subscriber image_sub = nh.subscribe("/usb_cam/image_raw", 1, img_callback);
    ROS_INFO("---------- Xycar A2 v1.0 ----------");
    ros::Duration(2).sleep();

    double angle = 2.5;
    double angle_temp = 2.5;
    int speed = 10;
    std::vector<double> angle_list;

    PID pid(0.56, 0.0007, 0.2);
    MovingAverage mm1(20);
    while(true){
        while(image.size() != (640*480*3)){
            continue;
        }
        cv::imshow("img", image);
        std::pair<int, int> lpos_rpos = process_image(image);

        int center = (lpos_rpos.first + lpos_rpos.second) / 2;
        int error = (center - Width/2);

        if(lpos_rpos.first == 0 && lpos_rpos.second == 640){
            angle_temp = angle;
            drive(angle_temp, 5);
            continue;
        }

        angle = pid.pid_control(error);
        angle_list = avg_angle(angle, angle_list);
        double mean_angle = std::accumulate(angle_list.begin(), angle_list.end(), 0.0) / angle_list.size();

        if(angle_list.size() == 3 && std::abs(mean_angle) >= 50){
            drive(mean_angle*2, 10);
            Offset = 380;
        }
        else{
            drive(mean_angle, 15);
            Offset = 390;
        }

        if(cv::waitKey(1) & 0xFF == ord('q')){
            break;
        }
    }
    ros::spin();
}

int main()
{
    start();
    struct sigaction act;
    act.sa_sigaction = signal_handler;
    sigaction(SIGINT, &act, NULL);
    return 0;
}