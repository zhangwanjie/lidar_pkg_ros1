/**
 * @file lidar_node.cpp
 * @brief ROS 1 版激光雷达驱动节点 
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <cerrno>
#include <limits>

// Linux 串口底层头文件
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <asm/termbits.h>

// OpenCV 头文件
#include <opencv2/opencv.hpp>

// --- 硬编码配置宏 ---
#define LIDAR_BAUDRATE 150000

class LidarNode
{
public:
    LidarNode() 
        : nh_private_("~"), fd_(-1), is_shutdown_(false), last_point_angle_(0.0), scan_count_(0)
    {
        // ==========================================
        // 1. 从 Parameter Server 加载参数 (对应 YAML)
        // ==========================================
        
        // 串口与坐标系参数
        // 如果 YAML 中没有定义，这里会使用第三个参数作为默认值
        nh_private_.param<std::string>("port_name", port_name_, "/dev/ttyACM0");
        nh_private_.param<std::string>("frame_id", frame_id_, "laser");
        
        // 滤波参数 (YAML中是嵌套的 filter: enabled: ...)
        // 在 ROS 1 参数服务器中会变成 "filter/enabled"
        nh_private_.param<bool>("filter/enabled", filter_enabled_, true);
        nh_private_.param<double>("filter/radius", filter_radius_, 0.10);
        nh_private_.param<int>("filter/min_neighbors", filter_min_neighbors_, 2);

        // ==========================================
        // 2. 初始化发布者
        // ==========================================
        scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10);
        full_scan_buffer_.reserve(1000);

        // 3. 初始化OpenCV可视化
        initVisualization();

        // 4. 打开并配置串口
        if (!openSerial(port_name_, LIDAR_BAUDRATE)) {
            ROS_FATAL("Failed to open serial port: %s", port_name_.c_str());
            exit(1);
        }

        // 5. 发送启动指令 (A5 60)
        sendCmd({0xA5, 0x60});
        ROS_INFO("Lidar started. Port: %s", port_name_.c_str());
        ROS_INFO("Filter Config -> Enabled: %s, Rad: %.2fm, Neighbors: %d", 
            filter_enabled_ ? "true" : "false", filter_radius_, filter_min_neighbors_);
    }

    ~LidarNode()
    {
        shutdown();
        cv::destroyAllWindows();
    }

    void run_loop()
    {
        uint8_t buffer[1024];
        ros::Rate r(500); // 500Hz 轮询读取

        while (ros::ok() && !is_shutdown_)
        {
            int n = read(fd_, buffer, sizeof(buffer));
            if (n > 0)
            {
                for (int i = 0; i < n; i++)
                {
                    processByte(buffer[i]);
                }
            }
            else if (n < 0)
            {
                if (errno != EAGAIN) {
                    ROS_WARN("Serial read error: %s", strerror(errno));
                }
            }

            // OpenCV 显示 (可选)
            // cv::waitKey(1); 
            
            ros::spinOnce();
            r.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher scan_pub_;

    std::string port_name_;
    std::string frame_id_;
    
    // 滤波控制变量
    bool filter_enabled_;
    double filter_radius_;
    int filter_min_neighbors_;

    int fd_; 
    bool is_shutdown_;
    int scan_count_;

    // --- 解析相关变量 ---
    enum State {
        WAIT_HEADER1,
        WAIT_HEADER2,
        READ_META,
        READ_PAYLOAD
    };
    
    State state_ = WAIT_HEADER1;
    std::vector<uint8_t> packet_buffer_; 
    uint8_t current_lsn_ = 0;
    size_t target_payload_size_ = 0;

    // --- 点云处理相关变量 ---
    struct LidarPoint
    {
        double angle_rad;
        double distance_m;
        double x; 
        double y;
    };
    std::vector<LidarPoint> full_scan_buffer_;
    double last_point_angle_;
    ros::Time scan_start_time_;
    bool first_packet_of_scan_ = true;

    // --- OpenCV 可视化相关变量 ---
    cv::Mat map_image_;
    const int img_size_ = 800;    
    const double max_range_ = 3.0; 
    double px_scale_;             
    const std::string win_name_ = "Lidar Scan Monitor";

    // ================= 初始化与辅助 =================

    void initVisualization()
    {
        map_image_ = cv::Mat::zeros(img_size_, img_size_, CV_8UC3);
        px_scale_ = (img_size_ / 2.0) / max_range_;
    }

    void resetImage()
    {
        map_image_ = cv::Scalar(0, 0, 0);
        cv::line(map_image_, cv::Point(img_size_/2, 0), cv::Point(img_size_/2, img_size_), cv::Scalar(50, 50, 50), 1);
        cv::line(map_image_, cv::Point(0, img_size_/2), cv::Point(img_size_, img_size_/2), cv::Scalar(50, 50, 50), 1);
        
        std::string status = filter_enabled_ ? "Filter: ON" : "Filter: OFF";
        cv::putText(map_image_, status, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    }

    // ================= 串口底层操作 =================

    bool openSerial(const std::string& port, int baudrate)
    {
        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd_ == -1) {
            ROS_ERROR("Open Error: %s", strerror(errno));
            return false;
        }

        struct termios2 tio;
        if (ioctl(fd_, TCGETS2, &tio) != 0) {
            close(fd_);
            return false;
        }

        tio.c_cflag &= ~CBAUD;
        tio.c_cflag |= BOTHER;
        tio.c_ispeed = baudrate;
        tio.c_ospeed = baudrate;

        tio.c_cflag &= ~PARENB;
        tio.c_cflag &= ~CSTOPB;
        tio.c_cflag &= ~CSIZE;
        tio.c_cflag |= CS8;

        tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tio.c_iflag &= ~(IXON | IXOFF | IXANY);
        tio.c_oflag &= ~OPOST;

        if (ioctl(fd_, TCSETS2, &tio) != 0) {
            close(fd_);
            return false;
        }

        ioctl(fd_, TCFLSH, TCIOFLUSH);
        return true;
    }

    void shutdown()
    {
        if (is_shutdown_) return;
        is_shutdown_ = true;

        if (fd_ != -1)
        {
            std::vector<uint8_t> stop_cmd = {0xA5, 0x00, 0xA5, 0x65, 0xA5, 0x65};
            write(fd_, stop_cmd.data(), stop_cmd.size());
            ioctl(fd_, TCSBRK, 1);
            close(fd_);
            fd_ = -1;
        }
    }

    void sendCmd(const std::vector<uint8_t>& cmd)
    {
        if (fd_ != -1) {
            write(fd_, cmd.data(), cmd.size());
        }
    }

    // ================= 协议解析状态机 =================

    void processByte(uint8_t byte)
    {
        switch (state_)
        {
        case WAIT_HEADER1:
            if (byte == 0xAA) {
                state_ = WAIT_HEADER2;
                packet_buffer_.clear();
                packet_buffer_.push_back(byte);
            }
            break;
        case WAIT_HEADER2:
            if (byte == 0x55) {
                state_ = READ_META;
                packet_buffer_.push_back(byte);
            } else if (byte == 0xAA) {
                state_ = WAIT_HEADER2; 
                packet_buffer_.clear();
                packet_buffer_.push_back(byte);
            } else {
                state_ = WAIT_HEADER1;
                packet_buffer_.clear();
            }
            break;
        case READ_META:
            packet_buffer_.push_back(byte);
            if (packet_buffer_.size() == 4) {
                current_lsn_ = packet_buffer_[3];
                target_payload_size_ = 4 + (current_lsn_ * 3);
                state_ = READ_PAYLOAD;
            }
            break;
        case READ_PAYLOAD:
            packet_buffer_.push_back(byte);
            if (packet_buffer_.size() == 4 + target_payload_size_) {
                parsePacket(packet_buffer_);
                state_ = WAIT_HEADER1; 
            }
            break;
        }
    }

    // ================= 点云转换算法 =================
    
    inline uint16_t bytesToUint16(const std::vector<uint8_t> &data, size_t index)
    {
        return (static_cast<uint16_t>(data[index + 1]) << 8) | data[index];
    }

    void parsePacket(const std::vector<uint8_t> &packet_data)
    {
        if (first_packet_of_scan_) {
            scan_start_time_ = ros::Time::now(); 
            first_packet_of_scan_ = false;
        }

        uint8_t lsn = packet_data[3];
        if (lsn == 0) return;

        uint16_t fsangle_raw = bytesToUint16(packet_data, 4);
        uint16_t lsangle_raw = bytesToUint16(packet_data, 6);

        double angle_start_deg = static_cast<double>(fsangle_raw >> 1) / 64.0;
        double angle_end_deg = static_cast<double>(lsangle_raw >> 1) / 64.0;

        double diff_angle_deg = 0.0;
        if (lsn > 1) {
            diff_angle_deg = angle_end_deg - angle_start_deg;
            if (diff_angle_deg < 0) {
                diff_angle_deg += 360.0;
            }
        }

        for (int i = 0; i < lsn; ++i)
        {
           size_t offset = 8 + i * 3;
           if (offset + 1 >= packet_data.size()) break; 

           uint16_t dist_raw = bytesToUint16(packet_data, offset);
           
           double distance_m = static_cast<double>(dist_raw) / 4.0 / 1000.0;
           double distance_mm = static_cast<double>(dist_raw) / 4.0;

           double angle_deg = angle_start_deg;
           if (lsn > 1) {
               angle_deg = (diff_angle_deg / (lsn - 1)) * i + angle_start_deg;
           }

           double angle_correct_deg = 0.0;
           if (distance_mm != 0) {
               double numerator = 21.8 * (155.3 - distance_mm);
               double denominator = 155.3 * distance_mm;
               double angle_correct_rad = std::atan(numerator / denominator);
               angle_correct_deg = angle_correct_rad * 180.0 / M_PI;
           }

           double final_angle_deg = angle_deg + angle_correct_deg;
           final_angle_deg = std::fmod(final_angle_deg, 360.0);
           if (final_angle_deg < 0) final_angle_deg += 360.0;

           double angle_rad = M_PI * final_angle_deg / 180.0;

            if (distance_m > 0.01) 
            {
                if (distance_m <= max_range_) 
                {
                    int center_xy = img_size_ / 2;
                    int px = center_xy + static_cast<int>(distance_m * px_scale_ * std::sin(angle_rad));
                    int py = center_xy - static_cast<int>(distance_m * px_scale_ * std::cos(angle_rad));

                    if (px >= 0 && px < img_size_ && py >= 0 && py < img_size_) {
                        cv::circle(map_image_, cv::Point(px, py), 1, cv::Scalar(0, 0, 255), -1);
                    }
                }

                // 处理一圈结束
                if (angle_rad < last_point_angle_ - M_PI) 
                {
                    if (scan_count_ > 0) {
                        publishScan();
                        // cv::imshow(win_name_, map_image_);
                    } else {
                        ROS_INFO("Skipping first partial scan...");
                    }
                    
                    scan_count_++;
                    resetImage();
                    full_scan_buffer_.clear();
                    first_packet_of_scan_ = true;
                    scan_start_time_ = ros::Time::now();
                }

                LidarPoint p;
                p.angle_rad = angle_rad;
                p.distance_m = distance_m;
                p.x = distance_m * std::cos(angle_rad);
                p.y = distance_m * std::sin(angle_rad);
                
                full_scan_buffer_.push_back(p);
                last_point_angle_ = angle_rad;
            }
        }
    }

    /**
     * @brief 半径滤波算法
     */
    void removeOutliers(std::vector<LidarPoint>& points)
    {
        if (points.empty()) return;

        std::vector<LidarPoint> clean_points;
        clean_points.reserve(points.size());

        double r2 = filter_radius_ * filter_radius_;

        for (size_t i = 0; i < points.size(); ++i) {
            int neighbors = 0;
            const auto& p1 = points[i];

            for (size_t j = 0; j < points.size(); ++j) {
                double dx = p1.x - points[j].x;
                double dy = p1.y - points[j].y;
                if ((dx*dx + dy*dy) < r2) {
                    neighbors++;
                }
                if (neighbors >= filter_min_neighbors_) break;
            }

            if (neighbors >= filter_min_neighbors_) {
                clean_points.push_back(p1);
            }
        }
        points = std::move(clean_points);
    }

    void publishScan()
    {
        if (full_scan_buffer_.empty()) return;

        if (filter_enabled_) {
            removeOutliers(full_scan_buffer_);
        }

        sensor_msgs::LaserScan scan;
        scan.header.stamp = (scan_start_time_.isZero()) ? ros::Time::now() : scan_start_time_;
        scan.header.frame_id = frame_id_;

        scan.angle_min = 0.0;
        scan.angle_max = 2.0 * M_PI;
        
        const int SCAN_SIZE = 720; 
        scan.angle_increment = (2.0 * M_PI) / SCAN_SIZE;
        
        scan.range_min = 0.05;
        scan.range_max = 8.0;

        scan.ranges.assign(SCAN_SIZE, std::numeric_limits<float>::infinity());

        for (const auto& point : full_scan_buffer_)
        {
            double raw_angle = point.angle_rad;
            
            // --- 倒序索引重定向逻辑 ---
            double index_angle = 2.0 * M_PI - raw_angle;
            
            if (index_angle >= 2.0 * M_PI) index_angle -= 2.0 * M_PI;
            if (index_angle < 0) index_angle += 2.0 * M_PI;

            int index = static_cast<int>(index_angle / scan.angle_increment);

            if (index >= 0 && index < SCAN_SIZE)
            {
                if (scan.ranges[index] == std::numeric_limits<float>::infinity() || 
                    point.distance_m < scan.ranges[index])
                {
                    scan.ranges[index] = static_cast<float>(point.distance_m);
                }
            }
        }

        scan.scan_time = 1.0 / 7.0; 
        scan.time_increment = scan.scan_time / SCAN_SIZE;

        scan_pub_.publish(scan);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_node");
    LidarNode node;
    try {
        node.run_loop();
    }
    catch (const std::exception &e) {
        ROS_ERROR("Exception: %s", e.what());
    }
    return 0;
}