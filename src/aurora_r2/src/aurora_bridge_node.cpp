#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>
#include "aurora_pubsdk_inc.h"
#include "cxx/slamtec_remote_public.hxx"
#include <cmath>
#include <limits>

class AuroraBridgeNode;

class AuroraListener : public rp::standalone::aurora::RemoteSDKListener {
public:
    AuroraListener(AuroraBridgeNode* node) : node_(node) {}

    void onConnectionStatus(slamtec_aurora_sdk_connection_status_t status) override {
        printf("[Bridge] Estado de conexión: %d\n", status);
    }
    
    void onLIDARScan(const slamtec_aurora_sdk_lidar_singlelayer_scandata_info_t& header, const slamtec_aurora_sdk_lidar_scan_point_t* points) override;
    void onRawCamImageData(uint64_t timestamp_ns, const rp::standalone::aurora::RemoteImageRef& left, const rp::standalone::aurora::RemoteImageRef& right) override;
    void onIMUData(const slamtec_aurora_sdk_imu_data_t* imu_data, size_t imu_data_count) override;
    void onDepthCameraDataArrived(uint64_t timestamp_ns) override;
private:
    AuroraBridgeNode* node_;
};

class AuroraBridgeNode : public rclcpp::Node {
public:
    AuroraBridgeNode() : Node("aurora_bridge_node") {
        this->declare_parameter("ip_address", "192.168.11.1");
        std::string ip = this->get_parameter("ip_address").as_string();
        
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        left_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/left/image_raw", 10);
        right_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/right/image_raw", 10);
        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/depth/image_raw", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 50);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 20);
        
        auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", map_qos);
        map_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_points", 10);

        odom_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&AuroraBridgeNode::publishOdometry, this));
        map_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&AuroraBridgeNode::publishMap, this));
        map_points_timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&AuroraBridgeNode::publishMapPoints, this));

        listener_ = std::make_unique<AuroraListener>(this);
        sdk_ = rp::standalone::aurora::RemoteSDK::CreateSession(listener_.get());
        
        if (!sdk_) return;

        rp::standalone::aurora::SDKServerConnectionDesc server_desc(ip.c_str());
        if (sdk_->connect(server_desc)) {
            sdk_->controller.setRawDataSubscription(true);
            sdk_->setEnhancedImagingSubscription(SLAMTEC_AURORA_SDK_ENHANCED_IMAGE_TYPE_DEPTH, true);
            sdk_->startBackgroundMapDataSyncing(); 
            
            rp::standalone::aurora::LIDAR2DGridMapGenerationOptions map_options;
            sdk_->lidar2DMapBuilder.startPreviewMapBackgroundUpdate(map_options);
        }
    }

    ~AuroraBridgeNode() {
        if (sdk_) {
            sdk_->disconnect();
            rp::standalone::aurora::RemoteSDK::DestroySession(sdk_);
        }
    }

    void publishScan(const slamtec_aurora_sdk_lidar_singlelayer_scandata_info_t& header, const slamtec_aurora_sdk_lidar_scan_point_t* points) {
        if (header.scan_count == 0) return;
        auto scan_msg = sensor_msgs::msg::LaserScan();
        scan_msg.header.stamp = this->now();
        scan_msg.header.frame_id = "aurora_lidar_frame";
        scan_msg.angle_min = -M_PI;
        scan_msg.angle_max = M_PI;
        scan_msg.angle_increment = (2.0 * M_PI) / header.scan_count;
        scan_msg.range_min = 0.15; 
        scan_msg.range_max = 25.0; 
        scan_msg.ranges.resize(header.scan_count, std::numeric_limits<float>::infinity());
        scan_msg.intensities.resize(header.scan_count, 0.0);

        for (uint32_t i = 0; i < header.scan_count; ++i) {
            float angle = points[i].angle;
            float dist = points[i].dist;
            if (angle >= scan_msg.angle_min && angle <= scan_msg.angle_max) {
                int index = std::round((angle - scan_msg.angle_min) / scan_msg.angle_increment);
                if (index >= 0 && index < (int)header.scan_count && dist > scan_msg.range_min && dist < scan_msg.range_max) {
                    scan_msg.ranges[index] = dist;
                    scan_msg.intensities[index] = points[i].quality;
                }
            }
        }
        scan_pub_->publish(scan_msg);
    }

    void publishImages(uint64_t timestamp_ns, const rp::standalone::aurora::RemoteImageRef& left, const rp::standalone::aurora::RemoteImageRef& right) {
        auto convert_to_ros = [&](const rp::standalone::aurora::RemoteImageRef& img, const std::string& frame_id) -> sensor_msgs::msg::Image::SharedPtr {
            if (img._desc.data_size == 0 || img._data == nullptr) return nullptr;
            int cv_type; std::string encoding;
            if (img._desc.format == 0) { cv_type = CV_8UC1; encoding = "mono8"; }
            else if (img._desc.format == 1) { cv_type = CV_8UC3; encoding = "rgb8"; }
            else return nullptr;

            cv::Mat cv_img(img._desc.height, img._desc.width, cv_type, (void*)img._data, img._desc.stride);
            std_msgs::msg::Header header; header.stamp = this->now(); header.frame_id = frame_id;
            return cv_bridge::CvImage(header, encoding, cv_img).toImageMsg();
        };

        auto left_msg = convert_to_ros(left, "aurora_camera_left_frame");
        if (left_msg) left_img_pub_->publish(*left_msg);

        auto right_msg = convert_to_ros(right, "aurora_camera_right_frame");
        if (right_msg) right_img_pub_->publish(*right_msg);
    }

    void publishDepth(uint64_t timestamp_ns) {
        if (!sdk_) return;
        rp::standalone::aurora::RemoteEnhancedImagingFrame frame;
        if (sdk_->enhancedImaging.peekDepthCameraFrame(frame, SLAMTEC_AURORA_SDK_DEPTHCAM_FRAME_TYPE_DEPTH_MAP)) {
            if (frame.image._desc.data_size == 0 || frame.image._data == nullptr) return;
            if (frame.image._desc.format == 3) {
                cv::Mat cv_img(frame.image._desc.height, frame.image._desc.width, CV_32FC1, (void*)frame.image._data, frame.image._desc.stride);
                std_msgs::msg::Header header;
                header.stamp = this->now();
                header.frame_id = "aurora_depth_frame";
                auto depth_msg = cv_bridge::CvImage(header, "32FC1", cv_img).toImageMsg();
                depth_pub_->publish(*depth_msg);
            }
        }
    }

    void publishIMU(const slamtec_aurora_sdk_imu_data_t* imuBuffer, size_t bufferCount) {
        for (size_t i = 0; i < bufferCount; ++i) {
            auto imu_msg = sensor_msgs::msg::Imu();
            imu_msg.header.stamp = this->now();
            imu_msg.header.frame_id = "aurora_imu_frame";
            
            imu_msg.linear_acceleration.x = imuBuffer[i].acc[0] * 9.80665;
            imu_msg.linear_acceleration.y = imuBuffer[i].acc[1] * 9.80665;
            imu_msg.linear_acceleration.z = imuBuffer[i].acc[2] * 9.80665;
            
            imu_msg.angular_velocity.x = imuBuffer[i].gyro[0] * (M_PI / 180.0);
            imu_msg.angular_velocity.y = imuBuffer[i].gyro[1] * (M_PI / 180.0);
            imu_msg.angular_velocity.z = imuBuffer[i].gyro[2] * (M_PI / 180.0);
            
            imu_pub_->publish(imu_msg);
        }
    }

    void publishOdometry() {
        if (!sdk_) return;
        
        slamtec_aurora_sdk_pose_se3_t pose;
        uint64_t timestamp_ns;
        
        if (sdk_->dataProvider.getCurrentPoseSE3WithTimestamp(pose, timestamp_ns)) {
            auto odom_msg = nav_msgs::msg::Odometry();
            odom_msg.header.stamp = this->now();
            odom_msg.header.frame_id = "map";
            odom_msg.child_frame_id = "aurora_base_link";

            odom_msg.pose.pose.position.x = pose.translation.x;
            odom_msg.pose.pose.position.y = pose.translation.y;
            odom_msg.pose.pose.position.z = pose.translation.z;

            odom_msg.pose.pose.orientation.x = pose.quaternion.x;
            odom_msg.pose.pose.orientation.y = pose.quaternion.y;
            odom_msg.pose.pose.orientation.z = pose.quaternion.z;
            odom_msg.pose.pose.orientation.w = pose.quaternion.w;

            odom_pub_->publish(odom_msg);
        }
    }

    void publishMap() {
        if (!sdk_) return;
        
        const auto& map_ref = sdk_->lidar2DMapBuilder.getPreviewMap();
        if (!slamtec_aurora_sdk_is_valid_handle(map_ref.getHandle())) return;

        slamtec_aurora_sdk_2dmap_dimension_t dim;
        map_ref.getMapDimension(dim);
        
        slamtec_aurora_sdk_rect_t rect;
        rect.x = dim.min_x;
        rect.y = dim.min_y;
        rect.width = dim.max_x - dim.min_x;
        rect.height = dim.max_y - dim.min_y;
        
        if (rect.width <= 0 || rect.height <= 0) return;
        
        slamtec_aurora_sdk_2d_gridmap_fetch_info_t fetch_info;
        std::vector<uint8_t> map_data;
        
        if (map_ref.readCellData(rect, fetch_info, map_data, true)) {
            auto map_msg = nav_msgs::msg::OccupancyGrid();
            map_msg.header.stamp = this->now();
            map_msg.header.frame_id = "map";
            
            map_msg.info.resolution = map_ref.getResolution();
            map_msg.info.width = fetch_info.cell_width;
            map_msg.info.height = fetch_info.cell_height;
            
            map_msg.info.origin.position.x = fetch_info.real_x;
            map_msg.info.origin.position.y = fetch_info.real_y;
            map_msg.info.origin.position.z = 0.0;
            map_msg.info.origin.orientation.w = 1.0;
            
            map_msg.data.resize(map_data.size());
            for (size_t i = 0; i < map_data.size(); ++i) {
                uint8_t val = map_data[i];
                if (val == 0) map_msg.data[i] = 0;
                else if (val == 255) map_msg.data[i] = 100;
                else if (val == 127 || val == 128) map_msg.data[i] = -1;
                else map_msg.data[i] = (int8_t)((val / 255.0) * 100.0);
            }
            
            map_pub_->publish(map_msg);
        }
    }

    void publishMapPoints() {
        if (!sdk_) return;

        rp::standalone::aurora::RemoteMapDataVisitor visitor;
        std::vector<slamtec_aurora_sdk_position3d_t> points;

        visitor.subscribeMapPointData([&](const slamtec_aurora_sdk_map_point_desc_t& mp_data) {
            points.push_back(mp_data.position);
        });

        if (sdk_->dataProvider.accessMapData(visitor)) {
            if (points.empty()) return;

            auto pc_msg = sensor_msgs::msg::PointCloud2();
            pc_msg.header.stamp = this->now();
            pc_msg.header.frame_id = "map"; 

            pc_msg.height = 1;
            pc_msg.width = points.size();
            pc_msg.is_dense = false;
            pc_msg.is_bigendian = false;

            sensor_msgs::PointCloud2Modifier modifier(pc_msg);
            modifier.setPointCloud2FieldsByString(1, "xyz");
            modifier.resize(points.size());

            sensor_msgs::PointCloud2Iterator<float> iter_x(pc_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(pc_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(pc_msg, "z");

            for (const auto& pt : points) {
                *iter_x = pt.x;
                *iter_y = pt.y;
                *iter_z = pt.z;
                ++iter_x; ++iter_y; ++iter_z;
            }

            map_points_pub_->publish(pc_msg);
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_img_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_img_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub_;
    
    rclcpp::TimerBase::SharedPtr odom_timer_;
    rclcpp::TimerBase::SharedPtr map_timer_;
    rclcpp::TimerBase::SharedPtr map_points_timer_;
    
    std::unique_ptr<AuroraListener> listener_;
    rp::standalone::aurora::RemoteSDK *sdk_;
};

void AuroraListener::onLIDARScan(const slamtec_aurora_sdk_lidar_singlelayer_scandata_info_t& header, const slamtec_aurora_sdk_lidar_scan_point_t* points) {
    node_->publishScan(header, points);
}
void AuroraListener::onRawCamImageData(uint64_t timestamp_ns, const rp::standalone::aurora::RemoteImageRef& left, const rp::standalone::aurora::RemoteImageRef& right) {
    node_->publishImages(timestamp_ns, left, right);
}
void AuroraListener::onIMUData(const slamtec_aurora_sdk_imu_data_t* imu_data, size_t imu_data_count) {
    node_->publishIMU(imu_data, imu_data_count);
}
void AuroraListener::onDepthCameraDataArrived(uint64_t timestamp_ns) {
    node_->publishDepth(timestamp_ns);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AuroraBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}