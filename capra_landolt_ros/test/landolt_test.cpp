#include <memory>
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>
#include <chrono>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/highgui.hpp"
#include "image_transport/image_transport.hpp"

#include "capra_landolt_msgs/msg/landolts.hpp"
#include "capra_landolt_msgs/msg/bounding_circles.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

struct MetaData {
    std::vector<float> angles;
    std::vector<float> radius;
    std::vector<cv::Point2f> centers; 
};

class LandoltTest : public testing::Test
{
protected:
    virtual void SetUp()
    {
        node_ = rclcpp::Node::make_shared("landolt_test_node");

        // WICHTIG: Topics anpassen, damit sie zu den Parametern deiner LandoltNode passen!
        topic_angles_ = "landolts";
        topic_images_ = "/capra/camera_3d/rgb/image_raw"; // Direkt auf das globale Kameratopic matchen
        topic_boundings_ = "boundings";

        node_->declare_parameter<std::string>("datapath", "");
        node_->get_parameter("datapath", data_path_);

        double D[] = {-0.363528858080088, 0.16117037733986861, -8.1109585007538829e-05, -0.00044776712298447841, 0.0};
        double K[] = {430.15433020105519,                0.0, 311.71339830549732,
                      0.0, 430.60920415473657, 221.06824942698509,
                      0.0,                0.0,                1.0};
        double R[] = {0.99806560714807102, 0.0068562422224214027, 0.061790256276695904,
                      -0.0067522959054715113, 0.99997541519165112, -0.0018909025066874664,
                      -0.061801701660692349, 0.0014700186639396652, 0.99808736527268516};
        double P[] = {295.53402059708782, 0.0, 285.55760765075684, 0.0,
                      0.0, 295.53402059708782, 223.29617881774902, 0.0,
                      0.0, 0.0, 1.0, 0.0};

        cam_info_.header.frame_id = "tf_frame";
        cam_info_.height = 480;
        cam_info_.width  = 640;
        cam_info_.d.resize(5);
        std::copy(D, D+5, cam_info_.d.begin());
        std::copy(K, K+9, cam_info_.k.begin());
        std::copy(R, R+9, cam_info_.r.begin());
        std::copy(P, P+12, cam_info_.p.begin());

        // Subscriber & Publisher
        angles_sub_ = node_->create_subscription<capra_landolt_msgs::msg::Landolts>(
            topic_angles_, 10, std::bind(&LandoltTest::landoltsCallback, this, std::placeholders::_1));
        
        bound_sub_ = node_->create_subscription<capra_landolt_msgs::msg::BoundingCircles>(
            topic_boundings_, 10, std::bind(&LandoltTest::boundingsCallback, this, std::placeholders::_1));

        image_sub_ = image_transport::create_subscription(node_.get(), topic_images_,
            std::bind(&LandoltTest::imageCallback, this, std::placeholders::_1), "raw");

        cam_pub_ = image_transport::create_camera_publisher(node_.get(), topic_images_);

        // DDS Discovery etwas Zeit geben (2 Sekunden spinnen statt Endlosschleife)
        auto start_time = node_->now();
        while ((node_->now() - start_time).seconds() < 2.0) {
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    std::string data_path_;
    rclcpp::Node::SharedPtr node_;
    std::string topic_angles_;
    std::string topic_boundings_;
    std::string topic_images_;

    int callback_count_ = 0;
    cv::Mat received_image_;
    std::vector<float> received_landolts_;
    std::vector<capra_landolt_msgs::msg::Point2f> received_centers_;
    std::vector<float> received_radius_;

    sensor_msgs::msg::CameraInfo cam_info_;
    MetaData image_meta_;

    rclcpp::Subscription<capra_landolt_msgs::msg::Landolts>::SharedPtr angles_sub_;
    rclcpp::Subscription<capra_landolt_msgs::msg::BoundingCircles>::SharedPtr bound_sub_;
    image_transport::Subscriber image_sub_;
    image_transport::CameraPublisher cam_pub_;

public:
    void boundingsCallback(const capra_landolt_msgs::msg::BoundingCircles::SharedPtr msg) {
        received_centers_ = msg->centers;
        received_radius_ = msg->radius;
        callback_count_++;
    }

    void landoltsCallback(const capra_landolt_msgs::msg::Landolts::SharedPtr msg) {
        received_landolts_ = msg->angles;
        callback_count_++;
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_FATAL(node_->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        received_image_ = cv_ptr->image.clone();
        callback_count_++;
    }

    bool publishImage(const std::string& imagePath, const std::string& metaPath) {
        callback_count_ = 0;
        cv::Mat mat = cv::imread(imagePath);
        if(mat.empty()) return false;

        std_msgs::msg::Header hdr;
        hdr.stamp = node_->now();
        hdr.frame_id = "tf_frame";
        auto img = cv_bridge::CvImage(hdr, "bgr8", mat).toImageMsg();
        
        cam_info_.header.stamp = hdr.stamp;
        cam_pub_.publish(*img, cam_info_);
        readMetaData(metaPath);

        // Auf Callbacks warten (Erhöhtes Timeout auf 5 Sekunden für DDS-Verbindungen)
        auto start_time = node_->now();
        while(callback_count_ < 3 && (node_->now() - start_time).seconds() < 5.0) {
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return (callback_count_ >= 3);
    }

    void readMetaData(const std::string& path) {
        image_meta_.angles.clear();
        image_meta_.centers.clear();
        image_meta_.radius.clear();

        cv::FileStorage fs(path, cv::FileStorage::READ);
        if(!fs.isOpened()) return;

        fs["radius"] >> image_meta_.radius;
        fs["angles"] >> image_meta_.angles;

        cv::FileNode centersNode = fs["centers"];
        for(auto it = centersNode.begin(); it != centersNode.end(); ++it) {
            image_meta_.centers.emplace_back((float)(*it)["x"], (float)(*it)["y"]);
        }
        fs.release();
    }
};

TEST_F(LandoltTest, testImages)
{
    std::string file_names[][2] = { { "landolt-c.png", "landolt-c.yml" } };
    size_t images_count = sizeof(file_names) / sizeof(file_names[0]);
    
    for (size_t i = 0; i < images_count; ++i) {
        ASSERT_TRUE(publishImage(data_path_ + file_names[i][0], data_path_ + file_names[i][1]));

        size_t size = received_landolts_.size();
        EXPECT_EQ(size, received_centers_.size());
        EXPECT_EQ(size, received_radius_.size());
        EXPECT_EQ(size, image_meta_.angles.size());
        EXPECT_EQ(size, image_meta_.radius.size());
        EXPECT_EQ(size, image_meta_.centers.size());

        const double radiusError = 3.0;
        const double boundingError = 3.0;
        const double angleError = 1.0;

        for(size_t j = 0; j < size; j++) {
            bool hasFound = false;
            for(size_t k = 0; k < size; k++) {
                if(std::abs(image_meta_.centers[k].x - received_centers_[j].x) < boundingError &&
                   std::abs(image_meta_.centers[k].y - received_centers_[j].y) < boundingError &&
                   std::abs(image_meta_.radius[k] - received_radius_[j]) < radiusError &&
                   std::abs(image_meta_.angles[k] - received_landolts_[j]) < angleError) {
                    hasFound = true;
                    break;
                }
            }
            EXPECT_TRUE(hasFound);
        }
    }
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}