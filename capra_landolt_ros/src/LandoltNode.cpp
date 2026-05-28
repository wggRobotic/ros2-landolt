#include <cmath>
#include <memory>
#include <vector>

#include "capra_landolt_msgs/msg/bounding_circles.hpp"
#include "capra_landolt_msgs/msg/landolts.hpp"
#include "capra_landolt_msgs/msg/point2f.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

namespace capra {

struct Gaps {
  std::vector<float> angles;
  std::vector<float> radius;
  std::vector<cv::Point2f> centers;
};

class LandoltNode : public rclcpp::Node {
public:
  explicit LandoltNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("landolt_detector", options) {
    this->declare_parameter<int>("queue_size", 5);
    this->declare_parameter<std::string>("camera_reading",
                                         "/camera_3d/rgb/image_raw");

    queue_size_ = this->get_parameter("queue_size").as_int();
    camera_reading_ = this->get_parameter("camera_reading").as_string();

    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s",
                camera_reading_.c_str());

    landolt_pub_ = this->create_publisher<capra_landolt_msgs::msg::Landolts>(
        "landolts", rclcpp::QoS(10));
    bounding_pub_ =
        this->create_publisher<capra_landolt_msgs::msg::BoundingCircles>(
            "boundings", rclcpp::QoS(10));
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "image", rclcpp::QoS(1));

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_reading_, rclcpp::QoS(queue_size_),
        std::bind(&LandoltNode::imageCb, this, std::placeholders::_1));
  }

private:
  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  int queue_size_{};
  std::string camera_reading_;

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<capra_landolt_msgs::msg::Landolts>::SharedPtr landolt_pub_;
  rclcpp::Publisher<capra_landolt_msgs::msg::BoundingCircles>::SharedPtr
      bounding_pub_;

  void imageCb(const sensor_msgs::msg::Image::SharedPtr image_msg) {
    cv_bridge::CvImageConstPtr img_ptr;
    try {
      img_ptr = cv_bridge::toCvShare(image_msg, "bgr8");
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    Gaps gaps;
    findLandoltGaps(img_ptr->image, gaps, 12, 0.8f, 10);

    // timestamp
    rclcpp::Time now = this->now();

    if (landolt_pub_->get_subscription_count() > 0) {
      capra_landolt_msgs::msg::Landolts landolts_msg;
      landolts_msg.angles = gaps.angles;
      landolts_msg.header.stamp = now;
      landolt_pub_->publish(landolts_msg);
    }

    if (bounding_pub_->get_subscription_count() > 0) {
      capra_landolt_msgs::msg::BoundingCircles boundings;
      boundings.header.stamp = now;
      boundings.radius = gaps.radius;

      std::vector<capra_landolt_msgs::msg::Point2f> centers;
      for (auto &i : gaps.centers) {
        capra_landolt_msgs::msg::Point2f center;
        center.x = i.x;
        center.y = i.y;
        centers.push_back(center);
      }
      boundings.centers = centers;
      bounding_pub_->publish(boundings);
    }

    if (image_pub_->get_subscription_count() > 0) {
      std_msgs::msg::Header hdr;
      hdr.stamp = now;
      cv_bridge::CvImage img_msg(hdr, sensor_msgs::image_encodings::BGR8,
                                 img_ptr->image);

      for (size_t i = 0; i < gaps.angles.size(); i++) {
        circle(img_msg.image, gaps.centers[i], static_cast<int>(gaps.radius[i]),
               cv::Scalar(0, 0, 1), 3);
      }
      // Speichert das aktuelle Bild im Home-Verzeichnis ab
      auto out = img_msg.toImageMsg();
      image_pub_->publish(*out);
    }
  }

  static float magnitudePoint(const cv::Point2f &diff) {
    return sqrtf(diff.dot(diff));
  }

  static cv::Point2f normalizePoint(const cv::Point2f &diff) {
    return diff / magnitudePoint(diff);
  }

  static float angleBetween(cv::Point2f origin, cv::Point2f dest) {
    float dot = origin.x * dest.x + origin.y * dest.y;
    float det = origin.x * dest.y - origin.y * dest.x;
    return atan2f(det, dot) * (float)(180.0 / M_PI) + 180;
  }

  void findLandoltGaps(const cv::Mat &imageRaw, Gaps &gaps, int minEdge,
                       float minRatioCircle, int minDepth) {
    cv::Mat thresholdMat;
    cvtColor(imageRaw, thresholdMat, cv::COLOR_BGR2GRAY);
    blur(thresholdMat, thresholdMat, cv::Size(3, 3));
    threshold(thresholdMat, thresholdMat, 140, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    findContours(thresholdMat, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE,
                 cv::Point(0, 0));

    for (auto &contour : contours) {

      if (contour.size() > static_cast<size_t>(minEdge)) {
        std::vector<cv::Point> hull;
        convexHull(contour, hull, true, true);
        double hullArea = cv::contourArea(hull);

        float contourRadius;
        cv::Point2f contourCenter;
        minEnclosingCircle(contour, contourCenter, contourRadius);
        double minArea = contourRadius * contourRadius * M_PI;

        if (hullArea / minArea > minRatioCircle) {
          std::vector<cv::Vec4i> defects;
          std::vector<int> hullsI;

          try {
            cv::convexHull(contour, hullsI, false, false);
            cv::convexityDefects(contour, hullsI, defects);
          } catch (const cv::Exception &e) {
            RCLCPP_WARN(this->get_logger(),
                        "OpenCV Fehler beim Berechnen der Defects: %s",
                        e.what());
            continue;
          }
          std::vector<cv::Vec4i> deepDefects;
          for (const auto &v : defects) {
            float depth = static_cast<float>(v[3]) / 256.0f;
            if (depth > minDepth) {
              deepDefects.push_back(v);
            }
          }

          if (deepDefects.size() == 1) {
            const cv::Vec4i &v = deepDefects[0];

            int startidx = v[0];
            int endidx = v[1];
            int faridx = v[2];

            std::vector<cv::Point> points;
            points.emplace_back(contour[startidx]);
            points.emplace_back(contour[endidx]);

            float defectRadius;
            cv::Point2f defectCenter;
            minEnclosingCircle(points, defectCenter, defectRadius);
            cv::Point2f dir =
                normalizePoint(cv::Point2f(contour[faridx]) - defectCenter);
            defectCenter += dir * defectRadius;

            float defectAngle = angleBetween(dir, cv::Point2f(1, 0));

            gaps.angles.push_back(defectAngle);
            gaps.radius.push_back(defectRadius);
            gaps.centers.push_back(defectCenter);
          }
        }
      }
    }
  }
};

} // namespace capra

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<capra::LandoltNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}