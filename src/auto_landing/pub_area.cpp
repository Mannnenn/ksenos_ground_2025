#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"

class PolygonPublisher : public rclcpp::Node
{
public:
    PolygonPublisher()
        : Node("polygon_publisher")
    {
        // Define three rectangles
        rectangles_ = {
            createRectangle(0, 0, 42, -2),
            createRectangle(0, -2, 10, -26),
            createRectangle(0, -28, 42, -2),
            createRectangle(4, -10, 5, -0.9),
            createRectangle(1, -13, 9, -3),
            createRectangle(10, -2, 22, -26),
            createRectangle(20, -9, 5, -12),
            createRectangle(20, -9, 5, -2),
            createRectangle(21, -13, 3, -5),
            createRectangle(23, -20, 2, -1),
            createRectangle(32, -2, 10, -26),
            createRectangle(33, -16, 5, -0.9),
        };

        poles_ = {
            createPole(32, -2, 3),
            createPole(32, -9, 3),
        };

        // Create publishers for each rectangle
        for (size_t i = 0; i < rectangles_.size(); ++i)
        {
            auto publisher = this->create_publisher<geometry_msgs::msg::PolygonStamped>("polygon_" + std::to_string(i), 10);
            publishers_.push_back(publisher);
        }

        // Create publishers for each pole
        for (size_t i = 0; i < poles_.size(); ++i)
        {
            auto publisher_pole = this->create_publisher<geometry_msgs::msg::PolygonStamped>("pole_" + std::to_string(i), 10);
            publisher_pole_.push_back(publisher_pole);
        }

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PolygonPublisher::publishPolygons, this));
    }

private:
    void publishPolygons()
    {
        for (size_t i = 0; i < rectangles_.size(); ++i)
        {
            auto message = geometry_msgs::msg::PolygonStamped();
            message.header.frame_id = "origin";
            message.header.stamp = this->now();

            for (const auto &point : rectangles_[i])
            {
                message.polygon.points.push_back(point);
            }

            publishers_[i]->publish(message);
        }

        for (size_t i = 0; i < poles_.size(); ++i)
        {
            auto message = geometry_msgs::msg::PolygonStamped();
            message.header.frame_id = "origin";
            message.header.stamp = this->now();

            for (const auto &point : poles_[i])
            {
                message.polygon.points.push_back(point);
            }

            publisher_pole_[i]->publish(message);
        }

        count_++;
        if (count_ > 100)
        {
            rclcpp::shutdown();
        }
    }

    std::vector<geometry_msgs::msg::Point32> createRectangle(float x, float y, float width, float height)
    {
        std::vector<geometry_msgs::msg::Point32> rect;
        geometry_msgs::msg::Point32 p1, p2, p3, p4;

        p1.x = x;
        p1.y = y;
        p2.x = x + width;
        p2.y = y;
        p3.x = x + width;
        p3.y = y + height;
        p4.x = x;
        p4.y = y + height;

        rect.push_back(p1);
        rect.push_back(p2);
        rect.push_back(p3);
        rect.push_back(p4);
        rect.push_back(p1); // Close the rectangle

        return rect;
    }

    std::vector<geometry_msgs::msg::Point32> createPole(float x, float y, float hight)
    {
        std::vector<geometry_msgs::msg::Point32> pole;
        geometry_msgs::msg::Point32 p1, p2;

        p1.x = x;
        p1.y = y;
        p1.z = 0;
        p2.x = x;
        p2.y = y;
        p2.z = hight;

        pole.push_back(p1);
        pole.push_back(p2);

        return pole;
    }

    std::vector<std::vector<geometry_msgs::msg::Point32>> rectangles_;
    std::vector<std::vector<geometry_msgs::msg::Point32>> poles_;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr> publishers_;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr> publisher_pole_;

    rclcpp::TimerBase::SharedPtr timer_;

    int count_ = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PolygonPublisher>());
    rclcpp::shutdown();
    return 0;
}