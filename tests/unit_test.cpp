#include <gtest/gtest.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tuple>
#include <Robot_link.cpp>

TEST(angle_test, aux_rpy)
{
    // make transformd stamped
    geometry_msgs::msg::TransformStamped t;
    t.transform.translation.x = 0;
    t.transform.translation.y = 0;
    t.transform.translation.z = 0;
    t.transform.rotation.x = 0;
    t.transform.rotation.y = 0;
    t.transform.rotation.z = 0;
    t.transform.rotation.w = 1;

    std::tuple<double, double, double> res = path_follow::GetRPY(t);

    EXPECT_EQ(0.0, std::get<0>(res));
    EXPECT_EQ(0.0, std::get<1>(res));
    EXPECT_EQ(0.0, std::get<2>(res));
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}