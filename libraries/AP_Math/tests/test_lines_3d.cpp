#include <AP_gtest.h>

#include <AP_Math/vector2.h>
#include <AP_Math/vector3.h>
#include <AP_Math/AP_Math.h>

TEST(Lines3dTests, ClosestDistBetweenLinePoint)
{
    const float dist = Vector3f::closest_distance_between_line_and_point(Vector3f{}, Vector3f{0.0f, 10.0f, 10.0f}, Vector3f{0.0f, 5.0f, 5.0f});
    EXPECT_FLOAT_EQ(dist, 0.0f);
}

AP_GTEST_MAIN()
