//
// Created by dtrimoul on 1/15/19.
//

#include <jsk_moveit_object_synchroniser/ObjectTopicHandler.h>
#include "gtest/gtest.h"
#include "memory"

GTEST_API_ int main(int argc, char **argv) {
    printf("Running main() from gtest_main.cc\n");
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_topic");
    return RUN_ALL_TESTS();
}


class jsk_moveit_Test : public ::testing::Test {
private:


public:

    std::map<std::string, moveit_msgs::CollisionObject> map_test;
    std::unique_ptr<ObjectTopicHandler> obj;

    void SetUp(){
        ros::NodeHandle nh;
        std::string name("/object_bb/green_cube");
        obj.reset( new ObjectTopicHandler(nh, name, map_test ) );
    }
};

TEST_F(jsk_moveit_Test, constructor_Test) {
    ASSERT_EQ( obj->get_topicName() , "/object_bb/green_cube");
}

TEST_F(jsk_moveit_Test, objName_Test) {
    ASSERT_EQ( obj->getObjectName() , "green_cube");
}


TEST_F(jsk_moveit_Test, generatePoint_Test) {
    geometry_msgs::Point p;
    p.x = 1;
    p.y = 2;
    p.z = 3;
    geometry_msgs::Point test = obj->generatePoint(1,2,3);

    ASSERT_EQ( p.x, test.x);
    ASSERT_EQ( p.y, test.y);
    ASSERT_EQ( p.z, test.z);

}
