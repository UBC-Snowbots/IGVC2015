#include "generate_global_map.hpp"
#include <iostream>

int main(int argc, char **argv) {

  ros::init(argc, argv, "test_generate_global_map");

  ros::Time::init();

  std::cout << "hello world" << std::endl;

  ros::Rate loop_rate(5); //10hz loop rate

  while (ros::ok()){

    GenerateGlobalMap generateGlobalMap;

  //generateGlobalMap.testDoSomething();

  ros::spinOnce(); //ros spin is to ensure that ros threading does not leave subscribes un processed
  loop_rate.sleep();

  }

  return 0;

}

