#include <iostream>

#include "app/application.hpp"
#include "commander/command_line.hpp"

int main(int argc, char** argv) {
  gripper::app::Application application;
  return application.run(gripper::commander::collectArgs(argc, argv), std::cin,
                         std::cout);
}
