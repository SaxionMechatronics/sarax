// Every shipped ImpedanceLaw must be creatable through pluginlib, the way the controller loads it
// (plugin XML, library path and registration), not only by direct construction as in the law tests.
#include <gtest/gtest.h>

#include <pluginlib/class_loader.hpp>

#include "sarax_impedance_controller/impedance_law.hpp"

TEST(LawPlugins, AllShippedLawsAreCreatable)
{
  pluginlib::ClassLoader<sarax_impedance_controller::ImpedanceLaw> loader(
    "sarax_impedance_controller", "sarax_impedance_controller::ImpedanceLaw");
  for (const char * name : {
      "sarax_impedance_controller::PDLaw", "sarax_impedance_controller::PDGravityLaw",
      "sarax_impedance_controller::PDPinocchioGravityLaw"})
  {
    EXPECT_NO_THROW(loader.createSharedInstance(name)) << name;
  }
}
