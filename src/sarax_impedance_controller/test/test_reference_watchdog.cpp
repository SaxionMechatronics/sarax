#include <gtest/gtest.h>

#include "sarax_impedance_controller/reference_watchdog.hpp"

using sarax_impedance_controller::ReferenceWatchdog;

TEST(ReferenceWatchdog, NoReferenceIsNeverActiveNorExpires)
{
  ReferenceWatchdog w(0.5);
  EXPECT_FALSE(w.update(false, false, false, 10.0));
  EXPECT_FALSE(w.expired());
  EXPECT_FALSE(w.active(false));
}

TEST(ReferenceWatchdog, StreamedReferenceStaysActive)
{
  ReferenceWatchdog w(0.5);
  for (int k = 0; k <= 100; ++k) {   // 20 Hz for 5 s, checked every cycle
    const double t = 0.05 * k;
    EXPECT_FALSE(w.update(true, true, false, t));
    EXPECT_TRUE(w.active(true));
  }
}

TEST(ReferenceWatchdog, SilentReferenceExpiresOnceAfterTimeout)
{
  ReferenceWatchdog w(0.5);
  EXPECT_FALSE(w.update(true, true, false, 1.0));    // reference arrives
  EXPECT_FALSE(w.update(true, false, false, 1.5));   // exactly at the timeout: still live
  EXPECT_TRUE(w.active(true));
  EXPECT_TRUE(w.update(true, false, false, 1.504));  // first cycle past it: expires
  EXPECT_TRUE(w.expired());
  EXPECT_FALSE(w.active(true));
  EXPECT_FALSE(w.update(true, false, false, 2.0));   // reported once, stays expired
  EXPECT_TRUE(w.expired());
}

TEST(ReferenceWatchdog, NewReferenceRearmsAfterExpiry)
{
  ReferenceWatchdog w(0.5);
  w.update(true, true, false, 0.0);
  ASSERT_TRUE(w.update(true, false, false, 1.0));
  EXPECT_FALSE(w.update(true, true, false, 1.1));    // publisher is back
  EXPECT_TRUE(w.active(true));
  EXPECT_TRUE(w.update(true, false, false, 1.7));    // and silent again
}

TEST(ReferenceWatchdog, PersistentReferenceNeverExpires)
{
  ReferenceWatchdog w(0.5);
  w.update(true, true, true, 0.0);
  EXPECT_FALSE(w.update(true, false, true, 100.0));
  EXPECT_TRUE(w.active(true));
}

TEST(ReferenceWatchdog, ZeroTimeoutDisables)
{
  ReferenceWatchdog w(0.0);
  w.update(true, true, false, 0.0);
  EXPECT_FALSE(w.update(true, false, false, 100.0));
  EXPECT_TRUE(w.active(true));
}

TEST(ReferenceWatchdog, ResetForgetsExpiry)
{
  ReferenceWatchdog w(0.5);
  w.update(true, true, false, 0.0);
  ASSERT_TRUE(w.update(true, false, false, 1.0));
  w.reset();
  EXPECT_FALSE(w.expired());
  EXPECT_FALSE(w.active(false));
}
