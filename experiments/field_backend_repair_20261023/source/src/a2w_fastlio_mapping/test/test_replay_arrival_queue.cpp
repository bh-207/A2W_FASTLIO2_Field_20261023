#include "replay_arrival_queue.hpp"
#include <gtest/gtest.h>
using replay_queue::ArrivalQueue;
TEST(ReplayArrivalQueue, RejectsBadStampsAndUnboundedCapacity) {
  EXPECT_THROW(ArrivalQueue({},32),std::invalid_argument);
  EXPECT_THROW(ArrivalQueue({0,0},32),std::invalid_argument);
  EXPECT_THROW(ArrivalQueue({2,1},32),std::invalid_argument);
  EXPECT_THROW(ArrivalQueue({-1,1},32),std::invalid_argument);
  EXPECT_THROW(ArrivalQueue({0,1},0),std::invalid_argument);
}
TEST(ReplayArrivalQueue, RealProducerDeliversTimestampOrderWithoutChangingInput) {
  const std::vector<std::int64_t> stamps{1700000000000000000LL,1700000000002000000LL,1700000000004000000LL};
  ArrivalQueue q(stamps,32);
  for (std::size_t i=0;i<3;++i) {EXPECT_EQ(q.pop(),i);}
  q.waitDone(); auto s=q.stats();
  EXPECT_EQ(s.produced,3U); EXPECT_EQ(s.consumed,3U); EXPECT_FALSE(s.overflow);
  auto e=q.events(); ASSERT_EQ(e.size(),6U);
  std::size_t arrivals=0;
  for (auto v:e) {EXPECT_LE(v.waiting,32U); EXPECT_GE(v.wall_ns,v.due_ns);
    if (std::string(v.action)=="arrival") {++arrivals;}}
  EXPECT_EQ(arrivals,3U); EXPECT_EQ(stamps.front(),1700000000000000000LL);
}
TEST(ReplayArrivalQueue, ProducerDetectsOverflowWithoutEndpointPolling) {
  ArrivalQueue q({0,1,2},2); q.waitDone();
  auto s=q.stats(); EXPECT_TRUE(s.overflow); EXPECT_EQ(s.produced,2U);
  EXPECT_EQ(s.max_waiting,2U); EXPECT_THROW(q.pop(),std::runtime_error);
  auto e=q.events(); ASSERT_FALSE(e.empty());
  EXPECT_EQ(std::string(e.back().action),"overflow"); EXPECT_EQ(e.back().id,2U);
  for (auto v:e) {EXPECT_LE(v.waiting,2U);}
}
TEST(ReplayArrivalQueue, CancellationWakesLongWaitAndJoinsOnlyOwnThread) {
  auto start=std::chrono::steady_clock::now();
  {ArrivalQueue q({0,60000000000LL},32); EXPECT_EQ(q.pop(),0U); q.cancel(); q.waitDone();}
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count(),.5);
}
