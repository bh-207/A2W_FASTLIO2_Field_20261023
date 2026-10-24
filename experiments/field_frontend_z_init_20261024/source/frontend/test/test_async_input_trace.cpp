#include <gtest/gtest.h>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <vector>
#include <csignal>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>
#include "frontend_async_input_trace.hpp"
#include "frontend_input_integrity.hpp"
namespace f=frontend_input_csv;
std::string path(const char*tag){
 static const std::string directory=[] {char pattern[]="/tmp/a2w_input_csv_tests_XXXXXX";const auto* p=mkdtemp(pattern);if(!p)throw std::runtime_error("test temporary directory failed");return std::string(p);}();
 static unsigned seq=0;return directory+"/"+tag+"_"+std::to_string(++seq);
}
f::Row row(unsigned i){f::Row r;r.raw_stamp_ns=1792724421199894905LL+i;r.aligned_stamp_ns=r.raw_stamp_ns-123;r.callback_id=i;r.input_frame="imu_frame";r.gyro_unit="rad/s";r.accel_unit="m/s^2";r.values[0]=.12345678901234566;return r;}
std::string read(const std::string&p){std::ifstream in(p);return {std::istreambuf_iterator<char>(in),{}};}
TEST(AsyncInputTrace, WorkerWritesFIFOExactIntegerNsAndPrecisionThenCloses) {
 const auto p=path("fifo");std::vector<unsigned long> ids;
 f::AsyncWriter w({p,16},[&](const auto&r){ids.push_back(r.callback_id);});
 for(unsigned i=0;i<3;++i)ASSERT_TRUE(w.enqueue(row(i)));
 w.close();EXPECT_TRUE(w.stats().healthy());EXPECT_EQ(w.stats().written,3U);
 EXPECT_EQ(ids,(std::vector<unsigned long>{0,1,2}));
 const auto csv=read(p);EXPECT_EQ(csv.substr(0,std::string(f::header()).size()),f::header());
 EXPECT_NE(csv.find("adapted,1792724421199894905,1792724421199894782,imu_frame,rad/s,m/s^2,0,0,0.12345678901234566"),std::string::npos);
 EXPECT_NE(read(p+".status.json").find("\"closed\":true"),std::string::npos);
}
TEST(AsyncInputTrace, BlockedObserverCannotBlockProducerAndOverflowRemainsVisible) {
 const auto p=path("overflow");std::mutex mutex;std::condition_variable ready;bool entered=false,release=false;
 f::AsyncWriter w({p,2},[&](const auto&r){if(r.callback_id==0){std::unique_lock<std::mutex> lock(mutex);entered=true;ready.notify_all();ready.wait(lock,[&]{return release;});}});
 ASSERT_TRUE(w.enqueue(row(0)));
 bool observed;
 {std::unique_lock<std::mutex> lock(mutex);observed=ready.wait_for(lock,std::chrono::seconds(2),[&]{return entered;});if(!observed)release=true;}
 ready.notify_all();ASSERT_TRUE(observed);
 const auto start=std::chrono::steady_clock::now();
 const bool a=w.enqueue(row(1)),b=w.enqueue(row(2)),c=w.enqueue(row(3));
 const auto elapsed=std::chrono::steady_clock::now()-start;
 {std::lock_guard<std::mutex> lock(mutex);release=true;}ready.notify_all();w.close();
 EXPECT_TRUE(a);EXPECT_TRUE(b);EXPECT_FALSE(c);EXPECT_LT(elapsed,std::chrono::milliseconds(100));
 EXPECT_EQ(w.stats().overflow,1U);EXPECT_EQ(w.stats().written,3U);EXPECT_FALSE(w.stats().healthy());
}
TEST(AsyncInputTrace, ImmediateStopDrainsEveryAcceptedRowAndRejectsNewIntake) {
 const auto p=path("stop");f::AsyncWriter w({p,512});
 for(unsigned i=0;i<200;++i)ASSERT_TRUE(w.enqueue(row(i)));
 w.close();w.close();EXPECT_TRUE(w.stats().healthy());EXPECT_EQ(w.stats().enqueued,200U);EXPECT_EQ(w.stats().written,200U);
 EXPECT_FALSE(w.enqueue(row(201)));const auto csv=read(p);EXPECT_EQ(std::count(csv.begin(),csv.end(),'\n'),201);
}
TEST(AsyncInputTrace, RealWriteFailureIsVisibleAndFaultStatusProducesNonzeroExit) {
 const auto p=path("io");const pid_t child=fork();ASSERT_GE(child,0);
 if(child==0){std::signal(SIGXFSZ,SIG_IGN);struct rlimit limit{512,512};if(setrlimit(RLIMIT_FSIZE,&limit))_exit(9);
  try{f::AsyncWriter w({p,64});for(unsigned i=0;i<32;++i)w.enqueue(row(i));w.close();_exit(w.stats().io_errors&&!w.stats().healthy()?2:0);}catch(...){_exit(3);}}
 int status=0;ASSERT_EQ(waitpid(child,&status,0),child);ASSERT_TRUE(WIFEXITED(status));EXPECT_EQ(WEXITSTATUS(status),2);
}

TEST(AsyncInputTrace, ProcessingStatusReportsActualModeAfterWriterStops) {
 const auto p=path("mode");frontend_integrity::Trace trace(p,16);
 trace.set_input_csv_mode("async_spsc_bounded_flush_worker");
 ASSERT_TRUE(trace.close({}));
 EXPECT_NE(read(p+"/frontend_processing_status.json").find("\"input_csv_mode\":\"async_spsc_bounded_flush_worker\""),std::string::npos);
}
TEST(AsyncInputTrace, RingWrapPreservesFIFOAndLoggedIsActualFlushedWrite) {
 const auto p=path("wrap");std::mutex mutex;std::condition_variable ready;unsigned count=0;bool actual=true;
 f::AsyncWriter w({p,2},[&](const auto&r){
  const auto csv=read(p);if(csv.find(std::to_string(r.raw_stamp_ns))==std::string::npos)actual=false;
  std::lock_guard<std::mutex> lock(mutex);if(r.callback_id!=count)actual=false;++count;ready.notify_all();
 });
 for(unsigned i=0;i<100;++i){ASSERT_TRUE(w.enqueue(row(i)));std::unique_lock<std::mutex> lock(mutex);ASSERT_TRUE(ready.wait_for(lock,std::chrono::seconds(2),[&]{return count==i+1;}));}
 w.close();EXPECT_TRUE(actual);EXPECT_TRUE(w.stats().healthy());EXPECT_EQ(w.stats().written,100U);
}
TEST(AsyncInputTrace, RejectedRowPreservesMissingColumnsAndDoesNotClaimAdaptedLogged) {
 const auto p=path("rejected");unsigned adapted=0;f::AsyncWriter w({p,4},[&](const auto&r){if(r.adapted)++adapted;});
 auto r=row(0);r.adapted=false;ASSERT_TRUE(w.enqueue(std::move(r)));w.close();
 EXPECT_TRUE(w.stats().healthy());EXPECT_EQ(adapted,0U);
 EXPECT_NE(read(p).find("rejected,1792724421199894905,,imu_frame,rad/s,m/s^2,0,0,0.12345678901234566,0,0,0,0,0,,,,,,\n"),std::string::npos);
}
TEST(AsyncInputTrace, RefusesOverwriteAndDrainsBeforeIntegrityTraceCloses) {
 const auto existing=path("existing");{std::ofstream out(existing);out<<"sentinel";}
 EXPECT_THROW((f::AsyncWriter({existing,4})),std::runtime_error);EXPECT_EQ(read(existing),"sentinel");
 const auto p=path("join");frontend_integrity::Trace trace(p+"_trace",32);
 f::AsyncWriter w({p,16},[&](const auto&r){trace.record(frontend_integrity::Stage::ImuLogged,r.aligned_stamp_ns,r.callback_id);});
 for(unsigned i=0;i<10;++i)ASSERT_TRUE(w.enqueue(row(i)));
 w.close();
 EXPECT_EQ(trace.count(frontend_integrity::Stage::ImuLogged),10U);EXPECT_TRUE(trace.close({}));EXPECT_TRUE(w.stats().healthy());
}
