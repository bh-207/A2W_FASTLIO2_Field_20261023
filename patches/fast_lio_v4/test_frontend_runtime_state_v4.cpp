// Real bounded writer behavior: no ROS, no mocked filesystem/threads.
#include <iostream>
#if __has_include("frontend_runtime_state_v4.hpp")
#include "frontend_runtime_state_v4.hpp"
#include <csignal>
#include <sys/resource.h>
#include <string>
#include <stdexcept>
#include <fstream>
using namespace frontend_runtime_v4;
void check(bool ok,const char *why){if(!ok)throw std::runtime_error(why);}
int main(int argc,char **argv){try{
  check(argc==3,"CASE NEW_CSV");const std::string mode=argv[1],path=argv[2];
  Config config;config.enabled=mode!="disabled";config.path=path;
  config.source_provenance="synthetic";config.capacity=mode=="overflow"?1:64;
  if(mode=="disabled"){config.capacity=0;config.source_provenance="unknown";}
  Writer writer(config);
  Row row;row.stage=Stage::PostLidarUpdate;row.scan_begin_ns=1000000001;
  row.scan_end_ns=1099999999;row.state_stamp_ns=1099999999;
  row.state={1,2,3,4,5,6,0,0,0,1,0,-4.9,-8.5,.01,.02,.03,.1,.2,.3,.06,-.02,.04,0,0,0,1};
  if(mode=="live_init"){
    row.stage=Stage::InitCommit;
    while(!writer.push(row))std::this_thread::yield();
    bool seen=false;
    for(int i=0;i<100&&!seen;++i){std::ifstream input(path);std::string line;
      while(std::getline(input,line))if(line.find(",INIT_COMMIT,")!=std::string::npos)seen=true;
      if(!seen)std::this_thread::sleep_for(std::chrono::milliseconds(5));}
    check(seen,"INIT_COMMIT not visible before close; readiness would stall");
  }else if(mode=="write_error"){
    std::signal(SIGXFSZ,SIG_IGN);rlimit limit{1024,1024};check(setrlimit(RLIMIT_FSIZE,&limit)==0,"setrlimit");
    for(int i=0;i<10000;++i){row.state_stamp_ns+=1;writer.push(row);}
  }else if(mode=="overflow"){
    for(int i=0;i<200000;++i){row.state_stamp_ns+=1;writer.push(row);}
  }else{
    if(mode=="disabled"){writer.push(row);}
    else {while(!writer.push(row))std::this_thread::yield();}
    row.state[0]=999;row.stage=Stage::InitCommit;row.state_stamp_ns=1095000000;
    if(mode=="disabled"){writer.push(row);}
    else {while(!writer.push(row))std::this_thread::yield();}
  }
  writer.close();writer.close();const auto stats=writer.stats();
  if(mode=="disabled")check(stats.attempted==0,"disabled changed counters");
  else{
    check(stats.closed,"writer not closed");
    check(stats.attempted==stats.enqueued+stats.dropped,"drop accounting");
    if(mode=="write_error")check(stats.write_errors>0&&!stats.flush_ok,"missing real IO failure");
    else check(stats.written==stats.enqueued&&stats.flush_ok,"drain/flush lost data");
    if(mode=="overflow")check(stats.dropped>0,"overflow was hidden");
    check(!writer.push(row),"post-close accepted a row");
  }
  std::cout<<"PASS "<<mode<<'\n';return 0;
}catch(const std::exception &error){std::cerr<<error.what()<<'\n';return 1;}}
#else
int main(){std::cerr<<"RUNTIME STATE LOG MISSING: no bounded writer, post-update state evidence unavailable\n";return 1;}
#endif
