#include "startup_trace.hpp"
#include <csignal>
#include <iostream>
#include <sys/resource.h>

int main(int argc,char **argv) {
  if(argc!=2)return 2;
  std::signal(SIGXFSZ,SIG_IGN);
  const rlimit limit{1024,1024};
  if(setrlimit(RLIMIT_FSIZE,&limit)!=0)return 2;
  offline_handoff::Trace trace(argv[1],"failed");
  try {for(int row=0;row<300;++row)trace.record("actual_disk_limit",row);}
  catch(const std::runtime_error&) {} // Expected if the worker sees failure first.
  trace.close();
  if(!trace.closed()||trace.flush_ok()) {
    std::cerr<<"real disk limit must not report a successful trace flush\n";
    return 1;
  }
  std::cout<<"real trace disk failure correctly reported\n";
}
