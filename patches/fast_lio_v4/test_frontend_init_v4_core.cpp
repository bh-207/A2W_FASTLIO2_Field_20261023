#include "frontend_init_v4.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
using namespace frontend_init_v4;

Sample sample(std::int64_t ns, Vec3 acc={0.,0.,9.81}, Vec3 gyro={0.,0.,0.})
{
  Sample s; s.stamp_ns=ns; s.acc=acc; s.gyro=gyro; s.frame_id="imu";
  s.gyro_unit="rad/s"; s.accel_unit="m/s^2"; return s;
}
GateConfig config()
{
  GateConfig c; c.enabled=true; c.minimum_samples=5; c.minimum_duration_s=.4;
  c.maximum_gap_s=.11; c.accel_variance_limit=.001; c.gyro_variance_limit=.00001;
  c.maximum_direction_change_rad=.02; return c;
}
void feed_static(Gate &gate, std::int64_t start=0, Vec3 acc={0.,0.,9.81})
{for(int i=0;i<5;++i)gate.feed(sample(start+i*100000000LL,acc));}

int main(int argc,char **argv)
{
  const std::string name=argc>1?argv[1]:"";
  if(name=="inclined") {
    Gate gate(config());feed_static(gate,0,{0.,4.905,8.495709211125343});
    assert(gate.ready());assert(gate.stats().count==5);
    assert(std::abs(gate.stats().mean_acc[1]-4.905)<1e-12);
    assert(gate.stats().first_stamp_ns==0&&gate.stats().last_stamp_ns==400000000);
  } else if(name=="count_duration") {
    auto c=config();c.minimum_samples=3;Gate gate(c);
    gate.feed(sample(0));gate.feed(sample(100000000));gate.feed(sample(200000000));
    assert(!gate.ready());gate.feed(sample(300000000));gate.feed(sample(400000000));assert(gate.ready());
    c.minimum_samples=8;Gate other(c);feed_static(other);assert(!other.ready());
  } else if(name=="motion_reset") {
    Gate gate(config());feed_static(gate);assert(gate.ready());
    gate.feed(sample(500000000,{0,0,9.81},{0,0,1}));assert(!gate.ready());
    assert(gate.stats().reset_count>0);feed_static(gate,600000000);assert(gate.ready());
    assert(gate.stats().first_stamp_ns==600000000);
  } else if(name=="late_bad_group") {
    Gate gate(config());feed_static(gate);assert(gate.ready());
    gate.feed(sample(500000000,{0,0,11.5}));assert(!gate.ready());
  } else if(name=="timing_reset") {
    for(int kind=0;kind<3;++kind) {
      Gate gate(config());feed_static(gate);assert(gate.ready());
      gate.feed(sample(kind==0?400000000:kind==1?300000000:900000000));
      assert(!gate.ready());assert(gate.stats().reset_count==1);
    }
  } else if(name=="variance_direction") {
    auto c=config();c.accel_variance_limit=.000001;Gate a(c);
    a.feed(sample(0));a.feed(sample(100000000,{.01,0,9.81}));assert(a.stats().reset_count>0);
    c=config();c.gyro_variance_limit=.00000001;Gate b(c);
    b.feed(sample(0));b.feed(sample(100000000,{0,0,9.81},{.001,0,0}));assert(b.stats().reset_count>0);
    c=config();Gate d(c);d.feed(sample(0));d.feed(sample(100000000,{.5,0,9.797}));assert(d.stats().reset_count>0);
  } else if(name=="invalid") {
    auto c=config();c.minimum_duration_s=-1;bool threw=false;
    try {Gate bad(c);}catch(const std::invalid_argument&){threw=true;}assert(threw);
    Gate gate(config());gate.feed(sample(0));gate.feed(sample(100000000,{0,0,std::numeric_limits<double>::quiet_NaN()}));
    assert(!gate.ready());assert(gate.stats().reset_count>0);
    auto s=sample(200000000);s.gyro_unit="deg/s";threw=false;
    try{gate.feed(s);}catch(const std::invalid_argument&){threw=true;}assert(threw);
  } else if(name=="units_once") {
    InputConfig c; c.enabled=true;c.gyro_unit="rad/s";c.accel_unit="m/s^2";c.frame_id="imu";
    c.gyro_evidence="synthetic-generator:rad";c.accel_evidence="synthetic-generator:SI";c.time_evidence="same_clock_known_offset";
    InputAdapter adapter(c);auto s=adapter.adapt(sample(1000000000,{1,2,9.81},{.01,.02,.03}));
    assert(s.acc[0]==1&&s.gyro[2]==.03);assert(s.conversion_count==1&&s.time_adjustment_count==1);
    bool threw=false;try{adapter.adapt(s);}catch(const std::invalid_argument&){threw=true;}assert(threw);
    c.gyro_unit="deg/s";c.accel_unit="g";InputAdapter convert(c);auto raw=sample(1000000000,{0,0,1},{0,0,180});raw.gyro_unit="deg/s";raw.accel_unit="g";
    auto out=convert.adapt(raw);assert(std::abs(out.gyro[2]-3.141592653589793)<1e-12);assert(std::abs(out.acc[2]-9.81)<1e-12);
  } else if(name=="offset_sign") {
    for(std::int64_t offset:{100000000LL,-100000000LL}) {
      InputConfig c;c.enabled=true;c.gyro_unit="rad/s";c.accel_unit="m/s^2";c.frame_id="imu";
      c.gyro_evidence="known";c.accel_evidence="known";c.time_evidence="fixture";c.imu_minus_lidar_offset_ns=offset;
      auto out=InputAdapter(c).adapt(sample(1000000000));assert(out.stamp_ns==1000000000-offset);
    }
  } else if(name=="unknown_mismatch") {
    InputConfig c;c.enabled=true;bool threw=false;
    try{InputAdapter a(c);}catch(const std::invalid_argument&){threw=true;}assert(threw);
    c.gyro_unit="rad/s";c.accel_unit="m/s^2";c.frame_id="imu";c.gyro_evidence="known";c.accel_evidence="known";c.time_evidence="known";
    InputAdapter a(c);auto s=sample(1000000000);s.gyro_unit="deg/s";threw=false;
    try{a.adapt(s);}catch(const std::invalid_argument&){threw=true;}assert(threw);
    s=sample(1000000000);s.frame_id="other";threw=false;try{a.adapt(s);}catch(const std::invalid_argument&){threw=true;}assert(threw);
  } else if(name=="off_default") {
    Gate gate(GateConfig{});assert(!gate.enabled());assert(!gate.ready());
    InputAdapter a(InputConfig{});auto s=sample(123);auto out=a.adapt(s);
    assert(out.stamp_ns==123&&out.conversion_count==0&&out.time_adjustment_count==0);
  } else return 3;
  std::cout<<name<<" PASS\n";
}
