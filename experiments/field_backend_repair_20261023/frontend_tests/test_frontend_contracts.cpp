#include "frontend_lifecycle.hpp"
#include "pose_covariance.hpp"
#include "startup_trace.hpp"
#include <Eigen/Geometry>
#include "IKFoM_toolkit/mtk/types/SOn.hpp"
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>

static void require(bool ok, const char *message) {
  if (!ok) throw std::runtime_error(message);
}

int main(int argc, char **argv) {
  require(argc == 2, "provide temporary fixture directory");
  frontend_contract::Lifecycle life;
  require(!life.accepts_work(), "startup cannot accept callbacks");
  life.run();
  require(life.accepts_work(), "released running session accepts callbacks");
  bool caught = false;
  try { life.require_outputs(false, "missing /Odometry required observer"); }
  catch (const std::runtime_error &) { caught = true; }
  require(caught, "missing required running identity must fail");
  life.request_stop("signal", 2);
  life.require_outputs(false, "disconnect during stop");
  require(!life.accepts_work(), "no new callbacks after stop request");
  require(life.exit_code() == 0 && life.signal() == 2, "SIGINT is a graceful process exit");
  life.fail("a real callback error");
  life.request_stop("second signal", 15);
  require(life.exit_code() == 2 && life.error() == "a real callback error", "later stop cannot hide failure");
  life.stopped();
  require(life.phase_name() == std::string("STOPPED"), "closed session marks stopped");

  Eigen::Matrix<double, 23, 23> p = Eigen::Matrix<double, 23, 23>::Identity();
  p.diagonal().head<6>() << 1., 2., 3., 4., 5., 6.;
  p.block<3, 3>(0, 3) << .1, .2, .3, .4, .5, .6, .7, .8, .9;
  p.block<3, 3>(3, 0) = p.block<3, 3>(0, 3).transpose();
  Eigen::Matrix3d rotation;
  rotation << 0., -1., 0., 1., 0., 0., 0., 0., 1.;
  const auto covariance = frontend_contract::pose_covariance_world(p, rotation);
  const double expected[36] = {
    1.,0.,0.,-.2,.1,.3,
    0.,2.,0.,-.5,.4,.6,
    0.,0.,3.,-.8,.7,.9,
    -.2,-.5,-.8,5.,0.,0.,
    .1,.4,.7,0.,4.,0.,
    .3,.6,.9,0.,0.,6.};
  for (std::size_t k=0;k<36;++k)
    require(std::abs(covariance[k]-expected[k])<1e-12, "anisotropic/cross covariance fixed-world conversion mismatch");
  const auto identity_covariance = frontend_contract::pose_covariance_world(p, Eigen::Matrix3d::Identity());
  require(identity_covariance[0]==1. && identity_covariance[21]==4. && identity_covariance[3]==.1,
    "IKFoM position-first order cannot be swapped");
  // Exercise the actual vendored SO3::boxplus, rather than relying only on
  // an assumed tangent convention. The fixed-world rotation error is R*d.
  MTK::SO3<double> perturbed(rotation);
  Eigen::Vector3d delta(1e-7,2e-7,-3e-7);
  perturbed.boxplus(delta);
  const Eigen::AngleAxisd fixed_error(perturbed.toRotationMatrix()*rotation.transpose());
  require((fixed_error.axis()*fixed_error.angle()-rotation*delta).norm()<1e-12,
    "vendored IKFoM right tangent must map to fixed-world rotation by R");

  const auto root = std::filesystem::path(argv[1]);
  std::filesystem::create_directories(root/"capture/run");
  std::filesystem::create_directories(root/"capture_sibling");
  require(offline_handoff::capture_directory(root/"capture", root/"capture/run") == std::filesystem::canonical(root/"capture/run"),
    "explicit capture root must allow isolated session descendants");
  caught=false;
  try { (void)offline_handoff::capture_directory(root/"capture", root/"capture_sibling"); }
  catch (const std::invalid_argument &) { caught=true; }
  require(caught, "sibling prefix must not bypass capture root");
  std::filesystem::create_directory_symlink(root/"capture_sibling", root/"capture/escape");
  caught=false;
  try { (void)offline_handoff::capture_directory(root/"capture", root/"capture/escape"); }
  catch (const std::invalid_argument &) { caught=true; }
  require(caught, "symlink escape must not bypass capture root");

  offline_handoff::trace=std::make_unique<offline_handoff::Trace>(root/"capture/run","frontend");
  for(int row=0;row<500;++row)offline_handoff::event("test_callback_completed",row);
  offline_handoff::trace->close();
  offline_handoff::trace->close();
  require(offline_handoff::trace->closed()&&offline_handoff::trace->flush_ok(),
    "graceful close must drain every queued row and be idempotent");
  offline_handoff::write_exit_status(life,true,true,true);
  std::cout << "lifecycle, pose covariance, capture root contracts passed\n";
}
