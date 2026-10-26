// Standalone offline integrator. No ROS, DDS, FAST-LIO state, or map mutation.
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
using Eigen::Matrix3d;using Eigen::Vector3d;
using M21=Eigen::Matrix<double,21,21>;
using V21=Eigen::Matrix<double,21,Eigen::Dynamic>;
static Matrix3d skew(const Vector3d& v){Matrix3d a;a<<0,-v.z(),v.y(),v.z(),0,-v.x(),-v.y(),v.x(),0;return a;}
static Matrix3d exp3(const Vector3d& v){double n=v.norm();return n<1e-12?Matrix3d(Matrix3d::Identity()+skew(v)):Eigen::AngleAxisd(n,v/n).toRotationMatrix();}
static Matrix3d jr(const Vector3d& v){double n=v.norm();auto s=skew(v);if(n<1e-5)return Matrix3d::Identity()-.5*s+s*s/6.;return Matrix3d::Identity()-(1-std::cos(n))/(n*n)*s+(n-std::sin(n))/(n*n*n)*s*s;}
extern "C" int integrate(int m,const double* it,const double* ar,const double* wr,
 int n,const double* times,const double* par,double alpha,const double* first,
 double sa,double sw,int covariance,double* output,double* cposes,double* cross,double* lastp){
 if(m<2||n<2||it[0]>times[0]||it[m-1]<times[n-1])return 1;
 for(int k=1;k<m;k++)if(it[k]<=it[k-1]||it[k]-it[k-1]>.05)return 2;
 for(int k=1;k<n;k++)if(times[k]<=times[k-1])return 3;
 Vector3d p=Eigen::Map<const Vector3d>(first),v=Eigen::Map<const Vector3d>(par+9);
 const Vector3d g=Eigen::Map<const Vector3d>(par),ba=Eigen::Map<const Vector3d>(par+3),bg=Eigen::Map<const Vector3d>(par+6);
 Matrix3d r=Matrix3d::Identity();double now=times[0];int j=0;
 while(j+1<m-1&&it[j+1]<=now)j++;
 M21 pc=M21::Zero();Eigen::Matrix<double,6,6> q=Eigen::Matrix<double,6,6>::Zero();
 q.diagonal()<<sa*sa,sa*sa,sa*sa,sw*sw,sw*sw,sw*sw;
 pc.block<6,6>(9,9)=q;pc.block<6,6>(15,15)=q;
 V21 xp=V21::Zero(21,6*n);
 Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>> cp(cposes,6*n,6*n);
 if(covariance)cp.setZero();
 for(int pose=0;pose<n;pose++){
  while(now<times[pose]-1e-12){
   if(j+1>=m)return 4;
   double stop=std::min(times[pose],it[j+1]);double dt=stop-now;
   if(dt<=1e-12){j++;continue;}
   double f=(.5*(now+stop)-it[j])/(it[j+1]-it[j]);
   Vector3d a=alpha*((1-f)*Eigen::Map<const Vector3d>(ar+3*j)+f*Eigen::Map<const Vector3d>(ar+3*(j+1)))-ba;
   Vector3d w=(1-f)*Eigen::Map<const Vector3d>(wr+3*j)+f*Eigen::Map<const Vector3d>(wr+3*(j+1))-bg;
   Matrix3d rh=r*exp3(.5*w*dt);Vector3d aw=rh*a+g;
   if(covariance){
    M21 phi=M21::Identity();
    phi.block<3,3>(0,6)=Matrix3d::Identity()*dt;
    phi.block<3,3>(0,3)=-.5*rh*skew(a)*exp3(-.5*w*dt)*dt*dt;
    phi.block<3,3>(3,3)=exp3(-w*dt);
    phi.block<3,3>(6,3)=-rh*skew(a)*exp3(-.5*w*dt)*dt;
    Eigen::Matrix<double,9,6> gn=Eigen::Matrix<double,9,6>::Zero();
    gn.block<3,3>(0,0)=.5*alpha*rh*dt*dt;gn.block<3,3>(6,0)=alpha*rh*dt;
    gn.block<3,3>(3,3)=jr(w*dt)*dt;
    gn.block<3,3>(0,3)=-.25*rh*skew(a)*jr(.5*w*dt)*dt*dt*dt;
    gn.block<3,3>(6,3)=-.5*rh*skew(a)*jr(.5*w*dt)*dt*dt;
    phi.block<9,6>(0,9)=gn*(1-f);phi.block<9,6>(0,15)=gn*f;
    pc=(phi*pc*phi.transpose()).eval();xp=(phi*xp).eval();
   }
   p+=v*dt+.5*aw*dt*dt;v+=aw*dt;r=r*exp3(w*dt);now=stop;
   if(now>=it[j+1]-1e-12&&j+2<m){
    if(covariance){
     M21 shift=M21::Zero();shift.block<9,9>(0,0).setIdentity();shift.block<6,6>(9,15).setIdentity();
     pc=(shift*pc*shift.transpose()).eval();pc.block<6,6>(15,15)=q;xp=(shift*xp).eval();
    }j++;
   }
  }
  Eigen::Quaterniond quat(r);quat.normalize();
  Eigen::Map<Vector3d>(output+10*pose)=p;
  Eigen::Map<Eigen::Vector4d>(output+10*pose+3)=quat.coeffs();
  Eigen::Map<Vector3d>(output+10*pose+7)=v;
  if(covariance){
   if(pose){cp.block(6*pose,0,6,6*pose)=xp.block(0,0,6,6*pose);cp.block(0,6*pose,6*pose,6)=cp.block(6*pose,0,6,6*pose).transpose();}
   cp.block<6,6>(6*pose,6*pose)=pc.block<6,6>(0,0);
   xp.block(0,6*pose,21,6)=pc.block(0,0,21,6);
  }
 }
 if(covariance){
  Eigen::Map<Eigen::Matrix<double,9,Eigen::Dynamic,Eigen::RowMajor>> cross_output(cross,9,6*n);
  cross_output=xp.topRows<9>();
  Eigen::Map<Eigen::Matrix<double,9,9,Eigen::RowMajor>> last_output(lastp);
  last_output=pc.topLeftCorner<9,9>();
 }
 return 0;
}
