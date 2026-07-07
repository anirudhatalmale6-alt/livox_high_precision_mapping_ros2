#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
using namespace Eigen;
int fails=0;
#define CHECK(c) do{ if(!(c)){printf("FAIL %d: %s\n",__LINE__,#c);fails++;}else printf("ok %s\n",#c);}while(0)
bool mercatorProj(double B0,double L0,double B,double L,double&X,double&Y){
  static double _A=6378137,_B=6356752.3142,_B0=B0,_L0=L0;
  static double e=std::sqrt(1-(_B/_A)*(_B/_A));
  static double e_=std::sqrt((_A/_B)*(_A/_B)-1);
  static double NB0=((_A*_A)/_B)/std::sqrt(1+e_*e_*std::cos(_B0)*std::cos(_B0));
  static double K=NB0*std::cos(_B0);
  if(L<-M_PI||L>M_PI||B<-M_PI_2||B>M_PI_2)return false;
  Y=K*(L-_L0); X=K*std::log(std::tan(M_PI_4+B/2)*std::pow((1-e*std::sin(B))/(1+e*std::sin(B)),e/2));
  return true;
}
// Build the per-point transform exactly like the node.
Matrix4d rtk2lidar;
Matrix4d makeTrans(const Matrix3d& rot,const Vector3d& cn,const Matrix4d& T1){
  Matrix4d T=Matrix4d::Identity(); T.block<3,3>(0,0)=rot; T.block<3,1>(0,3)=cn;
  return rtk2lidar*T1.inverse()*T*rtk2lidar.inverse();
}
int main(){
  rtk2lidar<<-1,0,0,0, 0,-1,0,0, 0,0,1,0, 0,0,0,1;
  double deg=M_PI/180.0;
  // Scenario: reference lat/lon
  double B0=48.1173*deg, L0=11.5167*deg;
  double x0,y0; mercatorProj(B0,L0,B0,L0,x0,y0);
  printf("ref proj x=%.6f y=%.6f (expect ~0,0)\n",x0,y0);
  CHECK(std::fabs(x0)<1e-6 && std::fabs(y0)<1e-6);

  // reference point transform == identity
  Matrix3d I3=Matrix3d::Identity();
  Matrix4d T1=Matrix4d::Identity(); T1.block<3,3>(0,0)=I3; T1.block<3,1>(0,3)=Vector3d(0,0,0);
  Matrix4d trans_ref=makeTrans(I3,Vector3d(0,0,0),T1);
  CHECK((trans_ref-Matrix4d::Identity()).norm()<1e-9);
  Vector4d pt(3,2,1,1);
  CHECK(((trans_ref*pt)-pt).norm()<1e-9);

  // stationary platform: any later point, same rot & pos as ref -> identity
  Matrix4d trans_stat=makeTrans(I3,Vector3d(0,0,0),T1);
  CHECK(((trans_stat*pt)-pt).norm()<1e-9);

  // pure translation: platform moved +5m east (y in merc), no rotation.
  // Later RTK gives cn=(0,5,0) relative to ref. Point at lidar origin should
  // map to +5 in the mapped frame after rtk2lidar sandwiching.
  Vector3d cn(0,5,0);
  Matrix4d trans_move=makeTrans(I3,cn,T1);
  Vector4d origin(0,0,0,1);
  Vector4d moved=trans_move*origin;
  printf("moved origin -> (%.3f, %.3f, %.3f)\n",moved[0],moved[1],moved[2]);
  // rtk2lidar flips x,y sign: T1.inv*T = translation cn; sandwich by rtk2lidar
  // gives translation rtk2lidar*cn = (-0,-5,0). Magnitude preserved = 5m.
  CHECK(std::fabs(moved.head<3>().norm()-5.0)<1e-9);

  // A real geodetic displacement: move ~1 arcsec north; check metric scale ~30m.
  double x1,y1; mercatorProj(B0,L0,B0+1.0/3600.0*deg,L0,x1,y1);
  double north_m=x1-x0;
  printf("1 arcsec north = %.3f m (expect ~30-31m)\n",north_m);
  CHECK(north_m>28.0 && north_m<33.0);

  printf("\n%s (%d fails)\n",fails==0?"ALL PASS":"FAIL",fails);
  return fails;
}
