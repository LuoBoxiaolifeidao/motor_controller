#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <iostream>

namespace mppi_controller {

struct JointData { const char* name; bool pris; double ox,oy,oz,ar,ap,ay; int parent; };
struct CylData  { const char* name; double r,l,cx,cy,cz,crx,cry,crz; int jidx; };

static const std::vector<JointData> JOINTS = {
  {"l_post_Joint",     true, -0.1275,-0.1065,0.34054,0,0,0,-1},
  {"l_shoulder_Joint", false,0,0.119,0.0575,0,0,0,0},
  {"l_arm_JointA",     true, 0.0002,0.254,0.0328,-1.5708,0,0,1},
  {"l_arm_JointB",     true, 0,0,0.015,0,0,0,2},
  {"l_arm_JointC",     true, 0,0,0.011,0,0,0,3},
  {"l_wrist_p_Joint",  false,-0.0002,0.0368,0.057,-1.5708,0,0,4},
  {"l_wrist_y_Joint",  false,0,0.027,0.0565,1.5708,0,0,5},
  {"l_wrist_r_Joint",  false,0,0.042,0.027,-1.5708,0,0,6},
  {"r_post_Joint",     true, 0.1275,-0.1065,0.34054,0,0,0,-1},
  {"r_shoulder_Joint", false,0,0.119,0.0575,0,0,0,8},
  {"r_arm_JointA",     true, 0,0.254,0.033,-1.5708,0,0,9},
  {"r_arm_JointB",     true, 0,0,0.015,0,0,0,10},
  {"r_arm_JointC",     true, 0,0,0.011,0,0,0,11},
  {"r_wrist_p_Joint",  false,0,0.037,0.066375,-1.5708,0,0,12},
  {"r_wrist_y_Joint",  false,0,0.036375,0.0565,1.5708,0,0,13},
  {"r_wrist_r_Joint",  false,0,0.042,0.027,-1.5708,0,0,14},
  {"l-hand-l-finger",  false,-0.044,0,0.112,1.5708,0,0,7},
  {"l-hand-r-finger",  false,0.044,0,0.112,1.5708,0,0,7},
  {"r-hand-l-finger",  false,-0.044,0,0.112,1.5708,0,0,15},
  {"r-hand-r-finger",  false,0.044,0,0.112,1.5708,0,0,15},
};

static const std::vector<CylData> CYLINDERS = {
  {"base_link",           0.2374,0.3836,0,-0.02,0.150,0,0,0,-1},
  {"l_post_Link",         0.1278,0.2900,0,0.1,0.15,-1.5708,0,0,0},
  {"l_shoulder_Link",     0.1278,0.1000,0,0.1,0.15,0,1.57,0,1},
  {"l_arm_LinkA",         0.0461,0.1400,0,0,-0.07,0,0,0,2},
  {"l_arm_LinkB",         0.0461,0.1500,0,0,-0.075,0,0,0,3},
  {"l_arm_LinkC",         0.0461,0.2500,0,0,-0.03,0,0,0,4},
  {"l_wrist_p_Link",      0.0472,0.0900,0,0.05,0.045,-1.5708,0,0,5},
  {"l_wrist_y_Link",      0.0391,0.0800,0.01,0.01,0.035,0,1.5708,0,6},
  {"l_wrist_r_Link",      0.0743,0.1400,0,-0.022,0.07,0,0,0,7},
  {"l-hand-l-finger_Link",0.0141,0.2000,-0.02,0.11,0,1.57,0,0.5,16},
  {"l-hand-r-finger_Link",0.0141,0.2000,0.02,0.11,0,1.57,0,-0.5,17},
  {"r_post_Link",         0.1278,0.2900,0,0.1,0.15,-1.5708,0,0,8},
  {"r_shoulder_Link",     0.1278,0.1000,0,0.1,0.15,0,1.57,0,9},
  {"r_arm_LinkA",         0.0461,0.1400,0,0,-0.07,0,0,0,10},
  {"r_arm_LinkB",         0.0461,0.1500,0,0,-0.075,0,0,0,11},
  {"r_arm_LinkC",         0.0461,0.2500,0,0,-0.03,0,0,0,12},
  {"r_wrist_p_Link",      0.0472,0.0900,0,0.05,0.045,-1.5708,0,0,13},
  {"r_wrist_y_Link",      0.0391,0.0800,0.01,0.01,0.035,0,1.5708,0,14},
  {"r_wrist_r_Link",      0.0743,0.1400,0,-0.022,0.07,0,0,0,15},
  {"r-hand-l-finger_Link",0.0141,0.2000,-0.02,0.11,0,1.57,0,0.5,18},
  {"r-hand-r-finger_Link",0.0141,0.2000,0.02,0.11,0,1.57,0,-0.5,19},
};

static const int COLL_PAIRS[][2] = {
  {0,18},{0,19},{0,20},
  {0,8},{0,9},{0,10},
  {3,13},{3,14},{3,15},{3,16},{3,17},{3,18},{3,19},{3,20},
  {4,13},{4,14},{4,15},{4,16},{4,17},{4,18},{4,19},{4,20},
  {5,13},{5,14},{5,15},{5,16},{5,17},{5,18},{5,19},{5,20},
  {6,13},{6,14},{6,15},{6,16},{6,17},{6,18},{6,19},{6,20},
  {7,13},{7,14},{7,15},{7,16},{7,17},{7,18},{7,19},{7,20},
  {8,13},{8,14},{8,15},{8,16},{8,17},{8,18},{8,19},{8,20},
  {9,13},{9,14},{9,15},{9,16},{9,17},{9,18},{9,19},{9,20},
  {10,13},{10,14},{10,15},{10,16},{10,17},{10,18},{10,19},{10,20},
};
static const int N_COLL_PAIRS = sizeof(COLL_PAIRS) / sizeof(COLL_PAIRS[0]);

inline double cpp_capsule_dist(
    const double* p0_a, const double* p1_a, double r_a,
    const double* p0_b, const double* p1_b, double r_b) {
  auto dot=[](const double*a,const double*b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];};
  double d1[3]={p1_a[0]-p0_a[0],p1_a[1]-p0_a[1],p1_a[2]-p0_a[2]};
  double d2[3]={p1_b[0]-p0_b[0],p1_b[1]-p0_b[1],p1_b[2]-p0_b[2]};
  double r[3]={p0_a[0]-p0_b[0],p0_a[1]-p0_b[1],p0_a[2]-p0_b[2]};
  double a2=dot(d1,d1),e2=dot(d2,d2),f=dot(d2,r),c=dot(d1,r),b=dot(d1,d2);
  double eps=1e-12,denom=a2*e2-b*b,s=0,t=0;
  if(a2<eps&&e2<eps){}
  else if(a2<eps){t=std::clamp(f/std::max(e2,eps),0.0,1.0);}
  else if(e2<eps){s=std::clamp(-c/a2,0.0,1.0);}
  else{
    if(denom>eps)s=std::clamp((b*f-c*e2)/denom,0.0,1.0);
    t=(b*s+f)/e2;
    if(t<0){t=0;s=std::clamp(-c/a2,0.0,1.0);}
    else if(t>1){t=1;s=std::clamp((b-c)/a2,0.0,1.0);}
  }
  double cp[3],cq[3],delta[3];
  for(int i=0;i<3;i++){cp[i]=p0_a[i]+d1[i]*s;cq[i]=p0_b[i]+d2[i]*t;delta[i]=cp[i]-cq[i];}
  return std::sqrt(dot(delta,delta))-r_a-r_b;
}

inline void matmul44(double* R,const double*A,const double*B){
  double t[16]={};
  for(int i=0;i<4;i++)for(int j=0;j<4;j++)
    for(int k=0;k<4;k++)t[i*4+j]+=A[i*4+k]*B[k*4+j];
  for(int i=0;i<16;i++)R[i]=t[i];
}

static const double COLL_MARGIN = 0.02;

inline bool check_collision(const std::vector<double>& q16,
                             std::string* worst_pair_out = nullptr,
                             double* worst_dist_out = nullptr) {
  int NJ=(int)JOINTS.size(),NC=(int)CYLINDERS.size();

  std::vector<std::vector<double>> JT_orig(NJ,{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1});
  for(int ji=0;ji<NJ;ji++){
    auto& j=JOINTS[ji];
    double cr=cos(j.ar),sr=sin(j.ar),cp=cos(j.ap),sp=sin(j.ap),cy=cos(j.ay),sy=sin(j.ay);
    JT_orig[ji][0]=cy*cp;JT_orig[ji][1]=cy*sp*sr-sy*cr;JT_orig[ji][2]=cy*sp*cr+sy*sr;JT_orig[ji][3]=j.ox;
    JT_orig[ji][4]=sy*cp;JT_orig[ji][5]=sy*sp*sr+cy*cr;JT_orig[ji][6]=sy*sp*cr-cy*sr;JT_orig[ji][7]=j.oy;
    JT_orig[ji][8]=-sp;  JT_orig[ji][9]=cp*sr;        JT_orig[ji][10]=cp*cr;       JT_orig[ji][11]=j.oz;
  }

  std::vector<std::vector<double>> JT(NJ);
  for(int ji=0;ji<NJ;ji++){
    JT[ji]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
    auto& j=JOINTS[ji];
    double qj=0;
    if(ji>=0&&ji<8)qj=q16[ji];
    else if(ji>=8&&ji<16)qj=q16[ji];
    double cq=cos(qj),sq=sin(qj);
    double Tm[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
    if(j.pris)Tm[11]=qj;
    else{Tm[0]=cq;Tm[1]=-sq;Tm[4]=sq;Tm[5]=cq;}
    double O1[16];matmul44(O1,JT_orig[ji].data(),Tm);
    if(j.parent>=0)matmul44(JT[ji].data(),JT[j.parent].data(),O1);
    else for(int i=0;i<16;i++)JT[ji][i]=O1[i];
  }

  std::vector<std::vector<double>> pw0(NC,{0,0,0}),pw1(NC,{0,0,0});
  static const double I4[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
  for(int ci=0;ci<NC;ci++){
    int ji=CYLINDERS[ci].jidx;
    auto* T=(ji<0)? I4 : JT[ji].data();
    double cr=cos(CYLINDERS[ci].crx),sr=sin(CYLINDERS[ci].crx),cp=cos(CYLINDERS[ci].cry),sp=sin(CYLINDERS[ci].cry),cy=cos(CYLINDERS[ci].crz),sy=sin(CYLINDERS[ci].crz);
    double Rc[9]={cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr,
                  sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr,
                  -sp,cp*sr,cp*cr};
    double half=CYLINDERS[ci].l*0.5;
    double pl0[3]={Rc[2]*(-half)+CYLINDERS[ci].cx,Rc[5]*(-half)+CYLINDERS[ci].cy,Rc[8]*(-half)+CYLINDERS[ci].cz};
    double pl1[3]={Rc[2]*(+half)+CYLINDERS[ci].cx,Rc[5]*(+half)+CYLINDERS[ci].cy,Rc[8]*(+half)+CYLINDERS[ci].cz};
    pw0[ci][0]=T[0]*pl0[0]+T[1]*pl0[1]+T[2]*pl0[2]+T[3];
    pw0[ci][1]=T[4]*pl0[0]+T[5]*pl0[1]+T[6]*pl0[2]+T[7];
    pw0[ci][2]=T[8]*pl0[0]+T[9]*pl0[1]+T[10]*pl0[2]+T[11];
    pw1[ci][0]=T[0]*pl1[0]+T[1]*pl1[1]+T[2]*pl1[2]+T[3];
    pw1[ci][1]=T[4]*pl1[0]+T[5]*pl1[1]+T[6]*pl1[2]+T[7];
    pw1[ci][2]=T[8]*pl1[0]+T[9]*pl1[1]+T[10]*pl1[2]+T[11];
  }

  double worst_d = 1e9;
  const char* worst_a = "";
  const char* worst_b = "";
  for(int pi=0;pi<N_COLL_PAIRS;pi++){
    int a=COLL_PAIRS[pi][0],b=COLL_PAIRS[pi][1];
    double d=cpp_capsule_dist(pw0[a].data(),pw1[a].data(),CYLINDERS[a].r,
                              pw0[b].data(),pw1[b].data(),CYLINDERS[b].r);
    if(d < worst_d){worst_d=d;worst_a=CYLINDERS[a].name;worst_b=CYLINDERS[b].name;}
    if(d <= COLL_MARGIN){
      if(worst_pair_out)*worst_pair_out=std::string(worst_a)+" vs "+worst_b;
      if(worst_dist_out)*worst_dist_out=worst_d;
      return true;
    }
  }
  if(worst_dist_out)*worst_dist_out=worst_d;
  return false;
}

inline double finger_wrist_dist(const std::vector<double>& q16,
                                 std::string* pair_out = nullptr) {
  int NJ=(int)JOINTS.size(),NC=(int)CYLINDERS.size();
  std::vector<std::vector<double>> JT_orig(NJ,{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1});
  for(int ji=0;ji<NJ;ji++){
    auto& j=JOINTS[ji];
    double cr=cos(j.ar),sr=sin(j.ar),cp=cos(j.ap),sp=sin(j.ap),cy=cos(j.ay),sy=sin(j.ay);
    JT_orig[ji][0]=cy*cp;JT_orig[ji][1]=cy*sp*sr-sy*cr;JT_orig[ji][2]=cy*sp*cr+sy*sr;JT_orig[ji][3]=j.ox;
    JT_orig[ji][4]=sy*cp;JT_orig[ji][5]=sy*sp*sr+cy*cr;JT_orig[ji][6]=sy*sp*cr-cy*sr;JT_orig[ji][7]=j.oy;
    JT_orig[ji][8]=-sp;  JT_orig[ji][9]=cp*sr;        JT_orig[ji][10]=cp*cr;       JT_orig[ji][11]=j.oz;
  }
  std::vector<std::vector<double>> JT(NJ);
  for(int ji=0;ji<NJ;ji++){
    JT[ji]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
    auto& j=JOINTS[ji];
    double qj=0;
    if(ji>=0&&ji<8)qj=q16[ji];
    else if(ji>=8&&ji<16)qj=q16[ji];
    double cq=cos(qj),sq=sin(qj);
    double Tm[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
    if(j.pris)Tm[11]=qj;
    else{Tm[0]=cq;Tm[1]=-sq;Tm[4]=sq;Tm[5]=cq;}
    double O1[16];matmul44(O1,JT_orig[ji].data(),Tm);
    if(j.parent>=0)matmul44(JT[ji].data(),JT[j.parent].data(),O1);
    else for(int i=0;i<16;i++)JT[ji][i]=O1[i];
  }
  std::vector<std::vector<double>> pw0(NC,{0,0,0}),pw1(NC,{0,0,0});
  static const double ID4[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
  for(int ci=0;ci<NC;ci++){
    int ji=CYLINDERS[ci].jidx;
    auto* T=(ji<0)? ID4 : JT[ji].data();
    double cr=cos(CYLINDERS[ci].crx),sr=sin(CYLINDERS[ci].crx),cp=cos(CYLINDERS[ci].cry),sp=sin(CYLINDERS[ci].cry),cy=cos(CYLINDERS[ci].crz),sy=sin(CYLINDERS[ci].crz);
    double Rc[9]={cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr,
                  sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr,
                  -sp,cp*sr,cp*cr};
    double half=CYLINDERS[ci].l*0.5;
    double pl0[3]={Rc[2]*(-half)+CYLINDERS[ci].cx,Rc[5]*(-half)+CYLINDERS[ci].cy,Rc[8]*(-half)+CYLINDERS[ci].cz};
    double pl1[3]={Rc[2]*(+half)+CYLINDERS[ci].cx,Rc[5]*(+half)+CYLINDERS[ci].cy,Rc[8]*(+half)+CYLINDERS[ci].cz};
    pw0[ci][0]=T[0]*pl0[0]+T[1]*pl0[1]+T[2]*pl0[2]+T[3];
    pw0[ci][1]=T[4]*pl0[0]+T[5]*pl0[1]+T[6]*pl0[2]+T[7];
    pw0[ci][2]=T[8]*pl0[0]+T[9]*pl0[1]+T[10]*pl0[2]+T[11];
    pw1[ci][0]=T[0]*pl1[0]+T[1]*pl1[1]+T[2]*pl1[2]+T[3];
    pw1[ci][1]=T[4]*pl1[0]+T[5]*pl1[1]+T[6]*pl1[2]+T[7];
    pw1[ci][2]=T[8]*pl1[0]+T[9]*pl1[1]+T[10]*pl1[2]+T[11];
  }
  int key_pairs[][2]={{9,16},{10,16},{19,6},{20,6}};
  double worst=1e9; const char* wa="",*wb="";
  for(int pi=0;pi<4;pi++){
    int a=key_pairs[pi][0],b=key_pairs[pi][1];
    double d=cpp_capsule_dist(pw0[a].data(),pw1[a].data(),CYLINDERS[a].r,
                              pw0[b].data(),pw1[b].data(),CYLINDERS[b].r);
    if(d<worst){worst=d;wa=CYLINDERS[a].name;wb=CYLINDERS[b].name;}
  }
  if(pair_out)*pair_out=std::string(wa)+" vs "+wb;
  return worst;
}

} // namespace mppi_controller
