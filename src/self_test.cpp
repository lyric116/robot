#include "throw_policy.hpp"
#include <iostream>
#include <limits>
using namespace throwing;
template<size_t N>void load(const YAML::Node& n,std::array<float,N>& out){auto v=n.as<std::vector<float>>();if(v.size()!=N)throw std::runtime_error("fixture size");std::copy(v.begin(),v.end(),out.begin());}
template<size_t N>float difference(const std::array<float,N>& a,const YAML::Node& n){auto b=n.as<std::vector<float>>();float e=0;for(size_t i=0;i<N;i++)e=std::max(e,std::abs(a[i]-b.at(i)));return e;}
int main(int argc,char** argv){try{
 const std::string root=argc>1?argv[1]:".";Policy p(root);auto f=YAML::LoadFile(root+"/validation/golden_rollout.json");
 float oe=0,ae=0,te=0,fe=0;int edges=0,release_step=-1;Measurement m;
 for(auto row:f["rows"]){load(row["q_sdk"],m.q);load(row["qd_sdk"],m.qd);load(row["quaternion_wxyz"],m.quat);load(row["gyro"],m.gyro);
  if(row["step"].as<int>()==0)p.reset(m);
  auto obs=p.observe(m);oe=std::max(oe,difference(obs,row["observation"]));
  auto a=p.infer(obs);ae=std::max(ae,difference(a,row["action"]));
  if(p.accept(a)){edges++;release_step=row["step"].as<int>();}
  te=std::max(te,difference(p.target,row["target"]));fe=std::max(fe,difference(p.torque(m),row["torque"]));
 }
 // Explicit extreme-command and irreversible-release checks.
 auto previous=p.target;Action extreme{};extreme.fill(1000);extreme[29]=1;
 bool duplicate=p.accept(extreme);bool bounded=true;
 for(int i=0;i<29;i++)bounded &= p.target[i]>=p.lower[i]-1e-6f&&p.target[i]<=p.upper[i]+1e-6f&&std::abs(p.target[i]-previous[i])<=p.velocity[i]*p.dt+1e-6f;
 bool nan_rejected=false;extreme[0]=std::numeric_limits<float>::quiet_NaN();try{p.accept(extreme);}catch(...){nan_rejected=true;}
 bool passed=oe<1e-4f&&ae<2e-4f&&te<1e-4f&&fe<.02f&&edges==1&&!duplicate&&bounded&&nan_rejected;
 std::cout<<"{\"passed\":"<<(passed?"true":"false")<<",\"observation_max_error\":"<<oe<<",\"action_max_error\":"<<ae<<",\"target_max_error_rad\":"<<te<<",\"torque_max_error_nm\":"<<fe<<",\"release_events\":"<<edges<<",\"release_step\":"<<release_step<<",\"release_time_s\":"<<release_step*.02<<",\"bounded_target_and_slew\":"<<(bounded?"true":"false")<<",\"nan_rejected\":"<<(nan_rejected?"true":"false")<<",\"robot_commands_sent\":false}"<<std::endl;
 return passed?0:1;
}catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 2;}}
