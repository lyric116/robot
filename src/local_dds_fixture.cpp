// Offline integration fixture. Deliberately hard-coded to loopback, not eth0.
#include "throw_policy.hpp"
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/dds_wrapper/common/crc.h>
#include <atomic>
#include <chrono>
#include <thread>
#include <iostream>
using namespace throwing;
using namespace unitree::robot;
int main(int argc,char** argv){
 if(argc!=3)return 2;
 std::string root=argv[1];double duration=std::stod(argv[2]);
 auto config=YAML::LoadFile(root+"/policy/deploy.yaml");
 auto rows=YAML::LoadFile(root+"/validation/golden_rollout.json")["rows"];
 ChannelFactory::Instance()->Init(77,"lo");
 using S=unitree_hg::msg::dds_::LowState_;using C=unitree_hg::msg::dds_::LowCmd_;
 ChannelPublisher<S> publisher("rt/lowstate");publisher.InitChannel();
 std::atomic<unsigned> received{0},pd_double_count{0};std::atomic<bool> nonfinite{false};
 ChannelSubscriber<C> subscriber("rt/lowcmd");
 subscriber.InitChannel([&](const void* p){auto& c=*static_cast<const C*>(p);received++;
  for(int i=0;i<29;i++){const auto& m=c.motor_cmd()[i];
   if(std::abs(m.tau())>1e-5f&&(m.kp()!=0||m.kd()!=0))pd_double_count++;
   if(!std::isfinite(m.tau())||!std::isfinite(m.kp())||!std::isfinite(m.kd()))nonfinite=true;
  }
 },1);
 auto start=std::chrono::steady_clock::now(),next=start;
 for(;;){double t=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();if(t>duration)break;
  int index=t<5?0:std::clamp(static_cast<int>((t-5)/.02),0,static_cast<int>(rows.size())-1);
  auto row=rows[index];auto q=row["q_sdk"].as<std::vector<float>>(),qd=row["qd_sdk"].as<std::vector<float>>();
  auto quat=row["quaternion_wxyz"].as<std::vector<float>>(),gyro=row["gyro"].as<std::vector<float>>();
  S s{};s.mode_machine()=5;
  for(int i=0;i<29;i++){s.motor_state()[i].q()=q[i];s.motor_state()[i].dq()=qd[i];}
  for(int i=0;i<4;i++)s.imu_state().quaternion()[i]=quat[i];
  for(int i=0;i<3;i++)s.imu_state().gyroscope()[i]=gyro[i];
  uint16_t keys=0;if(t>=2&&t<2.2)keys=(1u<<5)|(1u<<12);
  if((t>=5&&t<5.2)||(t>=9.6&&t<9.8))keys=(1u<<4)|(1u<<8);
  s.wireless_remote()[0]=0xfe;s.wireless_remote()[1]=0xef;
  s.wireless_remote()[2]=keys&255;s.wireless_remote()[3]=keys>>8;
  s.crc()=crc32_core(reinterpret_cast<uint32_t*>(&s),(sizeof(S)>>2)-1);publisher.Write(s);
  next+=std::chrono::microseconds(2000);std::this_thread::sleep_until(next);
 }
 std::cout<<"{\"interface\":\"lo\",\"lowcmd_received\":"<<received.load()<<",\"double_pd_count\":"<<pd_double_count.load()<<",\"nonfinite\":"<<(nonfinite.load()?"true":"false")<<"}"<<std::endl;
}
