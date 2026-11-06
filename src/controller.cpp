// Throw-specific adapter using Unitree SDK2 low-level messages and the official
// Passive -> FixStand -> manual policy-start interaction. No hand motor commands.
#include "throw_policy.hpp"
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/dds_wrapper/common/crc.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
#include <csignal>
#include <atomic>
#include <chrono>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>
using namespace throwing;
using Clock=std::chrono::steady_clock;
using LowState=unitree_hg::msg::dds_::LowState_;
using LowCmd=unitree_hg::msg::dds_::LowCmd_;
using namespace unitree::robot;
static volatile std::sig_atomic_t stopped=0;
static void interrupt(int){stopped=1;}
static double seconds(){return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();}
enum class Stage{Passive,FixStand,Throw,Hold};
static const char* name(Stage s){switch(s){case Stage::Passive:return "Passive";case Stage::FixStand:return "FixStand";case Stage::Throw:return "Throw";default:return "Hold";}}

struct StateBuffer {
 std::mutex lock;LowState message{};double timestamp=0.;uint64_t received=0,bad_crc=0;
 void receive(const void* ptr){
  auto* msg=static_cast<const LowState*>(ptr);
  if(msg->crc()!=crc32_core(reinterpret_cast<uint32_t*>(const_cast<LowState*>(msg)),(sizeof(LowState)>>2)-1)){
   std::lock_guard<std::mutex> g(lock);++bad_crc;return;
  }
  std::lock_guard<std::mutex> g(lock);message=*msg;timestamp=seconds();++received;
 }
 bool snapshot(LowState& out,double& age){std::lock_guard<std::mutex> g(lock);out=message;age=seconds()-timestamp;return received!=0;}
};

struct Telemetry {
 std::mutex mutex;std::string status="{}";std::deque<std::string> events;std::atomic<bool> running{true};std::thread worker;
 explicit Telemetry(const std::string& root){
  worker=std::thread([this,root]{std::ofstream event_file(root+"/release_events.jsonl",std::ios::app);int count=0;
   while(running){std::string current;std::deque<std::string> pending;
    {std::lock_guard<std::mutex> l(mutex);current=status;pending.swap(events);}
    for(const auto& e:pending){event_file<<e<<'\n';std::cout<<"RELEASE_EVENT "<<e<<std::endl;}event_file.flush();
    {std::ofstream f(root+"/status.json.tmp");f<<current<<'\n';}std::rename((root+"/status.json.tmp").c_str(),(root+"/status.json").c_str());
    if(++count%5==0)std::cout<<current<<std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
   }
   std::lock_guard<std::mutex> l(mutex);for(const auto& e:events)event_file<<e<<'\n';event_file.flush();
  });
 }
 ~Telemetry(){running=false;if(worker.joinable())worker.join();}
 void set(std::string s){std::lock_guard<std::mutex> l(mutex);status=std::move(s);}
 void event(std::string s){std::lock_guard<std::mutex> l(mutex);events.push_back(std::move(s));}
};

class ReleaseOutput {
 int socket_=-1;sockaddr_un address_{};
public:
 explicit ReleaseOutput(const std::string& path){
  if(path.empty())return;
  if(path.size()>=sizeof(address_.sun_path))throw std::runtime_error("Release socket path too long");
  socket_=::socket(AF_UNIX,SOCK_DGRAM|SOCK_NONBLOCK,0);
  if(socket_<0)throw std::runtime_error("Cannot create release event socket");
  address_.sun_family=AF_UNIX;std::strncpy(address_.sun_path,path.c_str(),sizeof(address_.sun_path)-1);
 }
 ~ReleaseOutput(){if(socket_>=0)::close(socket_);}
 bool send(const std::string& event){return socket_>=0&&::sendto(socket_,event.data(),event.size(),MSG_DONTWAIT|MSG_NOSIGNAL,reinterpret_cast<sockaddr*>(&address_),sizeof(address_))==static_cast<ssize_t>(event.size());}
};

static Measurement measurement(const LowState& s){
 Measurement m;for(int i=0;i<29;i++){m.q[i]=s.motor_state()[i].q();m.qd[i]=s.motor_state()[i].dq();}
 for(int i=0;i<4;i++)m.quat[i]=s.imu_state().quaternion()[i];
 for(int i=0;i<3;i++)m.gyro[i]=s.imu_state().gyroscope()[i];
 return m;
}

int main(int argc,char** argv){try{
 std::string root=".",network="eth0";bool control=false;int dds_domain=0;
 for(int i=1;i<argc;i++){
  std::string a=argv[i];if(a=="--root"&&i+1<argc)root=argv[++i];
  else if(a=="--network"&&i+1<argc)network=argv[++i];
  else if(a=="--dds-domain"&&i+1<argc)dds_domain=std::stoi(argv[++i]);
  else if(a=="--control")control=true;else if(a=="--monitor")control=false;
  else if(a=="--help"){std::cout<<"g1_throw --root PACKAGE --network eth0 [--monitor | --control]\nDefault: monitor only, publishes no lowcmd.\n";return 0;}
  else throw std::runtime_error("Unknown/incomplete argument: "+a);
 }
 if(dds_domain!=0&&(dds_domain!=77||network!="lo"))throw std::runtime_error("Domain 77 is reserved for loopback-only offline tests");
 Policy policy(root);auto config=YAML::LoadFile(root+"/policy/deploy.yaml");
 const double state_timeout=config["state_timeout_s"].as<double>();
 const double prepare_s=config["preparation_s"].as<double>();
 if(state_timeout<=0||state_timeout>1||prepare_s<1)throw std::runtime_error("Invalid runtime timing");
 int lock_fd=open("/tmp/g1_throw_controller.lock",O_CREAT|O_RDWR,0600);
 if(lock_fd<0||flock(lock_fd,LOCK_EX|LOCK_NB))throw std::runtime_error("Another throw controller is running");
 std::signal(SIGINT,interrupt);std::signal(SIGTERM,interrupt);
 ChannelFactory::Instance()->Init(dds_domain,network);
 StateBuffer state;
 auto subscriber=std::make_unique<ChannelSubscriber<LowState>>("rt/lowstate");
 subscriber->InitChannel([&](const void* p){state.receive(p);},1);
 std::atomic<bool> other_command{false};
 auto probe=std::make_unique<ChannelSubscriber<LowCmd>>("rt/lowcmd");
 probe->InitChannel([&](const void*){other_command=true;},1);
 std::cout<<"Waiting for valid G1 lowstate; no lowcmd published yet..."<<std::endl;
 LowState msg;double age=0;
 while(!stopped&&(!state.snapshot(msg,age)||age>state_timeout))std::this_thread::sleep_for(std::chrono::milliseconds(20));
 if(stopped)return 0;
 std::this_thread::sleep_for(std::chrono::milliseconds(300));
 if(!state.snapshot(msg,age)||age>state_timeout)throw std::runtime_error("Lowstate became stale before startup");
 if(msg.mode_machine()!=5)throw std::runtime_error("Expected G1 29-DOF mode_machine=5");
 if(control&&other_command)throw std::runtime_error("rt/lowcmd already has a publisher. Stop the existing controller / release official mode first.");
 probe.reset();
 std::unique_ptr<ChannelPublisher<LowCmd>> publisher;
 if(control){publisher=std::make_unique<ChannelPublisher<LowCmd>>("rt/lowcmd");publisher->InitChannel();}
 ReleaseOutput release(config["release_socket"].as<std::string>());Telemetry telemetry(root);
 Stage stage=Stage::Passive;double stage_start=seconds();Joints initial{};
 uint16_t previous_keys=0;bool keys_initialized=false;unsigned servo_ticks=0,trial=0;double inference_ms=0;
 std::string fault;bool delivered=false;double release_time=-1.;uint64_t published=0;
 // FixStand gains in SDK order, from Unitree's G1 deployment configuration.
 const Joints stand_kp_sdk={100,100,100,150,40,40,100,100,100,150,40,40,200,200,200,40,40,40,40,40,40,40,40,40,40,40,40,40,40};
 const Joints stand_kd_sdk={2,2,2,4,2,2,2,2,2,4,2,2,5,5,5,10,10,10,10,10,10,10,10,10,10,10,10,10,10};
 Joints stand_kp{},stand_kd{};for(int i=0;i<29;i++){stand_kp[i]=stand_kp_sdk[policy.map[i]];stand_kd[i]=stand_kd_sdk[policy.map[i]];}
 const uint16_t L2=1u<<5,R2=1u<<4,Up=1u<<12,A=1u<<8,B=1u<<9;
 auto next=Clock::now();const auto servo_period=std::chrono::microseconds(2000);
 std::cout<<(control?"CONTROL: Passive. L2+Up prepares; after placing feet on ground R2+A starts ONE throw. L2+B -> Passive.":"MONITOR ONLY: remote keys do not start any motion; no lowcmd publisher.")<<std::endl;
 while(!stopped){
  next+=servo_period;state.snapshot(msg,age);auto m=measurement(msg);bool valid=true;
  try{policy.check_measurement(m);if(msg.mode_machine()!=5)throw std::runtime_error("mode_machine changed");if(age>state_timeout)throw std::runtime_error("lowstate timeout");}
  catch(const std::exception& e){valid=false;fault=e.what();stage=Stage::Passive;}
  uint16_t keys=static_cast<uint16_t>(msg.wireless_remote()[2])|(static_cast<uint16_t>(msg.wireless_remote()[3])<<8);
  if(!keys_initialized){previous_keys=keys;keys_initialized=true;}
  auto pressed=[&](uint16_t chord){return (keys&chord)==chord&&(previous_keys&chord)!=chord;};
  if(control&&valid){
   if(pressed(L2|B)){stage=Stage::Passive;stage_start=seconds();}
   else if(pressed(L2|Up)&&(stage==Stage::Passive||stage==Stage::Hold)){
    for(int i=0;i<29;i++)initial[i]=m.q[policy.map[i]];
    stage=Stage::FixStand;stage_start=seconds();fault.clear();
   }else if(pressed(R2|A)&&stage==Stage::FixStand&&seconds()-stage_start>=prepare_s){
    try{policy.check_operating_limits(m);policy.reset(m);stage=Stage::Throw;servo_ticks=0;stage_start=seconds();
      ++trial;delivered=false;release_time=-1.;fault.clear();}
    catch(const std::exception& e){fault=e.what();stage=Stage::Passive;}
   }
  }
  previous_keys=keys;
  Joints tau{};
  if(control&&valid){try{
   if(stage==Stage::FixStand){
    const float alpha=std::clamp(static_cast<float>((seconds()-stage_start)/prepare_s),0.f,1.f);
    Joints desired{};for(int i=0;i<29;i++)desired[i]=std::clamp(initial[i]+alpha*(policy.q0[i]-initial[i]),policy.lower[i],policy.upper[i]);
    tau=policy.torque(m,desired,stand_kp,stand_kd);
   }else if(stage==Stage::Throw||stage==Stage::Hold){
    policy.check_operating_limits(m);
    if(stage==Stage::Throw){
     if(servo_ticks*static_cast<double>(policy.servo_dt)>=policy.duration-1e-6){stage=Stage::Hold;stage_start=seconds();}
     else if(servo_ticks%10==0){
      double began=seconds();bool edge=policy.update(m);inference_ms=(seconds()-began)*1000.;
      if(edge){
       release_time=(policy.step-1)*static_cast<double>(policy.dt);
       const auto stamp=std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
       std::ostringstream event;event<<std::setprecision(9)<<"{\"event\":\"release\",\"hand\":\"right\",\"pid\":"<<getpid()<<",\"trial_id\":"<<trial<<",\"policy_step\":"<<policy.step-1<<",\"policy_time_s\":"<<release_time<<",\"monotonic_ns\":"<<stamp<<",\"action\":"<<policy.last[29]<<",\"release_commanded\":true}";
       delivered=release.send(event.str());telemetry.event(event.str());
      }
     }
     ++servo_ticks;
    }
    tau=policy.torque(m);
   }
  }catch(const std::exception& e){fault=e.what();stage=Stage::Passive;}}
  if(publisher){
   LowCmd command{};command.mode_pr()=0;command.mode_machine()=5;
   for(int i=0;i<29;i++){
    auto& motor=command.motor_cmd()[policy.map[i]];motor.mode()=1;motor.q()=0;motor.dq()=0;
    motor.kp()=0;motor.kd()=stage==Stage::Passive?3.f:0.f;motor.tau()=stage==Stage::Passive?0.f:tau[i];
   }
   command.crc()=crc32_core(reinterpret_cast<uint32_t*>(&command),(sizeof(LowCmd)>>2)-1);publisher->Write(command);++published;
  }
  std::ostringstream status;status<<std::setprecision(7)<<"{\"mode\":\""<<(control?"CONTROL":"MONITOR")<<"\",\"state\":\""<<name(stage)<<"\",\"trial_id\":"<<trial<<",\"policy_step\":"<<policy.step<<",\"release_commanded\":"<<(policy.released?"true":"false")<<",\"release_event_delivered\":"<<(delivered?"true":"false")<<",\"release_time_s\":"<<release_time<<",\"state_age_ms\":"<<age*1000.<<",\"inference_ms\":"<<inference_ms<<",\"published\":"<<published<<",\"fault\":"<<std::quoted(fault)<<"}";
  telemetry.set(status.str());
  std::this_thread::sleep_until(next);
  if(Clock::now()>next+std::chrono::milliseconds(20))next=Clock::now();
 }
 if(publisher){
  // Same passive damping as the official Passive state, no automatic home reset.
  LowCmd command{};command.mode_pr()=0;command.mode_machine()=5;
  for(int i=0;i<29;i++){command.motor_cmd()[i].mode()=1;command.motor_cmd()[i].kd()=3;}
  for(int i=0;i<50;i++){command.crc()=crc32_core(reinterpret_cast<uint32_t*>(&command),(sizeof(LowCmd)>>2)-1);publisher->Write(command);std::this_thread::sleep_for(servo_period);}
 }
 ::close(lock_fd);return 0;
}catch(const std::exception& e){std::cerr<<"ERROR: "<<e.what()<<std::endl;return 1;}}
