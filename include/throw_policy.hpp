#pragma once
#include <onnxruntime_cxx_api.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace throwing {
using Joints=std::array<float,29>;
using Action=std::array<float,30>;
using Observation=std::array<float,126>;
struct Measurement {
  Joints q{},qd{}; // Always motor/SDK order.
  std::array<float,4> quat{1,0,0,0};
  std::array<float,3> gyro{};
};

class Policy {
  Ort::Env environment_{ORT_LOGGING_LEVEL_WARNING,"g1_throw"};
  Ort::SessionOptions options_;
  std::unique_ptr<Ort::Session> session_;
  std::string input_,output_;
public:
  std::array<int,29> map{};
  Joints q0{},kp{},kd{},lower{},upper{},velocity{},effort{},scale{},target{};
  Action last{};
  bool released=false;
  unsigned step=0;
  float dt=.02f,servo_dt=.002f,duration=4.1f,threshold=.5f;
  float position_tolerance=.001745329f,speed_failure_ratio=1.01f;

  explicit Policy(const std::string& root) {
    auto c=YAML::LoadFile(root+"/policy/deploy.yaml");
    if(c["actor_dim"].as<int>()!=126||c["action_dim"].as<int>()!=30)
      throw std::runtime_error("Expected throwing actor 126 -> 30");
    auto load=[&](const char* key,Joints& value){auto v=c[key].as<std::vector<float>>();
      if(v.size()!=29)throw std::runtime_error(std::string("Invalid config ")+key);
      std::copy(v.begin(),v.end(),value.begin());};
    load("default_joint_pos",q0);load("kp",kp);load("kd",kd);load("lower",lower);load("upper",upper);
    load("velocity",velocity);load("effort",effort);load("scale",scale);
    auto ids=c["joint_ids_map"].as<std::vector<int>>();
    if(ids.size()!=29)throw std::runtime_error("Invalid joint map size");
    std::copy(ids.begin(),ids.end(),map.begin());std::sort(ids.begin(),ids.end());
    for(int i=0;i<29;i++)if(ids[i]!=i||!std::isfinite(q0[i])||q0[i]<lower[i]||q0[i]>upper[i]||
        kp[i]<=0||kd[i]<0||scale[i]<=0||velocity[i]<=0||effort[i]<=0)
      throw std::runtime_error("Invalid joint map, gains or limits");
    dt=c["policy_dt"].as<float>();servo_dt=c["servo_dt"].as<float>();duration=c["policy_duration_s"].as<float>();
    threshold=c["release_threshold"].as<float>();
    position_tolerance=c["position_failure_tolerance_rad"].as<float>();speed_failure_ratio=c["speed_failure_ratio"].as<float>();
    if(std::abs(dt-.02f)>1e-7||std::abs(servo_dt-.002f)>1e-7||duration<=0||duration>8)
      throw std::runtime_error("Unsupported timing");
    options_.SetIntraOpNumThreads(1);options_.SetInterOpNumThreads(1);
    options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_=std::make_unique<Ort::Session>(environment_,(root+"/policy/policy.onnx").c_str(),options_);
    Ort::AllocatorWithDefaultOptions alloc;
    input_=session_->GetInputNameAllocated(0,alloc).get();output_=session_->GetOutputNameAllocated(0,alloc).get();
    if(session_->GetInputCount()!=1||session_->GetOutputCount()!=1||
       session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape()!=std::vector<int64_t>({1,126})||
       session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape()!=std::vector<int64_t>({1,30}))
      throw std::runtime_error("ONNX dimensions do not match throwing deployment");
    auto metadata=session_->GetModelMetadata().LookupCustomMetadataMapAllocated("joint_names",alloc);
    std::string expected;for(const auto& n:c["joint_names"].as<std::vector<std::string>>())expected+=(expected.empty()?"":",")+n;
    if(!metadata||expected!=metadata.get())throw std::runtime_error("ONNX/config joint order mismatch");
    // Warm up before any DDS command publisher is created.
    Observation zero{};for(int i=0;i<5;i++)infer(zero);
    target=q0;
  }
  void reset(const Measurement& m) {
    check_measurement(m);last.fill(0);released=false;step=0;
    for(int i=0;i<29;i++)target[i]=m.q[map[i]];
  }
  void check_measurement(const Measurement& m) const {
    for(float v:m.q)if(!std::isfinite(v))throw std::runtime_error("Nonfinite encoder position");
    for(float v:m.qd)if(!std::isfinite(v))throw std::runtime_error("Nonfinite encoder velocity");
    for(float v:m.gyro)if(!std::isfinite(v))throw std::runtime_error("Nonfinite IMU gyro");
    float norm=0;for(float v:m.quat)norm+=v*v;
    if(!std::isfinite(norm)||norm<.64f||norm>1.44f)throw std::runtime_error("Invalid IMU quaternion");
  }
  void check_operating_limits(const Measurement& m) const {
    check_measurement(m);
    for(int i=0;i<29;i++) {
      if(m.q[map[i]]<lower[i]-position_tolerance||m.q[map[i]]>upper[i]+position_tolerance)
        throw std::runtime_error("Measured operating angle violation at SDK motor "+std::to_string(map[i]));
      if(std::abs(m.qd[map[i]])>velocity[i]*speed_failure_ratio)
        throw std::runtime_error("Measured operating speed violation at SDK motor "+std::to_string(map[i]));
    }
  }
  Observation observe(const Measurement& m) const {
    check_measurement(m);Observation obs{};
    Eigen::Quaternionf q(m.quat[0],m.quat[1],m.quat[2],m.quat[3]);q.normalize();
    const auto gravity=q.conjugate()*Eigen::Vector3f(0,0,-1);
    for(int i=0;i<3;i++){obs[i]=m.gyro[i];obs[3+i]=gravity[i];}
    for(int i=0;i<29;i++){obs[6+i]=m.q[map[i]]-q0[i];obs[35+i]=m.qd[map[i]];
      obs[97+i]=(target[i]-q0[i])/scale[i];}
    std::copy(last.begin(),last.end(),obs.begin()+64);
    const float t=static_cast<float>(step)*dt;
    obs[94]=released?1.f:0.f;obs[95]=t/8.f;obs[96]=std::sin(2.f*3.14159265358979323846f*t/8.f);
    return obs;
  }
  Action infer(const Observation& obs) {
    for(float v:obs)if(!std::isfinite(v))throw std::runtime_error("Nonfinite policy observation");
    const int64_t shape[]={1,126};auto memory=Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault);
    auto tensor=Ort::Value::CreateTensor<float>(memory,const_cast<float*>(obs.data()),obs.size(),shape,2);
    const char* ins[]={input_.c_str()};const char* outs[]={output_.c_str()};
    auto values=session_->Run(Ort::RunOptions{nullptr},ins,&tensor,1,outs,1);
    Action a{};std::copy_n(values[0].GetTensorData<float>(),30,a.begin());
    for(float v:a)if(!std::isfinite(v))throw std::runtime_error("Nonfinite policy action");
    return a;
  }
  bool accept(const Action& a) {
    for(float v:a)if(!std::isfinite(v))throw std::runtime_error("Nonfinite policy action");
    for(int i=0;i<29;i++) {
      const float request=std::clamp(q0[i]+scale[i]*a[i],lower[i],upper[i]);
      const float max_delta=velocity[i]*dt;
      target[i]+=std::clamp(request-target[i],-max_delta,max_delta);
    }
    const bool edge=!released&&a[29]>threshold;released|=edge;last=a;++step;return edge;
  }
  bool update(const Measurement& m) {return accept(infer(observe(m)));}
  Joints torque(const Measurement& m,const Joints& desired,const Joints& p,const Joints& d) const {
    Joints tau{};
    for(int i=0;i<29;i++) {
      const float v=std::clamp(m.qd[map[i]],-2*velocity[i],2*velocity[i]);
      const float hi=std::min(effort[i]*(1-v/velocity[i]),effort[i]);
      const float lo=std::max(effort[i]*(-1-v/velocity[i]),-effort[i]);
      tau[i]=std::clamp(p[i]*(desired[i]-m.q[map[i]])-d[i]*m.qd[map[i]],lo,hi);
    }
    return tau;
  }
  Joints torque(const Measurement& m) const {return torque(m,target,kp,kd);}
};
} // namespace throwing
