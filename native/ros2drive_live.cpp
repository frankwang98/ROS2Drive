#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include "planning/reference_path_planner.hpp"
#include "scenario/agriculture_route_scenario.hpp"
#include "scenario/mining_haul_scenario.hpp"
#include "scenario/port_transport_scenario.hpp"
#include "scenario/ring_scenario.hpp"
#include "simulation/simulation_engine.hpp"

using namespace sdc;
static scenario::ScenarioDefinition sceneFor(const std::string& name) {
  if(name=="ring_demo") return scenario::makeRingScenarioDefinition();
  if(name=="mining_haul") return scenario::makeMiningHaulScenarioDefinition();
  if(name=="port_transport") return scenario::makePortTransportScenarioDefinition();
  if(name=="agriculture_route") return scenario::makeAgricultureRouteScenarioDefinition();
  throw std::invalid_argument("unknown scenario");
}
static const char* runtimeName(domain::RuntimeState s) {
  switch(s) {
    case domain::RuntimeState::kInit:return "INIT";
    case domain::RuntimeState::kReady:return "READY";
    case domain::RuntimeState::kRunning:return "RUNNING";
    case domain::RuntimeState::kPaused:return "PAUSED";
    case domain::RuntimeState::kDegraded:return "DEGRADED";
    case domain::RuntimeState::kStopped:return "STOPPED";
    case domain::RuntimeState::kFault:return "FAULT";
    case domain::RuntimeState::kEstop:return "ESTOP";
  }
  return "UNKNOWN";
}
static const char* missionName(domain::MissionState s) {
  switch(s) {
    case domain::MissionState::kPending:return "PENDING";
    case domain::MissionState::kActive:return "ACTIVE";
    case domain::MissionState::kPaused:return "PAUSED";
    case domain::MissionState::kSucceeded:return "SUCCEEDED";
    case domain::MissionState::kFailed:return "FAILED";
    case domain::MissionState::kCanceled:return "CANCELED";
  }
  return "UNKNOWN";
}
static void ack(bool ok) {
  std::cout<<"{\"kind\":\"ack\",\"accepted\":"<<(ok?"true":"false")<<"}"<<std::endl;
}
int main(int argc,char** argv) {
  try {
    std::string scenario_id="port_transport";
    double dt=0.05;
    for(int i=1;i<argc;++i) {
      const std::string key=argv[i];
      if(i+1>=argc) throw std::invalid_argument("missing argument");
      if(key=="--scenario")scenario_id=argv[++i];
      else if(key=="--dt")dt=std::stod(argv[++i]);
      else throw std::invalid_argument("unknown argument: "+key);
    }
    if(!std::isfinite(dt)||dt<=0||dt>1)throw std::invalid_argument("invalid dt");
    const auto scene=sceneFor(scenario_id);
    auto engine=std::make_unique<simulation::SimulationEngine>(
      std::make_unique<planning::ReferencePathPlanner>());
    engine->reset(scene.initial_pose.x,scene.initial_pose.y,scene.initial_pose.yaw);
    engine->setObstacles(scene.static_obstacles);
    std::string reason;
    auto default_mission=scene.materializeDefaultMission(reason);
    if(!engine->setMission(default_mission))throw std::runtime_error("default mission rejected");
    std::cout<<std::setprecision(9);
    std::string line;
    while(std::getline(std::cin,line)) {
      std::istringstream input(line);
      std::string cmd;
      input>>cmd;
      if(cmd=="STEP") {
        const auto result=engine->step(dt);
        const auto& v=engine->vehicle();
        const auto& m=engine->runtime().missions().current();
        std::cout<<"{\"kind\":\"frame\",\"t\":"<<engine->simulationTime()
          <<",\"x\":"<<v.x()<<",\"y\":"<<v.y()<<",\"yaw\":"<<v.yaw()
          <<",\"speed\":"<<v.speed()<<",\"steering\":"<<v.steer()
          <<",\"runtime_state\":\""<<runtimeName(result.state)<<"\""
          <<",\"mission_id\":\""<<(m?m->id:"")<<"\""
          <<",\"mission_state\":\""<<(m?missionName(m->state):"UNKNOWN")<<"\""
          <<",\"mission_progress\":"<<(m?m->progress:0)
          <<",\"planning_ok\":"<<(result.planning.success?"true":"false")
          <<",\"trajectory\":[";
        const auto& points=result.planning.trajectory.points;
        for(std::size_t i=0;i<points.size();++i) {
          if(i)std::cout<<",";
          std::cout<<"{\"x\":"<<points[i].pose.x<<",\"y\":"<<points[i].pose.y
            <<",\"theta\":"<<points[i].pose.yaw<<"}";
        }
        std::cout<<"]}"<<std::endl;
      } else if(cmd=="MISSION") {
        domain::Mission m;
        int type=1,preempt=0,n=0;
        if(!(input>>m.id>>type>>m.speed_limit>>m.goal_tolerance>>m.timeout_s>>preempt>>n)
           ||m.id.empty()||m.id.size()>128||type<0||type>5||n<1||n>5000
           ||!std::isfinite(m.speed_limit)||m.speed_limit<=0||m.speed_limit>10
           ||!std::isfinite(m.goal_tolerance)||m.goal_tolerance<=0
           ||!std::isfinite(m.timeout_s)||m.timeout_s<0) {ack(false);continue;}
        m.type=static_cast<domain::MissionType>(type);
        bool valid=true;
        for(int i=0;i<n;++i) {
          if(!std::getline(std::cin,line)){valid=false;break;}
          std::istringstream point(line);
          domain::Pose2D p;
          if(!(point>>p.x>>p.y>>p.yaw)||!std::isfinite(p.x)||!std::isfinite(p.y)
             ||!std::isfinite(p.yaw))valid=false;
          m.route.push_back(p);
        }
        ack(valid&&engine->setMission(std::move(m),preempt!=0));
      } else if(cmd=="CANCEL")ack(engine->cancelMission());
      else if(cmd=="PAUSE")ack(engine->pause());
      else if(cmd=="RESUME")ack(engine->resume());
      else if(cmd=="ESTOP") {
        int enabled=-1;
        if(input>>enabled&&(enabled==0||enabled==1)){
          engine->requestEmergencyStop(enabled==1);ack(true);
        } else ack(false);
      } else if(cmd=="RECOVER")ack(engine->acknowledgeSafetyRecovery());
      else if(cmd=="QUIT")break;
      else ack(false);
    }
  }catch(const std::exception& e) {
    std::cerr<<"ros2drive_live: "<<e.what()<<std::endl;
    return 1;
  }
}
