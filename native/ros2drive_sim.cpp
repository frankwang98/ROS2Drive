#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include "planning/reference_path_planner.hpp"
#include "scenario/agriculture_route_scenario.hpp"
#include "scenario/mining_haul_scenario.hpp"
#include "scenario/port_transport_scenario.hpp"
#include "scenario/ring_scenario.hpp"
#include "simulation/simulation_engine.hpp"

using sdc::scenario::ScenarioDefinition;
using sdc::domain::Pose2D;

static ScenarioDefinition scenarioFor(const std::string& id) {
  if (id == "ring_demo") return sdc::scenario::makeRingScenarioDefinition();
  if (id == "mining_haul") return sdc::scenario::makeMiningHaulScenarioDefinition();
  if (id == "port_transport") return sdc::scenario::makePortTransportScenarioDefinition();
  if (id == "agriculture_route") return sdc::scenario::makeAgricultureRouteScenarioDefinition();
  throw std::invalid_argument("unknown scenario: " + id);
}
static void pose(std::ostream& out, const Pose2D& p) {
  out << "{\"x\":" << p.x << ",\"y\":" << p.y << ",\"yaw\":" << p.yaw << ",\"z\":" << p.z << "}";
}
static void route(std::ostream& out, const std::vector<Pose2D>& points) {
  out << "[";
  for (std::size_t i=0;i<points.size();++i) {
    if(i) out << ",";
    pose(out,points[i]);
  }
  out << "]";
}
int main(int argc, char** argv) {
  try {
    std::string id="port_transport", output="studio/recording.json", route_file="";
    int steps=800;
    double dt=0.05;
    for(int i=1;i<argc;++i) {
      std::string a=argv[i];
      auto value=[&]() -> std::string {
        if(i+1>=argc) throw std::invalid_argument("missing value for "+a);
        return argv[++i];
      };
      if(a=="--scenario") id=value();
      else if(a=="--output") output=value();
      else if(a=="--steps") steps=std::stoi(value());
      else if(a=="--dt") dt=std::stod(value());
      else if(a=="--help") {
        std::cout << "ros2drive_sim --scenario ring_demo|port_transport|mining_haul|agriculture_route --steps 800 --dt 0.05 --output studio/recording.json\n";
        return 0;
      } else throw std::invalid_argument("unknown option: "+a);
    }
    if(steps<=0 || steps>1000000 || !std::isfinite(dt) || dt<=0 || dt>1)
      throw std::invalid_argument("invalid steps or dt");
    auto scene=scenarioFor(id);
    std::string reason;
    if(!sdc::scenario::validate(scene,reason))
      throw std::runtime_error("invalid scenario: "+reason);
    auto mission=scene.materializeDefaultMission(reason);
    auto engine=std::make_unique<sdc::simulation::SimulationEngine>(
      std::make_unique<sdc::planning::ReferencePathPlanner>());
    engine->reset(scene.initial_pose.x,scene.initial_pose.y,scene.initial_pose.yaw);
    engine->setObstacles(scene.static_obstacles);
    if(!engine->setMission(mission)) throw std::runtime_error("mission rejected");
    std::ofstream out(output);
    if(!out) throw std::runtime_error("cannot open output: "+output);
    out << std::setprecision(9);
    out << "{\"format\":\"ROS2DriveStudioV1\",\"scenario\":\""<<id<<"\",\"dt\":"<<dt<<",\"lanes\":[";
    for(std::size_t i=0;i<scene.road_network.lanes.size();++i) {
      if(i)out<<",";
      const auto& lane=scene.road_network.lanes[i];
      out<<"{\"id\":\""<<lane.id<<"\",\"points\":";
      route(out,lane.centerline);
      out<<"}";
    }
    out << "],\"route\":";
    route(out,mission.route);
    out << ",\"obstacles\":[";
    for(std::size_t i=0;i<scene.static_obstacles.size();++i) {
      if(i)out<<",";
      out<<"{\"radius\":"<<scene.static_obstacles[i].radius<<",\"pose\":";
      pose(out,scene.static_obstacles[i].pose);
      out<<"}";
    }
    out << "],\"frames\":[";
    for(int i=0;i<steps;++i) {
      const auto result=engine->step(dt);
      const auto& vehicle=engine->vehicle();
      if(i)out<<",";
      out<<"{\"t\":"<<engine->simulationTime()<<",\"pose\":";
      pose(out,{vehicle.x(),vehicle.y(),vehicle.yaw(),0.0});
      out<<",\"speed\":"<<vehicle.speed()<<",\"steering\":"<<vehicle.steer()
         <<",\"state\":"<<static_cast<int>(result.state)
         <<",\"planning_ok\":"<<(result.planning.success?"true":"false")
         <<",\"trajectory\":[";
      const auto& points=result.planning.trajectory.points;
      for(std::size_t j=0;j<points.size();++j) {
        if(j)out<<",";
        pose(out,points[j].pose);
      }
      out<<"]}";
    }
    out << "]}\n";
    if(!out)throw std::runtime_error("write failed");
    std::cout<<"Wrote "<<steps<<" frames to "<<output<<"\n";
  } catch(const std::exception& e) {
    std::cerr<<"ros2drive_sim: "<<e.what()<<"\n";
    return 1;
  }
}
