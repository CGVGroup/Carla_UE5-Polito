

#pragma once

#include "carla/Debug.h"
#include "carla/Memory.h"
#include "carla/client/detail/simulator.h"

namespace carla{
  namespace client {

	  class Dashboard : public Actor {
	  public:

		 // rpc::Actor Dashboard::GetDashboard() {
		//	  return GetEpisode().Lock()->GetDashboard();
		 // }

		  // explicit Dashboard(ActorInitializer initializer);
		  void Dashboard::SetControlValues(float speed, float steer) {
			  GetEpisode().Lock()->SetControlValuesDashboard(speed, steer);
		  }

		  void Dashboard::SetTypeString(std::string type) {
			  GetEpisode().Lock()->SetTypeStringDashboard(type);
		  }

		  void Dashboard::AddVectorPair(geom::Vector3D Pos, geom::Vector3D Rot) {
			  GetEpisode().Lock()->AddVectorPairDashboard(Pos, Rot);
		  }

		  void Dashboard::RemoveIdObstacle(int Id) {
			  GetEpisode().Lock()->RemoveIdObstacleDashboard(Id);
		  }

		  void Dashboard::UpdateObstacle(int Id, const std::string Type, bool isDanger,  geom::Vector3D  Vector1,  geom::Vector3D Vector2, float obstacleSpeed, float obstacleSteer) {
			  GetEpisode().Lock()->UpdateObstacleDashboard(Id, Type, isDanger, Vector1, Vector2, obstacleSpeed, obstacleSteer);
		  }

		  void Dashboard::Update() {
			  GetEpisode().Lock()->UpdateDashboard();
		  }

		  void Dashboard::TriggerPath() {
			  GetEpisode().Lock()->TriggerPathDashboard();
		  }

		  /*
		  private :
		  float variable;
		  */
	  };

  }
}