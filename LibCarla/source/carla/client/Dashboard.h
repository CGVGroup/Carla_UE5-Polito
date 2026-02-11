

#pragma once

#include "carla/Debug.h"
#include "carla/Memory.h"
#include "carla/client/detail/simulator.h"

namespace carla{
  namespace client {

	  class Dashboard : public Actor {
	  public:

		  // explicit Dashboard(ActorInitializer initializer);
		  void Dashboard::SetControlValues(float speed, float steer) {
			  GetEpisode().Lock()->SetControlValuesDashboard(speed, steer);
		  }

		  /*
		  private :
		  float variable;
		  */
	  };

  }
}