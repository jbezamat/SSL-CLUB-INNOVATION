/*
	This file is part of SSL.

	Copyright 2018 ROMAINPC (romainpc.lechat@laposte.net)

	SSL is free software: you can redistribute it and/or modify
	it under the terms of the GNU Lesser General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	SSL is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public License
	along with SSL.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "RPC_Strat.h"
#include <robot_behavior/position_follower.h>
#include <robot_behavior/go_to_xy.h>

namespace RhobanSSL {
namespace Strategy {

HighFive::HighFive(Ai::AiData & ai_data):
	Strategy(ai_data),
      go_to_xy(std::shared_ptr<Robot_behavior::GoToXY>(
        new Robot_behavior::GoToXY(ai_data)
      ))
{
}

HighFive::~HighFive(){
}

/*
 * We define the minimal number of robot in the field.
 * The goalkeeper is not counted.
 */
int HighFive::min_robots() const {
	return 5;
}

/*
 * We define the maximal number of robot in the field.
 * The goalkeeper is not counted.
 */
int HighFive::max_robots() const {
	return 5;
}

Goalie_need HighFive::needs_goalie() const {
	return Goalie_need::NO;
}

const std::string HighFive::name = "HighFive";

void HighFive::start(double time){
	DEBUG("START PREPARE KICKOFF");
	behaviors_are_assigned = false;
}
void HighFive::stop(double time){
	DEBUG("STOP PREPARE KICKOFF");
}

void HighFive::update(double time){
}

void HighFive::assign_behavior_to_robots(
  std::function<
  void (int, std::shared_ptr<Robot_behavior::RobotBehavior>)
  > assign_behavior,
  double time, double dt
){
  //we assign now all the other behavior
  	assert( get_player_ids().size() == 5 );
  

	/*
	*					1				2	
	*
	*  0						3
	* goal
	*					4				5
	*/



	//cas par défaut:
	DEBUG("ok--------------------------------------------");
/*
	Robot_behavior::PositionFollower* follower = new Robot_behavior::PositionFollower(ai_data, time, dt);
            follower->set_following_position(
                ball_position(),
                0
            );*/
	//Robot_behavior::GoToXY* go_to_xy = new Robot_behavior::GoToXY(ai_data, time, dt);	
	assign_behavior (player_id(0), go_to_xy);
	assign_behavior (player_id(1), go_to_xy);
	assign_behavior (player_id(2), go_to_xy);
	assign_behavior (player_id(3), go_to_xy);
	assign_behavior (player_id(4), go_to_xy);



	/*
  int nearest_ballID = get_nearest_ball(Vision::Team::Ally);

  int id_to_obstruct = id_threat_max( Vision::Team::Opponent );
  obstructeur->declare_robot_to_obstruct(id_to_obstruct, Vision::Team::Opponent);
  if ( nearest_ballID == robotID ) {
	assign_behavior( robotID, degageur );
  }
  else{
	assign_behavior( robotID, obstructeur );
  }

  behaviors_are_assigned = true;*/
}

// We declare here the starting positions that are used to :
//   - place the robot during STOP referee state
//   - to compute the robot order of get_player_ids(),
//     we minimize the distance between
//     the startings points and all the robot position, just
//     before the start() or during the STOP referee state.
std::list<
	std::pair<rhoban_geometry::Point,ContinuousAngle>
> HighFive::get_starting_positions( int number_of_avalaible_robots ){
	assert( min_robots() <= number_of_avalaible_robots );
	assert(
		max_robots()==-1 or
		number_of_avalaible_robots <= max_robots()
	);

	return {
		std::pair<rhoban_geometry::Point,ContinuousAngle>(
			ally_goal_center(),
			0.0
		)
	};
}

//
// This function return false if you don't want to
// give a staring position. So the manager will chose
// a default position for you.
//
bool HighFive::get_starting_position_for_goalie(
	rhoban_geometry::Point & linear_position,
	ContinuousAngle & angular_position
){
	linear_position =  ally_goal_center();
	angular_position = ContinuousAngle(0.0);
	return true;
}

RhobanSSLAnnotation::Annotations HighFive::get_annotations() const {
	RhobanSSLAnnotation::Annotations annotations;

	for (auto it = this->get_player_ids().begin(); it != this->get_player_ids().end(); it++)
	{
		const rhoban_geometry::Point & robot_position = get_robot(*it).get_movement().linear_position( time() );
		//annotations.addText("Behaviour: " + this->name, robot_position.getX() + 0.15, robot_position.getY(), "white");
		annotations.addText("Strategy: " + this->name, robot_position.getX() + 0.15, robot_position.getY() + 0.30, "white");
	}
	return annotations;
}

}
}
