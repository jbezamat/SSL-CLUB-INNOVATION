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
	Strategy(ai_data)
{
	
	for(size_t i = 0; i < 5; i++)
	{
		go_to_xy[i] = std::shared_ptr<Robot_behavior::GoToXY>(
        new Robot_behavior::GoToXY(ai_data)
      );
	}
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
  	
  	assert( get_player_ids().size() == 5 );
  

	/*
	*					5				2	
	*
	*  0						3
	* goal
	*					4				1
	*/
	/*NB : taille terrain division B: 9 * 6 mètres 
	*
	* coordonnées : (0,0) -> centre terrain
	*				unité: mètre
	*				Y-down
	*				X-left quand on est à droite du terrain
	*
	*
	*/

	bool followBallMode = true; //quand == false, le 5 garde la meme hauteur et position, il suit juste BallX
	
	double ballX = ball_position().x;
	double ballY = ball_position().y;
	double d = 0;
	if(followBallMode)
		d = 1.5d;
	else
		d = 2;

	//0 -> attanquant droit
	//1 -> attanquant gauche
	//2 -> milieu
	//3 -> defenseur droit
	//4 -> defenseur gauche


	//placement  quand pas d'action spécifique:
	if(!followBallMode){
		
		double translation = ballX;
		double limit = 3.7d;
		
		go_to_xy[0] -> setX(d + translation > limit ? limit : d + translation);
		go_to_xy[0] -> setY(-d);
		assign_behavior (player_id(0), go_to_xy[0]);

		go_to_xy[1] -> setX(d + translation > limit ? limit : d + translation);
		go_to_xy[1] -> setY(d);
		assign_behavior (player_id(1), go_to_xy[1]);

		go_to_xy[2] -> setX(0 + ballX);
		go_to_xy[2] -> setY(0);
		assign_behavior (player_id(2), go_to_xy[2]);

		go_to_xy[3] -> setX(-d + translation < -limit ? -limit : -d + translation);
		go_to_xy[3] -> setY(-d);
		assign_behavior (player_id(3), go_to_xy[3]);

		go_to_xy[4] -> setX(-d + translation < -limit ? -limit : -d + translation);
		go_to_xy[4] -> setY(d);
		assign_behavior (player_id(4), go_to_xy[4]);



	} else {
		double xLimit = 4.3d;
		double yLimit = 2.8d;
		
		rhoban_geometry::Point P0 = polarFromOriginToXY(ballX, ballY, +135, d);
		go_to_xy[0] -> setX(P0.x > xLimit ? xLimit : P0.x);
		go_to_xy[0] -> setY(P0.y < - yLimit ? -yLimit : P0.y);
		assign_behavior (player_id(0), go_to_xy[0]);

		rhoban_geometry::Point P1 = polarFromOriginToXY(ballX, ballY, -135, d);
		go_to_xy[1] -> setX(P1.x > xLimit ? xLimit : P1.x);
		go_to_xy[1] -> setY(P1.y > yLimit ? yLimit : P1.y);
		assign_behavior (player_id(1), go_to_xy[1]);

		go_to_xy[2] -> setX(ballX);
		go_to_xy[2] -> setY(ballY);
		assign_behavior (player_id(2), go_to_xy[2]);

		rhoban_geometry::Point P3 = polarFromOriginToXY(ballX, ballY, 45, d);
		go_to_xy[3] -> setX(P3.x < -xLimit ? -xLimit : P3.x);
		go_to_xy[3] -> setY(P3.y < - yLimit ? -yLimit : P3.y);
		assign_behavior (player_id(3), go_to_xy[3]);

		rhoban_geometry::Point P4 = polarFromOriginToXY(ballX, ballY, -45, d);
		go_to_xy[4] -> setX(P4.x < -xLimit ? -xLimit : P4.x);
		go_to_xy[4] -> setY(P4.y > yLimit ? yLimit : P4.y);
		assign_behavior (player_id(4), go_to_xy[4]);


	}


	
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