/*
    This file is part of SSL.

    Copyright 2018 TO COMPLETE 

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

#include "striker_ai.h"
#include <math/tangents.h>
#include <math/vector2d.h>

namespace RhobanSSL {
namespace Robot_behavior {


StrikerAi::StrikerAi(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data),
    follower( Factory::fixed_consign_follower(ai_data) )
{
}

void StrikerAi::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    // At First, we update time and update potition from the abstract class robot_behavior.
    // DO NOT REMOVE THAT LINE
    RobotBehavior::update_time_and_position( time, robot, ball );
    // Now 
    //  this->robot_linear_position
    //  this->robot_angular_position 
    // are all avalaible
    
    const rhoban_geometry::Point & robot_position = robot.get_movement().linear_position( ai_data.time );
    
    std::pair<rhoban_geometry::Point, double> results = GameInformations::find_goal_best_move( ball_position() );
    rhoban_geometry::Point goal_point = results.first;


    Vector2d ball_goal_vector = goal_point - ball_position();
    Vector2d ball_robot_vector = robot_position - ball_position();
    double dist_ball_robot = ball_robot_vector.norm();

    ball_goal_vector = ball_goal_vector / ball_goal_vector.norm();
    ball_robot_vector = ball_robot_vector / ball_robot_vector.norm();

    double target_radius_from_ball;
    double scalar_ball_robot = - scalar_product( ball_robot_vector , ball_goal_vector );
    

    if ( scalar_ball_robot < 0 ) {
        follower->avoid_the_ball(true);
        target_radius_from_ball = 0.4;
    } else {
        follower->avoid_the_ball(false);
        //target_radius_from_ball = 1.0 / ( 4.0*(scalar_ball_robot - 1.2) ) + 1.0;
        target_radius_from_ball = 1.0 / ( 24.0*(scalar_ball_robot - 1.04) ) + 0.44;


        if ( dist_ball_robot < 0.4 ) {
            follower->avoid_opponent(false);
        } 
    }
    if (dist_ball_robot > 0.4) {
        follower->avoid_opponent(true);
    }

    rhoban_geometry::Point target_position = ball_position() - ball_goal_vector *target_radius_from_ball;
    double target_rotation = detail::vec2angle(ball_goal_vector);

    follower->set_following_position(target_position, target_rotation);
    follower->update(time, robot, ball);   
}

Control StrikerAi::control() const {
    Control ctrl = follower->control();
    ctrl.charge = true;
    ctrl.kick = true; 
    return ctrl; 
}

StrikerAi::~StrikerAi(){
    delete follower;
}

RhobanSSLAnnotation::Annotations StrikerAi::get_annotations() const {
    return follower->get_annotations();
}

}
}
