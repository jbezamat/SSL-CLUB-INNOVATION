#include "navigation_with_obstacle_avoidance.h"

namespace RhobanSSL {
namespace Robot_behavior {

Navigation_with_obstacle_avoidance::Navigation_with_obstacle_avoidance( Ai::AiData & ai_data ):
    ai_data(ai_data), position(0.0, 0.0), angle(0.0)
{
} 

void Navigation_with_obstacle_avoidance::set_following_position(
    const Vector2d & position_to_follow,
    const ContinuousAngle & angle
){
    this->position = position_to_follow;
    this->angle = angle;
    this->angle = this->robot_angular_position;
    this->angle.set_to_nearest(angle); 
}

void Navigation_with_obstacle_avoidance::determine_the_closest_obstacle(){

}

bool Navigation_with_obstacle_avoidance::do_we_activate_obstacle_avoidance(){
    return true;
}

void Navigation_with_obstacle_avoidance::compute_the_radius_of_limit_cycle(){
}

void Navigation_with_obstacle_avoidance::compute_the_limit_cycle_direction(){
}

void Navigation_with_obstacle_avoidance::convert_cycle_direction_to_linear_and_angular_velocity(){
}

void Navigation_with_obstacle_avoidance::convert_position_error_to_linear_and_angular_velocity(){
}


void Navigation_with_obstacle_avoidance::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    // At First, we update time and update potition from the abstract class robot_behavior.
    // DO NOT REMOVE THAT LINE
    RobotBehavior::update_time_and_position( time, robot, ball );
    // Now 
    //  this->robot_linear_position
    //  this->ball_position
    //  this->robot_angular_position 
    // are all avalaible

    determine_the_closest_obstacle();
    if( do_we_activate_obstacle_avoidance() ){
        compute_the_radius_of_limit_cycle();
        compute_the_limit_cycle_direction();
        convert_cycle_direction_to_linear_and_angular_velocity();
    }else{
        convert_position_error_to_linear_and_angular_velocity();
    }   
}

Control Navigation_with_obstacle_avoidance::control() const {
    Control ctrl;
    return ctrl;
}

}
}