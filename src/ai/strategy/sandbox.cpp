#include "sandbox.h"

#include <robot_behavior/position_follower.h>
#include <robot_behavior/goalie.h>
#include <robot_behavior/shooter.h>
#include <robot_behavior/do_nothing.h>

namespace RhobanSSL {
namespace Strategy {

#ifdef SSL_SIMU
const int TeamId::goalie_id = 5; 
const int TeamId::shooter_id = 0;
const int TeamId::follower_id = 3;
#else
const int TeamId::goalie_id = 8; 
const int TeamId::shooter_id = 5;
const int TeamId::follower_id = 3;
#endif

const std::string Sandbox::name="sandbox";

Sandbox::Sandbox(Ai::AiData & game_state):
    game_state(game_state)
{
}

void Sandbox::start(double time){
    DEBUG("START SANDBOX");
    behavior_has_been_assigned = false;
}

void Sandbox::stop(double time){
    DEBUG("STOP SANDBOX");
}

void Sandbox::assign_behavior_to_robots(
    std::map<
        int, 
        std::shared_ptr<RobotBehavior>
    > & robot_behaviors,
    double time, double dt
){
    if( ! behavior_has_been_assigned ){
        for(
            std::pair<int, std::shared_ptr<RobotBehavior> > elem : 
            robot_behaviors 
        ){
            robot_behaviors[ elem.first ] = std::shared_ptr<RobotBehavior>(
                new DoNothing()
            );
        }

        DEBUG("SANDBOX ASSIGN BEHAVIOR");
        PositionFollower* follower = new PositionFollower(
            time, dt
        );
        const Ai::Robot & robot_follower = game_state.robots[
            Vision::Ally
        ][TeamId::follower_id];
        Eigen::Vector2d follower_position(
            robot_follower.get_movement().linear_position(time).getX(),
            robot_follower.get_movement().linear_position(time).getY()
        );
        follower->set_following_position(
            follower_position, ContinuousAngle(M_PI/2.0)
        );
        follower->set_translation_pid(
            game_state.constants.p_translation,
            game_state.constants.i_translation, 
            game_state.constants.d_translation
        );
        follower->set_orientation_pid(
            game_state.constants.p_orientation, game_state.constants.i_orientation, 
            game_state.constants.d_orientation
        );
        follower->set_limits(
            game_state.constants.translation_velocity_limit,
            game_state.constants.rotation_velocity_limit
        );
        robot_behaviors[TeamId::follower_id] = std::shared_ptr<
            RobotBehavior
        >( follower ); 

        // We create a goalie :    
        Goalie* goalie = new Goalie(
            game_state.constants.left_post_position, 
            game_state.constants.right_post_position, 
            game_state.constants.waiting_goal_position, 
            game_state.constants.penalty_rayon, 
            game_state.constants.robot_radius,
            time, dt
        );
        goalie->set_translation_pid( 
            game_state.constants.p_translation,
            game_state.constants.i_translation, 
            game_state.constants.d_translation
        );
        goalie->set_orientation_pid(
            game_state.constants.p_orientation,
            game_state.constants.i_orientation, 
            game_state.constants.d_orientation
        );
        goalie->set_limits(
            game_state.constants.translation_velocity_limit,
            game_state.constants.rotation_velocity_limit
        );
        robot_behaviors[TeamId::goalie_id] = std::shared_ptr<
            RobotBehavior
        >( goalie ); 

        #if 1
        // We create a shooter :
        Shooter* shooter = new Shooter(
            game_state.constants.goal_center,
            game_state.constants.robot_radius,
            game_state.constants.front_size,
            game_state.constants.radius_ball,
            game_state.constants.translation_velocity,
            game_state.constants.translation_acceleration,
            game_state.constants.angular_velocity,
            game_state.constants.angular_acceleration,
            game_state.constants.calculus_step,
            time, dt
        );
        shooter->set_translation_pid(
            game_state.constants.p_translation,
            game_state.constants.i_translation, 
            game_state.constants.d_translation
        );
        shooter->set_orientation_pid(
            game_state.constants.p_orientation,
            game_state.constants.i_orientation, 
            game_state.constants.d_orientation
        );
        shooter->set_limits(
            game_state.constants.translation_velocity_limit,
            game_state.constants.rotation_velocity_limit
        );
        const Ai::Ball & ball = game_state.ball;
        Eigen::Vector2d ball_position(
            ball.get_movement().linear_position(time).getX(),
            ball.get_movement().linear_position(time).getY()
        );
        const Ai::Robot & robot = game_state.robots[Vision::Ally][TeamId::shooter_id];
        Eigen::Vector2d robot_position(
            robot.get_movement().linear_position(time).getX(),
            robot.get_movement().linear_position(time).getY()
        );
        double robot_orientation(
            robot.get_movement().angular_position(time).value()
        );
        shooter->go_to_shoot( 
            ball_position, robot_position, robot_orientation, 
            time, dt 
        );
        robot_behaviors[TeamId::shooter_id] = std::shared_ptr<
            RobotBehavior
        >( shooter ); 
        #endif

        behavior_has_been_assigned = true;
    }
}

Sandbox::~Sandbox(){
}

}
}
