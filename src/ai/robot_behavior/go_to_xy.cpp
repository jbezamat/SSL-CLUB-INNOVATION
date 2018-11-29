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

#include "go_to_xy.h"
#include <math/vector2d.h>

namespace RhobanSSL {
namespace Robot_behavior {

#define PERIOD 10.0

GoToXY::GoToXY(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data),
    follower( Factory::fixed_consign_follower(ai_data) ),
    x(0),
    y(0)
    {
}

void GoToXY::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    // At First, we update time and update potition from the abstract class robot_behavior.
    // DO NOT REMOVE THAT LINE
    RobotBehavior::update_time_and_position( time, robot, ball );
    
    annotations.clear();
    
    
    const rhoban_geometry::Point & robot_position = robot.get_movement().linear_position( ai_data.time );

    // On ajoute un text à la position du robot.
    //annotations.addText("Exemple :)", robot_position, "blue");

    Vector2d direction = ball_position() - robot_position;
    ContinuousAngle target_rotation = vector2angle( direction );

    rhoban_geometry::Point target = rhoban_geometry::Point(x,y);
    
    follower->set_following_position( target, target_rotation );

    follower->avoid_the_ball(false);
    follower->update(time, robot, ball);
}

Control GoToXY::control() const {
    Control ctrl = follower->control();
    // ctrl.spin = true; // We active the dribler !
    ctrl.kick = false; 
    return ctrl; 
}

void GoToXY::setX( int val ){
    x = val;
}
void GoToXY::setY( int val ){
    y = val;
}


GoToXY::~GoToXY(){
    delete follower;
}




RhobanSSLAnnotation::Annotations GoToXY::get_annotations() const {
    RhobanSSLAnnotation::Annotations annotations;
    annotations.addAnnotations( this->annotations );
    annotations.addAnnotations( follower->get_annotations() );
    return annotations;
}



}
}
