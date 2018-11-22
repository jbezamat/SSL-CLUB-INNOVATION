/*
    This file is part of SSL.

    Copyright 2018 Bezamat Jérémy (jeremy.bezamat@gmail.com)

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

#ifndef __STRATEGY__HIGHFIVE__H__
#define __STRATEGY__HIGHFIVE__H__

#include "Strategy.h"
#include <robot_behavior/position_follower.h>


namespace RhobanSSL {
namespace Strategy {

class HighFive : public Strategy {
    private:
    bool behaviors_are_assigned;
    std::shared_ptr<Robot_behavior::PositionFollower> position_follower;
    

    public:
    HighFive(Ai::AiData & ai_data);
    virtual ~HighFive();

    virtual int min_robots() const;
    virtual int max_robots() const;
    virtual Goalie_need needs_goalie() const;

    static const std::string name;

    virtual void start(double time);
    virtual void stop(double time);

    virtual void update(double time);

    virtual void assign_behavior_to_robots(
        std::function<
            void (int, std::shared_ptr<Robot_behavior::RobotBehavior>)
        > assign_behavior,
        double time, double dt
    );

    virtual std::list<
        std::pair<rhoban_geometry::Point,ContinuousAngle>
    > get_starting_positions( int number_of_avalaible_robots ) ;
    virtual bool get_starting_position_for_goalie(
        rhoban_geometry::Point & linear_position,
        ContinuousAngle & angular_position
    ) ;

    virtual RhobanSSLAnnotation::Annotations get_annotations() const;


};

};
};
#endif
