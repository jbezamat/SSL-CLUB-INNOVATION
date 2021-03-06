/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

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

#ifndef __STRATEGY__STRATEGY__H__
#define __STRATEGY__STRATEGY__H__

#include <game_informations.h>
#include <robot_behavior/robot_behavior.h>
#include <map>
#include <utility>
#include <math/ContinuousAngle.h>
#include <memory>
#include <AiData.h>
#include <annotations/Annotations.h>
namespace RhobanSSL {
namespace Strategy {

enum Goalie_need {
    YES,
    IF_POSSIBLE,
    NO
};

class Strategy : public GameInformations {
    protected:
    Ai::AiData & ai_data;
    private:
    int goalie_id;
    bool manage_a_goalie;

    int goalie_opponent_id;
    std::vector<int> player_ids;

    void update_player_ids();

    public:

    Strategy(Ai::AiData & ai_data);

    virtual void set_goalie( int id, bool to_be_managed );
    void set_goalie_opponent( int id );
    
    // Get the goalie id. If id<0 then no goalie is declared 
    int get_goalie() const;   
    // Get the opponent goalie id. If id<0 then no opponent goalie is declared  
    int get_goalie_opponent() const;   

    /* 
     * This function is called by the manager to affect robot to the stratgey.
     * Here only robot for field player are affected.
     * The robot id for the goal can only be obtained from the function get_goalie().
     * 
     * IMPORTANT : 
     * If you overload this function do not forgive to call this base function.
     * The implementation of set_robot_affectation, is important to calculate 
     * get_player_ids(). 
     */
    virtual void set_robot_affectation( const std::vector<int> & robot_ids );
    
    const std::vector<int> & get_player_ids() const;

    int robot_id( int id ) const;
    int player_id( int id ) const;


    virtual void update(double time){};

    virtual void start(double time){};
    virtual void stop(double time){};
    virtual void pause(double time){};
    virtual void resume(double time){};

    /* 
     * Return a list of position where it is recommended to place a 
     * robot before starting the strategy.
     * the size of the list should be smaller than the parameter 
     * number_of_avalaible_robots.
     */
    virtual std::list<
        std::pair<rhoban_geometry::Point,ContinuousAngle>
    > get_starting_positions( int number_of_avalaible_robots ) const;  
    /* 
     * Set the position where it is recommended to place a goalie 
     * before starting the strategy.
     * If this function return false, then no position is given for a goale.
     * If the strategy have no goalie, this function have to return false.
     */
    virtual bool get_starting_position_for_goalie(
        rhoban_geometry::Point & linear_position, 
        ContinuousAngle & angular_position
    ) const;  

    /*
     * This function give the minimal numer of non goalie robot 
     * that the strategy commands.
     */
    virtual int min_robots() const = 0;
    /*
     * This function give the maximal numer of non goalie robot 
     * that strategy commands.
     * if it is set to -1, then the strategy can command any number of robot.
     */
    virtual int max_robots() const = 0;

    /*
     * Say if the strategy need a goalie.
     */
    virtual Goalie_need needs_goalie() const = 0;

    virtual void assign_behavior_to_robots(
        std::function<
            void (int, std::shared_ptr<Robot_behavior::RobotBehavior>)
        > assign_behavior,
        double time, double dt
    ) = 0;

    //
    // This function is used to draw annotations in the viewer.
    // You can use it to print what you want.
    //
    // For example :
    //
    // 
    //  RhobanSSLAnnotation::Annotations get_annotations() const{
    //      RhobanSSLAnnotation::Annotations annotations;
    //      static double d = 0;
    //      d += 0.01;
    //
    //      annotations.addCircle(3, 3, 1, "cyan");
    //      annotations.addArrow(0, 0, cos(d), sin(d)*2, "magenta", true);
    //      return annotations;
    //  }
    virtual RhobanSSLAnnotation::Annotations get_annotations() const;

    virtual ~Strategy();

    /*
     * This function return true when the 
     * the strategie have to manage the 
     * goalie.
     */
    bool have_to_manage_the_goalie() const;

};

};
};
#endif
