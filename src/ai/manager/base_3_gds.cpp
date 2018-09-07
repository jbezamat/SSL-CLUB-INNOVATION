/*
    This file is part of SSL.

    Copyright 2018 Name Surname (mail)

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

#include "base_3_gds.h"

// The different strategies
#include <strategy/halt.h>
#include <strategy/tare_and_synchronize.h>
#include <strategy/placer.h>
#include <strategy/prepare_kickoff.h>
#include <strategy/base.h>
#include <strategy/base_3_gds.h>
#include <strategy/base_3_gms.h>
#include <strategy/from_robot_behavior.h>
#include <robot_behavior/goalie.h>
#include <robot_behavior/defensor.h>
#include <robot_behavior/mur_defensor.h>
#include <robot_behavior/striker.h>
#include <core/collection.h>
#include <core/print_collection.h>

#define GOALIE "Goalie" 
#define DEFENSOR1 "Defensor1" 
#define DEFENSOR2 "Defensor2" 
#define STRIKER "Striker"
#define MUR_DEFENSOR "Mur_defensor"

namespace RhobanSSL {
namespace Manager {

Base_3_gds::Base_3_gds(
    Ai::AiData & ai_data,
    const Referee & referee
):
    Manager(ai_data),
    referee(referee),
    last_referee_changement(0)
{

    register_strategy(
        Strategy::Halt::name, std::shared_ptr<Strategy::Strategy>(
            new Strategy::Halt(ai_data) 
        )
    );
    register_strategy(
        Strategy::Tare_and_synchronize::name,
        std::shared_ptr<Strategy::Strategy>( 
            new Strategy::Tare_and_synchronize(ai_data)
        )
    );
    register_strategy(
        Strategy::Prepare_kickoff::name,
        std::shared_ptr<Strategy::Strategy>(
            new Strategy::Prepare_kickoff(ai_data)
        )
    );
    register_strategy(
        Strategy::Base::name,
        std::shared_ptr<Strategy::Strategy>(
            new Strategy::Base(ai_data)
        )
    );
    register_strategy(
        Strategy::Base_3_gds::name, std::shared_ptr<Strategy::Strategy>(
            new Strategy::Base_3_gds(ai_data) 
        )
    );
    register_strategy(
        GOALIE, std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Goalie* goalie = new Robot_behavior::Goalie(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(goalie);
                }, true //it is a goal
            )
        )
    );
    register_strategy(
        DEFENSOR1, std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Defensor* defensor = new Robot_behavior::Defensor(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(defensor);
                }, false // it is not a goal
            )
        )
    );
    register_strategy(
        DEFENSOR2, std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Defensor* defensor = new Robot_behavior::Defensor(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(defensor);
                }, false // it is not a goal
            )
        )
    );
    register_strategy(
        STRIKER, std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Striker* striker = new Robot_behavior::Striker(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(striker);
                }, false // it is not a goal
            )
        )
    );
    register_strategy(
        MUR_DEFENSOR, std::shared_ptr<Strategy::Strategy>(
            new Strategy::From_robot_behavior(
                ai_data,
                [&](double time, double dt){
                    Robot_behavior::Mur_defensor* mur_defensor = new Robot_behavior::Mur_defensor(ai_data);
                    return std::shared_ptr<Robot_behavior::RobotBehavior>(mur_defensor);
                }, false // it is not a goal
            )
        )
    );
    assign_strategy(
        Strategy::Halt::name, 0.0, 
        get_team_ids()
    ); // TODO TIME !
}


void Base_3_gds::choose_a_strategy(double time){
    if( referee.edge_entropy() > last_referee_changement ){
        clear_strategy_assignement();
        if( referee.get_state() == Referee_Id::STATE_INIT ){
        } else if( referee.get_state() == Referee_Id::STATE_HALTED ){
            assign_strategy( Strategy::Halt::name, time, get_valid_team_ids() );
        } else if( referee.get_state() == Referee_Id::STATE_STOPPED ){
            if(get_valid_team_ids().size() > 0){
                if( not( get_strategy_<Strategy::Tare_and_synchronize>().is_tared_and_synchronized() ) ){
                    assign_strategy( Strategy::Tare_and_synchronize::name, time, get_valid_player_ids() );
                }else{
                    place_all_the_robots(time, future_strats);
                }
            }
        } else if( referee.get_state() == Referee_Id::STATE_PREPARE_KICKOFF ){
            if( get_team() == referee.kickoff_team() ){
                get_strategy_<Strategy::Prepare_kickoff>().set_kicking(true);
            }else{
                get_strategy_<Strategy::Prepare_kickoff>().set_kicking(false);
            }
            future_strats = {Strategy::Prepare_kickoff::name};
            declare_and_assign_next_strategies( future_strats );
        } else if( referee.get_state() == Referee_Id::STATE_PREPARE_PENALTY ){
        } else if( referee.get_state() == Referee_Id::STATE_RUNNING ){
            //future_strats = { GOALIE, MUR_DEFENSOR };
            future_strats = { Strategy::Base_3_gds::name };
            declare_and_assign_next_strategies(future_strats);
        } else if( referee.get_state() == Referee_Id::STATE_TIMEOUT ){
            assign_strategy( Strategy::Halt::name, time, get_valid_team_ids() );
        }
        last_referee_changement = referee.edge_entropy();
    }
}

void Base_3_gds::update(double time){
    //update_strategies(time);
    update_current_strategies(time);
    choose_a_strategy(time);
}

Base_3_gds::~Base_3_gds(){ }

};
};