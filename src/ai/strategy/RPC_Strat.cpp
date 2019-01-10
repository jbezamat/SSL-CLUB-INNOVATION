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

#include <AiData.h>

#include <robot_behavior/position_follower.h>
#include <robot_behavior/go_to_xy.h>

#define TIMER_APPROACH 50;
#define TIMER_MIDDLE 200;
#define TIMER_MUR 100;
#define TIMER_DEF 100;

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
	  striker[i] = std::shared_ptr<Robot_behavior::Striker>(
        new Robot_behavior::Striker(ai_data)
      );
	  degageur[i] = std::shared_ptr<Robot_behavior::Degageur>(
        new Robot_behavior::Degageur(ai_data)
      );
	  mur[i] = std::shared_ptr<Robot_behavior::Mur_defensor>(
        new Robot_behavior::Mur_defensor(ai_data)
      );
	}
	timerApproach = 0;
	approachM = false;
	approachAD = false;
	approachAG = false;
	timerMiddle = 0;
	for(size_t i = 0; i < 5; i++)
	{
		middle[i] = false;
	}
	degMurD = false;
	degMurG = false;
	timerMur = 0;
	degDefM = false;
	degDefAD = false;
	degDefAG = false;
	timerDef = 0;
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

	/*
	*ATTAQUE :
	* le milieu à un couloir de 1/3 de la hauteur, chaque ailier aussi.
	* En gros si le tir est possible => striker dès qu'on atteint a balle 
	* Sinon les aliés font la passe au milieu et le milieu aux ailers
	*
	* Si la balle est hors de portée de M, et est sur un bandeau latéral,
	* les attaquants peuvent reculer pour la récupérer (puis tir ou passe)
	* 
	*
	* si la balle est vraiment hors de portée,
	*	les défenseurs peuvent rapidemment avancer pour envoyer la balle
	* au milieu ou à l'attaquant devant.
	* 
	* MILIEU :
	* Au milieu on passe en mode followballMode très rapproché, quand un des robots atteint la balle,
	* il peut la dégager vers l'attaque.
	* Un booleen timer doit empecher que tous se jettent dessus, priorité au milieu. 
	*
	*	DEFENSE: 
	*   2 murs dégaeurs et 3 dégaeurs, avec tout le couloir latéral  gérer pour les attaquants
	*/


	//0 -> attanquant droit
	//1 -> attanquant gauche
	//2 -> milieu
	//3 -> defenseur droit
	//4 -> defenseur gauche


	


	//settings
	double ballX = ball_position().x;
	double ballY = ball_position().y;
	bool followBallMode = false; //quand == false, le 5 garde la meme hauteur et position, il suit juste BallX
	if(ballX <= 1 && ballX >= -1){
		followBallMode = true;
	}
	double d = 0;
	if(followBallMode)
		d = 1;
	else
		d = 2;

	//double cote = 1.20;//0.80;
	/*Box zoneApproche = Box(
                    {ballX - cote, ballY - cote},
                     {ballX + cote, ballY + cote});*/
	double seuilApproche = 1.50;

	double seuilBandeau = 0.4;
	Vision::Team ennemis = Vision::Team::Opponent;

	


	//#######   placement par défaut:   #####################################################################
	if(!followBallMode){
		
		double translation = ballX;
		double limit = 3.7;
		
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
		double xLimit = 4.3;
		double yLimit = 2.8;
		
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



	rhoban_geometry::Point coordM = get_robot(player_id(2)).get_movement().linear_position( time );
	rhoban_geometry::Point coordAD = get_robot(player_id(0)).get_movement().linear_position( time );
	rhoban_geometry::Point coordAG = get_robot(player_id(1)).get_movement().linear_position( time );
	rhoban_geometry::Point coordDD = get_robot(player_id(3)).get_movement().linear_position( time );
	rhoban_geometry::Point coordDG = get_robot(player_id(4)).get_movement().linear_position( time );


	//#######   comportement offensif   #####################################################################
		double distAttDefD = coordAD.x - coordDD.x;
		if(distAttDefD < 0) distAttDefD = 0;
		double distAttDefG = coordAG.x - coordDG.x;
		if(distAttDefG < 0) distAttDefG = 0;

		//=====>   millieu:
		double y = coordM.y;
		double x, deltaX;
		//condition d'attaque:
		if(ballX > 1){
			
			//condition de zone: (bandeau 1/3)
			if( (y <= 1 && y >= -1 ) || approachM){
				//condition d'approche:
				if(distBetween(coordM, ball_position()) <= seuilApproche && !approachAG && !approachAD){
					
					std::vector<int> bandeauBut = get_robot_in_line(coordM,oponent_goal_center(),ennemis,seuilBandeau);
					std::vector<int> bandeauAD = get_robot_in_line(coordM,coordAD,ennemis,seuilBandeau);
					std::vector<int> bandeauAG = get_robot_in_line(coordM,coordAG,ennemis,seuilBandeau);
					
					if(!approachM) timerApproach = TIMER_APPROACH;
					if(ballY<= 1 && ballY >= -1) approachM = true;

					//choix de tir ou de passe à l'ailier:
					if(bandeauBut.size() <= bandeauAD.size() && bandeauBut.size() <= bandeauAG.size()){
						//cas de tir:
						striker[2] ->  declare_point_to_strik(oponent_goal_center());
					}else{
						//cas de passe:
						if(bandeauAG.size() < bandeauAD.size())
							striker[2] ->  declare_point_to_strik(coordAG);
						else
							striker[2] ->  declare_point_to_strik(coordAD);
					}
					assign_behavior (player_id(2), striker[2]);

				}
			}

		}


		//=====>   attaquant droit:
		y = coordAD.y;

		//condition d'attaque:
		if(ballX > 1){
			
			//condition de zone: (bandeau 1/3)
			if(( y < -1 ) || approachAD){
				//condition d'approche:
				if(distBetween(coordAD, ball_position()) <= seuilApproche && !approachM && !approachAG){
					std::vector<int> bandeauBut = get_robot_in_line(coordAD,oponent_goal_center(),ennemis,seuilBandeau);
					std::vector<int> bandeauM = get_robot_in_line(coordAD,coordM,ennemis,seuilBandeau);

					if(!approachAD) timerApproach = TIMER_APPROACH;
					approachAD = true;

					//choix de tir ou de passe au milieu:
					if(bandeauBut.size() <= bandeauM.size()){
						//cas de tir:
						striker[0] ->  declare_point_to_strik(oponent_goal_center());
					}else{
						//cas de passe:
						striker[0] ->  declare_point_to_strik(coordM);
					}
					assign_behavior (player_id(0), striker[0]);
				}
			}

		}

		//=====>   attaquant gauche:
		y = coordAG.y;

		//condition d'attaque:
		if( ballX > 1){
			
			//condition de zone: (bandeau 1/3)
			if(( y > 1) || approachAG){
				//condition d'approche:
				if(distBetween(coordAG, ball_position()) <= seuilApproche && !approachM && !approachAD){
					std::vector<int> bandeauBut = get_robot_in_line(coordAG,oponent_goal_center(),ennemis,seuilBandeau);
					std::vector<int> bandeauM = get_robot_in_line(coordAG,coordM,ennemis,seuilBandeau);
					
					if(!approachAG) timerApproach = TIMER_APPROACH;
					approachAG = true;

					//choix de tir ou de passe au milieu:
					if(bandeauBut.size() <= bandeauM.size()){
						//cas de tir:
						striker[1] ->  declare_point_to_strik(oponent_goal_center());
					}else{
						//cas de passe
						striker[1] ->  declare_point_to_strik(coordM);
					}
					assign_behavior (player_id(1), striker[1]);
				}
			}

		}


	//=====>   défenseur droit:		

	y = coordDD.y;
	x = coordDD.x;
	deltaX = abs(x - ballX);
	//condition d'attaque:
	if(ballX > 1){
		//condition de bandeau et d'approche:
		if(ballY < -1 && !approachM && !approachAD && deltaX < 0.5*distAttDefD){

			std::vector<int> bandeauA = get_robot_in_line(coordDD,coordAD,ennemis,seuilBandeau);
			std::vector<int> bandeauM = get_robot_in_line(coordDD,coordM,ennemis,seuilBandeau);
			if(bandeauA.size() <= bandeauM.size()){
				striker[3] ->  declare_point_to_strik(coordAD);
			}else{
				striker[3] ->  declare_point_to_strik(coordM);
			}
			assign_behavior (player_id(3), striker[3]);
		}
	}

	//=====>   défenseur gauche:

	y = coordDG.y;
	x = coordDG.x;
	deltaX = abs(x - ballX);
	//condition d'attaque:
	if(ballX > 1){
		//condition de bandeau et d'approche:
		if(ballY > 1 && !approachM && !approachAG && deltaX < 0.5*distAttDefG){
			
			std::vector<int> bandeauA = get_robot_in_line(coordDG,coordAG,ennemis,seuilBandeau);
			std::vector<int> bandeauM = get_robot_in_line(coordDG,coordM,ennemis,seuilBandeau);
			if(bandeauA.size() <= bandeauM.size()){
				striker[4] ->  declare_point_to_strik(coordAG);
			}else{
				striker[4] ->  declare_point_to_strik(coordM);
			}
			assign_behavior (player_id(4), striker[4]);
			
		}
	}

	//#######   comportement milieu   #####################################################################

	//followballmode est actif
	if(ballX <= 1 && ballX >= -1){
		
		if((distBetween(coordM, ball_position()) <= seuilApproche && timerMiddle == 0) || middle[2]) {
			//striker[2] ->  declare_point_to_strik(oponent_goal_center());
			assign_behavior (player_id(2), degageur[2]);
			if(timerMiddle == 0){
				timerMiddle = TIMER_MIDDLE;
				middle[2] = true;
			} 
		}
		
		if((distBetween(coordAD, ball_position()) <= seuilApproche && timerMiddle == 0) || middle[0]) {
			//striker[0] ->  declare_point_to_strik(oponent_goal_center());
			assign_behavior (player_id(0), degageur[0]);
			if(timerMiddle == 0){
				timerMiddle = TIMER_MIDDLE;
				middle[0] = true;
			} 
		}
		if((distBetween(coordAG, ball_position()) <= seuilApproche && timerMiddle == 0) || middle[1]) {
			//striker[1] ->  declare_point_to_strik(oponent_goal_center());
			assign_behavior (player_id(1), degageur[1]);
			if(timerMiddle == 0){
				timerMiddle = TIMER_MIDDLE;
				middle[1] = true;
			} 
		}
		
		if((distBetween(coordDD, ball_position()) <= seuilApproche && timerMiddle == 0) || middle[3]) {
			//striker[3] ->  declare_point_to_strik(oponent_goal_center());
			assign_behavior (player_id(3), degageur[3]);
			if(timerMiddle == 0){
				timerMiddle = TIMER_MIDDLE;
				middle[3] = true;
			} 
		}
		if((distBetween(coordDG, ball_position()) <= seuilApproche && timerMiddle == 0) || middle[4]) {
			//striker[4] ->  declare_point_to_strik(oponent_goal_center());
			assign_behavior (player_id(4), degageur[4]);
			if(timerMiddle == 0){
				timerMiddle = TIMER_MIDDLE;
				middle[4] = true;
			} 
		}

	}





	
	//#######   comportement défensif   #####################################################################
	
	double seuil = 0.5;
	//défenseur droit :
	if(ballX < -1){
		assign_behavior (player_id(3), mur[3]);
		if(((distBetween(coordDD, ball_position()) <= seuil  && timerMur == 0 )|| degMurD) &&(!degDefAG && !degDefAD && !degDefM)){
			assign_behavior (player_id(3), degageur[3]);
			degMurD = true;
			if(timerMur == 0) timerMur = TIMER_MUR;
		}
	}


	//défenseur gauche :
	if(ballX < -1){
		assign_behavior (player_id(4), mur[4]);
		if(((distBetween(coordDG, ball_position()) <= seuil  && timerMur == 0 )|| degMurG) &&(!degDefAG && !degDefAD && !degDefM)){
			assign_behavior (player_id(4), degageur[4]);
			degMurG = true;
			if(timerMur == 0) timerMur = TIMER_MUR;
		}
	}
	


	
	//milieu:
	if(ballX < - 1){
		if((( distBetween(coordM, ball_position()) <= seuilApproche && timerDef == 0) || degDefM)&&(!degMurG && !degMurD)){
			assign_behavior (player_id(2), degageur[2]);
			degDefM = true;
			if(timerDef == 0) timerDef = TIMER_DEF;
		}
	}

	//attaquant droite:
	if(ballX < - 1){
		if((( ballY < -1 && timerDef == 0) || degDefAD)&&(!degMurG && !degMurD)){
			assign_behavior (player_id(0), degageur[0]);
			degDefAD = true;
			if(timerDef == 0) timerDef = TIMER_DEF;
		}
	}

	//attaquant gauche:
	if(ballX < - 1){
		if((( ballY > 1 && timerDef == 0) || degDefAG)&&(!degMurG && !degMurD)){
			assign_behavior (player_id(1), degageur[1]);
			degDefAG = true;
			if(timerDef == 0) timerDef = TIMER_DEF;
		}
	}
	


	//#####   timers:
	if(timerApproach > 0){
		timerApproach--;
	}
	if(timerApproach == 0){
		approachM = false;
		approachAD = false;
		approachAG = false;
	}

	if(timerMiddle > 0){
		timerMiddle--;
		if(timerMiddle == 0){
			for(int i =0; i < 5; i++){
				middle[i] = false;
			}
		}
	}


	if(timerMur > 0){
		timerMur--;
		if(timerMur == 0){
			degMurD = false;
			degMurG = false;
		}
	}

	if(timerDef > 0){
		timerDef--;
		if(timerDef == 0){
			degDefM = false;
			degDefAD = false;
			degDefAG = false;
		}
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
