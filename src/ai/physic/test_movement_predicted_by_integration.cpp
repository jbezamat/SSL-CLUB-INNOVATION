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

#include <gtest/gtest.h>

#include "movement_predicted_by_integration.h"
#include <cmath>

using namespace rhoban_geometry;

double norm( const Point & p ){
    return std::sqrt( p.getX()*p.getX() + p.getY()*p.getY() );
}
    
TEST(test_movement_predicted_by_integration, use_cases){
    {
        RhobanSSL::MovementSample mov(3);
        mov.insert( 
            RhobanSSL::PositionSample( 1, Point(1,2), ContinuousAngle(4) ) 
        );
        mov.insert( 
            RhobanSSL::PositionSample( 3, Point(8,16), ContinuousAngle(32) ) 
        );
        mov.insert( 
            RhobanSSL::PositionSample( 6, Point(27,54), ContinuousAngle(108) ) 
        );

        RhobanSSL::Movement_predicted_by_integration pred;
        pred.set_sample( mov );

        Point pos(27,54);
        EXPECT_TRUE( pred.linear_position(6) == pos );

        ContinuousAngle ang( 108 );
        EXPECT_TRUE( pred.angular_position(6) == ang );

        Point vel = Point(27-8,54-16)/3.0;
        EXPECT_TRUE( pred.linear_velocity(6) == vel );
       
        double ang_vel = (108.0-32)/(6-3.0);
        EXPECT_TRUE( pred.angular_velocity(6) == ang_vel );

 
        Point vel0 = Point(27-8,54-16)/3.0;
        Point vel1 = Point(8-1,16-2)/2.0;

        Point accel = (vel0 - vel1)/3.0;

        EXPECT_TRUE( pred.linear_acceleration(6) == accel );

        double ang_vel1 = (32.0-4)/(3-1.0);
        double ang_vel0 = (108.0-32)/(6-3.0);
        double ang_accel = (ang_vel0 - ang_vel1)/(6-3.0);

        EXPECT_TRUE( pred.angular_acceleration(6) == ang_accel );

        double step = 0.1;
        double t = 0;
        for( int i=0; i<100; i++ ){
            EXPECT_TRUE( 
                norm(
                    pred.linear_position(6+t) - (
                        pos + vel*t + accel*t*t/2.0 
                    )
                ) < 0.00001
            );
            EXPECT_TRUE(
                std::fabs(
                    (
                        pred.angular_position(6+t) - (
                            ang + ang_vel*t + ang_accel*t*t/2.0 
                        )
                    ).value()
                ) < 0.00001
            );

            EXPECT_TRUE( pred.linear_acceleration(6+t) == accel );
            EXPECT_TRUE( pred.angular_acceleration(6+t) == ang_accel );
            t+= step;
        }

    }
}

TEST(test_movement_predicted_by_integration, clone){
    {
        RhobanSSL::MovementSample mov(3);
        mov.insert( 
            RhobanSSL::PositionSample( 1, Point(1,2), ContinuousAngle(4) ) 
        );
        mov.insert( 
            RhobanSSL::PositionSample( 3, Point(8,16), ContinuousAngle(32) ) 
        );
        mov.insert( 
            RhobanSSL::PositionSample( 6, Point(27,54), ContinuousAngle(108) ) 
        );

        RhobanSSL::Movement_predicted_by_integration pred_tmp;
        RhobanSSL::Movement * pred_ptr = pred_tmp.clone();
        RhobanSSL::Movement & pred = *pred_ptr;
        
        pred.set_sample( mov );

        delete(pred_ptr);
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
