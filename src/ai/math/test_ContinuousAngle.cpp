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

#include "ContinuousAngle.h"
#include <math.h>
#include <debug.h>
#include <sstream>

using namespace rhoban_utils;

TEST(test_ContinuousAngle, constructors){
    {
        ContinuousAngle c;
        EXPECT_TRUE( c.value() == 0.0 );
    }

    {
        ContinuousAngle c(2.1);
        EXPECT_TRUE( c.value() == 2.1 );
    }

    {
        ContinuousAngle c(2.1);
        ContinuousAngle d(c);
        EXPECT_TRUE( c.value() == 2.1 );
        EXPECT_TRUE( d.value() == 2.1 );
    }


    {
        ContinuousAngle c(4.5);
        EXPECT_TRUE( c.value() == 4.5 );
    }

    {
        ContinuousAngle c(-4.5);
        EXPECT_TRUE( c.value() == -4.5 );
    }

    {
        ContinuousAngle c(-4.5);
        EXPECT_TRUE( c.value() == -4.5 );
    }
/*
    {
        ContinuousAngle c( Angle(321) );
        EXPECT_TRUE( c.angle() == Angle(321) );
    }
*/
    {
        ContinuousAngle c = -4.5;
        EXPECT_TRUE( c.value() == -4.5 );
    }

    {
        ContinuousAngle c = 4.5;
        EXPECT_TRUE( c.value() == 4.5 );
    }
/*
    {
        ContinuousAngle c = Angle(321);
        EXPECT_TRUE( c.angle() == Angle(321) );
    }
*/
}

TEST(test_ContinuousAngle, angle){
    {
        ContinuousAngle c(2*M_PI);
        EXPECT_TRUE( std::fabs( c.angle().getSignedValue() - 0 ) < 0.00001 );
    }
    {
        ContinuousAngle c(4*M_PI);
        EXPECT_TRUE( std::fabs( c.angle().getSignedValue() - 0 ) < 0.00001 );
    }
    {
        ContinuousAngle c(-4*M_PI);
        EXPECT_TRUE( std::fabs( c.angle().getSignedValue() - 0 ) < 0.00001 );
    }
}

TEST(test_ContinuousAngle, abs){
    {
        ContinuousAngle c(1.4);
        ContinuousAngle d = c.abs();
        EXPECT_TRUE( std::fabs( d.value() - 1.4 ) < 0.0001 );
    }
    {
        ContinuousAngle c(-1.4);
        ContinuousAngle d = c.abs();
        EXPECT_TRUE( std::fabs( d.value() - 1.4 ) < 0.0001 );
    }
}

TEST(test_ContinuousAngle, operator_plus){
    {
        ContinuousAngle c(1.4);
        c+=1;
        EXPECT_TRUE( std::fabs( c.value() - 2.4 ) < 0.0001 );
    }
    {
        ContinuousAngle c(1.4);
        ContinuousAngle d = c + 1;
        EXPECT_TRUE( std::fabs( c.value() - 1.4 ) < 0.0001 );
        EXPECT_TRUE( std::fabs( d.value() - 2.4 ) < 0.0001 );
    }
    {
        ContinuousAngle c(1.4);
        ContinuousAngle d = +c;
        EXPECT_TRUE( std::fabs( c.value() - 1.4 ) < 0.0001 );
        EXPECT_TRUE( std::fabs( d.value() - 1.4 ) < 0.0001 );
    }
}

TEST(test_ContinuousAngle, operator_minus){
    {
        ContinuousAngle c(1.4);
        c-=1;
        EXPECT_TRUE( std::fabs( c.value() - 0.4 ) < 0.0001 );
    }
    {
        ContinuousAngle c(1.4);
        ContinuousAngle d = c - 1;
        EXPECT_TRUE( std::fabs( c.value() - 1.4 ) < 0.0001 );
        EXPECT_TRUE( std::fabs( d.value() - 0.4 ) < 0.0001 );
    }
    {
        ContinuousAngle c(1.4);
        ContinuousAngle d = -c;
        EXPECT_TRUE( std::fabs( c.value() - 1.4 ) < 0.0001 );
        EXPECT_TRUE( std::fabs( d.value() + 1.4 ) < 0.0001 );
    }
}

TEST(test_ContinuousAngle, operator_prod){
    {
        ContinuousAngle c(1.4);
        c*=2;
        EXPECT_TRUE( std::fabs( c.value() - 2.8 ) < 0.0001 );
    }
    {
        ContinuousAngle c(1.4);
        ContinuousAngle d = c * 2;
        EXPECT_TRUE( std::fabs( c.value() - 1.4 ) < 0.0001 );
        EXPECT_TRUE( std::fabs( d.value() - 2.8 ) < 0.0001 );
    }
}

TEST(test_ContinuousAngle, operator_div){
    {
        ContinuousAngle c(2.8);
        c/=2;
        EXPECT_TRUE( std::fabs( c.value() - 1.4 ) < 0.0001 );
    }
    {
        ContinuousAngle c(2.8);
        ContinuousAngle d = c / 2;
        EXPECT_TRUE( std::fabs( c.value() - 2.8 ) < 0.0001 );
        EXPECT_TRUE( std::fabs( d.value() - 1.4 ) < 0.0001 );
    }
}

TEST(test_ContinuousAngle, operator_equal){
    {
        ContinuousAngle c(2.8);
        ContinuousAngle d(2.8);
        ContinuousAngle e(4.8);

        EXPECT_TRUE( c == c );
        EXPECT_TRUE( not(c != c) );
        EXPECT_TRUE( not(c < c) );
        EXPECT_TRUE( c <= c );
        EXPECT_TRUE( not(c > c) );
        EXPECT_TRUE( c >= c );
    }
}


TEST(test_ContinuousAngle, turn){
    {
        ContinuousAngle c(2.8);
        EXPECT_TRUE( std::fabs( c.turn() - 0.445633840657 ) < 0.0001 );
    }
}

TEST(test_ContinuousAngle, nb_turn){
    {
        ContinuousAngle c(2.8);
        EXPECT_TRUE( c.nb_turn() == 0 );
        c += (2*M_PI);
        EXPECT_TRUE( c.nb_turn() == 1 );
        c += (2*M_PI);
        EXPECT_TRUE( c.nb_turn() == 2 );
    }
    {
        ContinuousAngle c(2.8);
        EXPECT_TRUE( c.nb_turn() == 0 );
        c -= (2*M_PI);
        EXPECT_TRUE( c.nb_turn() == 0 );
        c -= (2*M_PI);
        EXPECT_TRUE( c.nb_turn() == -1 );
        c -= (2*M_PI);
        EXPECT_TRUE( c.nb_turn() == -2 );
    }
}

TEST(test_ContinuousAngle, stream){
    {
        ContinuousAngle c(2.8);

        std::ostringstream s1;
        
        s1 << c;
        EXPECT_TRUE( "2.8" == s1.str() ); 
        s1.str("");
        s1.clear();
        
        c += (2*M_PI);
        s1 << c;
        EXPECT_TRUE( "1*2pi+2.8" == s1.str() ); 
        s1.str("");
        s1.clear();
        
        c += (2*M_PI);
        s1 << c;
        EXPECT_TRUE( "2*2pi+2.8" == s1.str() ); 
        s1.str("");
        s1.clear();
    } 
    {
        ContinuousAngle c(2.8);

        std::ostringstream s1;
        
        s1 << c;
        EXPECT_TRUE( "2.8" == s1.str() ); 
        s1.str("");
        s1.clear();
        
        c -= (2*M_PI);
        s1 << c;
        EXPECT_TRUE( "-3.48319" == s1.str() ); 
        s1.str("");
        s1.clear();
        
        c -= (2*M_PI);
        s1 << c;

        EXPECT_TRUE( "-1*2pi-3.48319" == s1.str() ); 
        s1.str("");
        s1.clear();

        c -= (2*M_PI);
        s1 << c;
        EXPECT_TRUE( "-2*2pi-3.48319" == s1.str() ); 
        s1.str("");
        s1.clear();
    } 
}

TEST(test_ContinuousAngle, set_to_nearest){
    {
        double amplitude = 4*2*M_PI;
        ContinuousAngle c(-amplitude);
        ContinuousAngle d(c);
        double step = .1;
        for( int i=0; i<2*amplitude/step; i++ ){
            d = c;

            double angle = std::fmod(
                c.value() + step, 2*M_PI
            );
            c.set_to_nearest( angle );
            
            EXPECT_TRUE(
                std::fabs( c.value() - d.value() ) < 2*step
            );
        } 
    }
    {
        double amplitude = 4*2*M_PI;
        ContinuousAngle c(amplitude);
        ContinuousAngle d(c);
        double step = .1;
        for( int i=0; i<2*amplitude/step; i++ ){
            d = c;
            double angle = std::fmod(
                c.value() - step, 2*M_PI
            );
            c.set_to_nearest( angle );
            EXPECT_TRUE(
                std::fabs( c.value() - d.value() ) < 2*step
            );
            
        }
    } 
    {
        double amplitude = 4*2*M_PI;
        ContinuousAngle c(-amplitude);
        ContinuousAngle d(c);
        double step = .1;
        for( int i=0; i<2*amplitude/step; i++ ){
            d = c;
            
            Angle angle( rad2deg( c.value() + step ) );
            c.set_to_nearest( angle );
           
            EXPECT_TRUE(
                std::fabs( 
                    Angle( rad2deg(c.value()) ).getSignedValue()
                    -
                    angle.getSignedValue()
                ) < 0.000001
            );
            
            EXPECT_TRUE(
                std::fabs( c.value() - d.value() ) < 2*step
            );
        }
    }
    {
        double amplitude = 4*2*M_PI;
        ContinuousAngle c(amplitude);
        ContinuousAngle d(c);
        double step = .1;
        for( int i=0; i<2*amplitude/step; i++ ){
            d = c;
            Angle angle ( rad2deg( c.value() - step ) );
            c.set_to_nearest( angle );

            EXPECT_TRUE(
                std::fabs( 
                    Angle( rad2deg(c.value()) ).getSignedValue()
                    -
                    angle.getSignedValue()
                ) < 0.000001
            );

            EXPECT_TRUE(
                std::fabs( c.value() - d.value() ) < 2*step
            );
            
        }
    } 
/*
    {
        double amplitude = 4*2*M_PI;
        ContinuousAngle c(-amplitude);
        ContinuousAngle d(c);
        double step = .1;
        for( int i=0; i<2*amplitude/step; i++ ){
            d = c;

            double angle = std::fmod(
                c.value() + step, 2*M_PI
            );
            c = Angle( rad2deg(angle) );
            
            EXPECT_TRUE(
                std::fabs( c.value() - d.value() ) < 2*step
            );
        } 
    }
    {
        double amplitude = 4*2*M_PI;
        ContinuousAngle c(amplitude);
        ContinuousAngle d(c);
        double step = .1;
        for( int i=0; i<2*amplitude/step; i++ ){
            d = c;
            double angle = std::fmod(
                c.value() - step, 2*M_PI
            );
            c = Angle( rad2deg(angle) );
            EXPECT_TRUE(
                std::fabs( c.value() - d.value() ) < 2*step
            );
            
        }
    } 
*/
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

