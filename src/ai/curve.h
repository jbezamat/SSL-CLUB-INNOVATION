#ifndef __curve__H__
#define __curve__H__

#include "debug.h"
#include <Eigen/Dense>
#include <vector>

struct VelocityConsign {
    double distance;
    double max_velocity;
    double max_acceleration; 

    VelocityConsign(
        double distance, 
        double max_velocity,
        double max_acceleration 
    );

    double operator()(double t);
    double time_of_deplacement();
    double time_of_acceleration();
};

class Curve2d {
    public:
        std::function<Eigen::Vector2d (double u)> curve;
        
        double step_time;
        double curve_length;
        double time_max;

        void init();

    public:
        Curve2d(
            const std::function<Eigen::Vector2d (double u)> & curve,
            double step_time
        );
        Curve2d( const Curve2d& curve );

        Eigen::Vector2d operator()(double u) const;

        double arc_length( double u ) const;
        double inverse_of_arc_length( double l ) const;
        double size() const;
        
};

class RenormalizedCurve : public Curve2d {
    public:
        std::function<double (double t)> velocity_consign;
        double time_max;

        void init();

    public:
        RenormalizedCurve(
            const std::function<Eigen::Vector2d (double u)> & curve,
            const std::function<double (double t)> & velocity_consign,
            double step_time
        );
        RenormalizedCurve(
            const Curve2d & curve,
            const std::function<double (double t)> & velocity_consign
        );

        void set_step_time( double dt );

        double max_time() const;
        Eigen::Vector2d original_curve( double u ) const;
        double time( double length ) const;
        double position_consign( double t ) const;
        Eigen::Vector2d operator()(double t) const;

        double error_position_consign() const;
        double get_step_time() const;
};

#endif