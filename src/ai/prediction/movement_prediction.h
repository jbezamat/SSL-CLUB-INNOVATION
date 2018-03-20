#ifndef __MOVEMENT_PREDICTION_H__ 
#define __MOVEMENT_PREDICTION_H__ 

#include <tools/Movement.h>

namespace RhobanSSL {

class Movement_predicted_by_integration : public Movement {
    private:
        MovementSample samples;

        void check();

    public:
        virtual Movement * clone() const;

        virtual void set_sample( const MovementSample & samples );

        virtual Point linear_position( double time ) const;
        virtual Angle angular_position( double time ) const;

        virtual Vector2d linear_velocity( double time ) const;
        virtual Angle angular_velocity( double time ) const;

        virtual Vector2d linear_acceleration( double time ) const;
        virtual Angle angular_acceleration( double time ) const;

        virtual void print(std::ostream& stream) const;
};


class Movement_with_no_prediction : public Movement {
    private:
        MovementSample samples;

    public:
        virtual Movement * clone() const;

        virtual void set_sample( const MovementSample & samples );

        virtual Point linear_position( double time ) const;
        virtual Angle angular_position( double time ) const;

        virtual Vector2d linear_velocity( double time ) const;
        virtual Angle angular_velocity( double time ) const;

        virtual Vector2d linear_acceleration( double time ) const;
        virtual Angle angular_acceleration( double time ) const;

        virtual void print(std::ostream& stream) const;
};


}

#endif