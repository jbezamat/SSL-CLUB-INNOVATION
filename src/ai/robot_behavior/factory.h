#ifndef __ROBOT_BEHAVIOR__FACTORY__H__
#define __ROBOT_BEHAVIOR__FACTORY__H__

#include <AiData.h>
#include "robot_behavior.h"
#include "consign_follower.h"

namespace RhobanSSL {
namespace Robot_behavior {

class Factory {
    public:
    static ConsignFollower* fixed_consign_follower( Ai::AiData & ai_data );

};

};
} ;

#endif