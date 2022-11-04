#ifndef _SHOOTER_MESSAGE_HPP_
#define _SHOOTER_MESSAGE_HPP_

#include <Message.hpp>
#include "MessageStructure.hpp"

typedef struct ShooterMessageType
{
    //  shooter variable
    bool all_reload;
    shooter_t shooter;
} shooter_message_t;

//  create message
typedef sb::Message<shooter_message_t> ShooterMessage;

#endif