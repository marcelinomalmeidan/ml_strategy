#ifndef STRUCTS_H_
#define STRUCTS_H_

#include "ml_strategy/team_strategy_class.h"
#include <thread>

struct globalVariables {
    // Mutex protected variables
    TeamStrategy obj_team_strategy;
};

class mutexStruct {
 public:
    pthread_mutex_t m_team_strategy;

    // Methods
    mutexStruct() {
        pthread_mutex_init(&m_team_strategy, NULL);
    }
    void destroy() {
        pthread_mutex_destroy(&m_team_strategy);
    }
};

#endif  // STRUCTS_H_