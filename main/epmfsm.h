#ifndef empfsm_h
#define empfsm_h

#include <stdlib.h>

/* State and Relation structures */
typedef struct epmfsm_state epmfsm_state;
typedef struct epmfsm_relation epmfsm_relation;

struct epmfsm_state {
    void *essence;
    char *purpose;
    void (*manifest)(void *essence, epmfsm_state *states, epmfsm_relation *relations);
    bool is_active;
};

struct epmfsm_relation {
    epmfsm_state *state1;
    epmfsm_state *state2;
    void *essence;  // This could be used for more complex relations.
    char *purpose;
    void (*manifest)(epmfsm_relation* relation, epmfsm_state *state1, epmfsm_state *state2, epmfsm_state *states, epmfsm_relation *relations);
    bool is_active;
};

/* Scheduler */
static void epmfsm_scheduler(epmfsm_state *states, int num_states, epmfsm_relation *relations, int num_relations) {
    for (int i = 0; i < num_states; i++) {
        if (states[i].is_active && states[i].manifest != NULL) {
            states[i].manifest(states[i].essence, states, relations);
        }
    }

    for (int i = 0; i < num_relations; i++) {
        if (relations[i].is_active && relations[i].manifest != NULL) {
            relations[i].manifest(&relations[i], relations[i].state1, relations[i].state2, states, relations);
        }
    }
}

#endif