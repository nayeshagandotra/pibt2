/*
 * Implementation of Priority Inheritance with Backtracking (PIBT)
 *
 * - ref
 * Okumura, K., Machida, M., Défago, X., & Tamura, Y. (2019).
 * Priority Inheritance with Backtracking for Iterative Multi-agent Path
 * Finding. In Proceedings of the Twenty-Eighth International Joint Conference
 * on Artificial Intelligence (pp. 535–542).
 */

#pragma once
#include "solver.hpp"
#include <iostream>

class PIBT : public MAPF_Solver
{
public:
  static const std::string SOLVER_NAME;

private:

  struct Agent; 
  using Agents = std::vector<Agent*>; 

  // PIBT agent
  struct Agent {
    int id;
    Node* v_now;        // current location
    Node* v_next;       // next location
    Node* v_next_best;  // best next location
    Node* g;            // goal
    int elapsed;        // eta
    int init_d;         // initial distance
    float tie_breaker;  // epsilon, tie-breaker
    bool is_conflicting; //should we recurse through all the actions?
    Agents* group;        // group this belongs to
  };

  // <node-id, agent>, whether the node is occupied or not
  // work as reservation table
  Agents occupied_now;
  Agents occupied_next;

  using Groups = std::vector<Agents>;

  // new additions
  int timestep_penalty;
  static bool compareAgents(const Agent* a, const Agent* b) {
        if (a->elapsed != b->elapsed) return a->elapsed > b->elapsed;
        if (a->init_d != b->init_d) return a->init_d > b->init_d;
        return a->tie_breaker > b->tie_breaker;
  }

  // option
  bool disable_dist_init = false;

  // result of priority inheritance: true -> valid, false -> invalid
  bool funcPIBT(Agent* ai, Agent* aj = nullptr);

  // plan one step
  void plan_one_step(Agents agents);

  // main optimal function
  std::pair<bool, int> OptiPIBT(Agents A, Agent* aj, int accumulated_penalty, int best_penalty);
  void print_penalty(const std::string& filename, int penalty);
  void check_potential_conflicts(Agents A);

  // clear lists and update so we can run again
  void refresh_lists(Agents A);

  // main
  void run();

public:
  PIBT(MAPF_Instance* _P);
  ~PIBT() {}

  void setParams(int argc, char* argv[]);
  static void printHelp();
};
