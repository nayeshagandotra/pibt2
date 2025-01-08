#include "../include/pibt.hpp"
#include <fstream>

const std::string PIBT::SOLVER_NAME = "PIBT";

PIBT::PIBT(MAPF_Instance* _P)
    : MAPF_Solver(_P),
      occupied_now(Agents(G->getNodesSize(), nullptr)),
      occupied_next(Agents(G->getNodesSize(), nullptr))
{
  solver_name = PIBT::SOLVER_NAME;
}

void PIBT::print_penalty(const std::string& filename, int penalty) {
    // std::string output_dir = "code/output/";
    std::string full_filename = filename;
    
    std::ofstream outFile(full_filename, std::ios::app);  // Open in append mode
    if (!outFile) {
        std::cerr << "Error opening file: " << full_filename << "\n";
        return;
    }

    // Write start positions
    outFile << penalty << "\n";
    outFile.close();
}

void PIBT::check_potential_conflicts(Agents A){
  for (auto a : A){
    if (!a->is_conflicting){
      // get candidates
      Nodes C = a->v_now->neighbor;
      C.push_back(a->v_now);
      for (auto u: C){
        if (occupied_now[u->id] != nullptr && occupied_now[u->id] != a){
          occupied_now[u->id]->is_conflicting = true;
          a->is_conflicting = true;
        }
      }
    } 
  }
}

void PIBT::plan_one_step(Agents A){
  for (auto a : A) {
    // if the agent has next location, then skip
    if (a->v_next == nullptr) {
      // determine its next 
      funcPIBT(a);
    }
  }
}

  std::pair<bool, int> PIBT::OptiPIBT(Agents A, Agent* aj, int accumulated_penalty, int best_penalty){

  // empty A return condition
  if (A.empty()) {
    return std::make_pair(false, accumulated_penalty);
  }

  // set penalty to 0 initially
  timestep_penalty = 0;
  // planning
  std::sort(A.begin(), A.end(), compareAgents);
  plan_one_step(A);

  if (timestep_penalty == 0){
    //ideal moves. return. 
    for (auto a: A){
      a->v_next_best = a->v_next;
    }
    // refresh_lists(A);
    return std::make_pair(false, accumulated_penalty);
  }

  Agent* ai;
  // pick highest priority agent 
  if (aj == nullptr){
    ai = A[0];
  }
  else{
    ai = aj;
  }

  // get subset of agents
  Agents agents_subset;
  // Copy all pointers except for agent ai
  std::copy_if(A.begin(), A.end(), std::back_inserter(agents_subset), 
                [ai](Agent* agent) { return agent != ai; });
  

  // so far, current best is this
  if (timestep_penalty + accumulated_penalty < best_penalty){
    // for (auto a: A){
    ai->v_next_best = ai->v_next;
    // }
  }

  // compare two nodes
  auto compare = [&](Node* const v, Node* const u) {
    int d_v = pathDist(ai->id, v);
    int d_u = pathDist(ai->id, u);
    if (d_v != d_u) return d_v < d_u;
    // tie break
    if (occupied_now[v->id] != nullptr && occupied_now[u->id] == nullptr)
      return false;
    if (occupied_now[v->id] == nullptr && occupied_now[u->id] != nullptr)
      return true;
    return false;
  };

  // get candidates
  Nodes C = ai->v_now->neighbor;
  C.push_back(ai->v_now);
  // randomize
  // std::shuffle(C.begin(), C.end(), *MT);
  // sort
  std::sort(C.begin(), C.end(), compare);

  // calculate ideal dist for penalty purposes
  int ideal_dist = pathDist(ai->id, C[0]);  // Distance to goal if taking ideal move
  int actual_dist;

  // counting number of skipped actions
  int n_avail_acts, n_skipped_acts;

  for (auto u: C){   //include best action for completeness
    refresh_lists(A); // clear the results from the last PIBT call
    n_avail_acts += 1;
    actual_dist = pathDist(ai->id, u);
    int action_penalty =  (actual_dist - ideal_dist); //0 if equal
    if (action_penalty + accumulated_penalty >= best_penalty){
      n_skipped_acts += 1;
      continue; //bad action
    }
    if (occupied_next[u->id] != nullptr && occupied_next[u->id] != ai){
      // The agent at vertex 'id' in occupied_next is not nullptr and not in A
      // and it's not the current agent
      // means it's been reserved by a previous fixed agent
      occupied_next[u->id]->is_conflicting = true;
      ai->is_conflicting = true;
      n_skipped_acts += 1;
      continue;
    }

    // option 2: try recursing through ak and skip action if it doesn't improve
    occupied_next[u->id] = ai;
    ai->v_next = u; //TODO:remove if not required by PIBT
    // if action is causing swap conflict, continue
    auto ak = occupied_now[u->id];
    if (ak != nullptr && ak != ai){
        // The agent at vertex 'id' in occupied_now is not nullptr and not in A
        // and it's not the current agent
        // means it's been reserved by a previous fixed agent
        // // check if we have a swap conflict
        if (ak->v_next == ai->v_now){
          ak->is_conflicting = true;
          ai->is_conflicting = true;
          // the node we are trying to move to is currently occupied by an agent that 
          // wants to move to us or stay at u. since we are lower priority, we sacrifice this action.
          n_skipped_acts += 1;
          continue;
        }
        // there is an unplanned agent here, let's see if we can comfortably move it
        if (ak->v_next == nullptr){
          auto [failed, bas] = OptiPIBT(agents_subset, ak, accumulated_penalty + action_penalty, best_penalty);
          if (failed){
            ak->is_conflicting = true;
            ai->is_conflicting = true;
            n_skipped_acts += 1;
            // we tried moving to this action and moving other agents accordingly, but agent ak is stuck
            continue;
          }
          if (bas < best_penalty){
            best_penalty = bas;
            ai->v_next_best = u;
            continue;
          }
        }
    }
   
    
    // run OptiPIBT recursive call
    auto [f, best_after_subset] = OptiPIBT(agents_subset, nullptr, accumulated_penalty + action_penalty, best_penalty);
    
    // this should give it some new v_next values
    if (best_after_subset < best_penalty){
      best_penalty = best_after_subset;
      ai->v_next_best = u;
    }
    if (!ai->is_conflicting){
      // refresh_lists(A); // clear the results from the last PIBT call
      break;
    }
  }
  // either all moves have failed or next best has been found
  if (n_skipped_acts == n_avail_acts){
    return std::make_pair(true, best_penalty);
  }
  return std::make_pair(false, best_penalty);
}

void PIBT::refresh_lists(Agents A){
  timestep_penalty = 0;
  // clear occupied next (local list) before next optipibt iter
  for (auto a : A) {
    // clear
    if (a->v_next != nullptr){
      occupied_next[a->v_next->id] = nullptr;
      a->v_next = nullptr;
    }
  }
}

void PIBT::run()
{
  unsigned int seed = 12345; // Replace with your desired seed
  MT->seed(seed);

  Agents A;
  // initialize
  for (int i = 0; i < P->getNum(); ++i) {
    Node* s = P->getStart(i);
    Node* g = P->getGoal(i);
    int d = disable_dist_init ? 0 : pathDist(i);
    Agent* a = new Agent{i,                          // id
                         s,                          // current location
                         nullptr,                    // next location (local)
                         nullptr,                    // next best location
                         g,                          // goal
                         0,                          // elapsed
                         d,                          // dist from s -> g
                         getRandomFloat(0, 1, MT)};  // tie-breaker
                         false;                      // conflict remembrance
    A.push_back(a);
    occupied_now[s->id] = a;
  }
  solution.add(P->getConfigStart());

  // main loop
  int timestep = 0;
  while (true) {
    info(" ", "elapsed:", getSolverElapsedTime(), ", timestep:", timestep);
    
    // plan one step using optipibt
    auto [f, os_penalty] = OptiPIBT(A, nullptr, 0, 100000000);
    print_penalty("costs.txt", os_penalty);
    refresh_lists(A);
    for (auto a : A) {
      a->is_conflicting = false;
    }

    // acting
    bool check_goal_cond = true;
    Config config(P->getNum(), nullptr);
    for (auto a : A) {
      // clear
      if (occupied_now[a->v_now->id] == a) occupied_now[a->v_now->id] = nullptr;
      // occupied_next[a->v_next_best->id] = nullptr;  
      // set next location
      config[a->id] = a->v_next_best;
      occupied_now[a->v_next_best->id] = a;
      // check goal condition
      check_goal_cond &= (a->v_next_best == a->g);
      // update priority
      a->elapsed = (a->v_next_best == a->g) ? 0 : a->elapsed + 1;
      // reset params
      a->v_now = a->v_next_best;
      a->v_next_best = nullptr;
    }
    // update plan
    solution.add(config);
  
    // success
    if (check_goal_cond) {
      solved = true;
    }

    if (solved){
      break;
    }

    ++timestep;

    // // failed
    // if (timestep >= max_timestep || overCompTime()) {
    //   break;
    // }
  }

  // memory clear
  for (auto a : A) delete a;
}

bool PIBT::funcPIBT(Agent* ai, Agent* aj)
{
  // compare two nodes
  auto compare = [&](Node* const v, Node* const u) {
    int d_v = pathDist(ai->id, v);
    int d_u = pathDist(ai->id, u);
    if (d_v != d_u) return d_v < d_u;
    // tie break
    if (occupied_now[v->id] != nullptr && occupied_now[u->id] == nullptr)
      return false;
    if (occupied_now[v->id] == nullptr && occupied_now[u->id] != nullptr)
      return true;
    return false;
  };

  int tsp = timestep_penalty;

  // get candidates
  Nodes C = ai->v_now->neighbor;
  C.push_back(ai->v_now);
  // randomize
  // std::shuffle(C.begin(), C.end(), *MT);
  // sort
  std::sort(C.begin(), C.end(), compare);

  // calculate ideal dist for penalty purposes
  int ideal_dist = pathDist(ai->id, C[0]);  // Distance to goal if taking ideal move
  // std::cout << ideal_dist << std::endl;
  int actual_dist;
  int diff;

  for (auto u : C) {
    // avoid conflicts
    if (occupied_next[u->id] != nullptr){
      ai->is_conflicting = true;
      occupied_next[u->id]->is_conflicting = true;
      continue;
    } 
    if (aj != nullptr && u == aj->v_now){
      aj->is_conflicting = true;
      ai->is_conflicting = true;
      continue;  //prevents swap conflict
    } 

    auto ak = occupied_now[u->id];
    // explicitly prevent swap conflict in globally planned agents
    // means u = v_now for pre-planned agent, so we can't swap
    // aka this node is not valid, don't even think about it.
    if (ak != nullptr && ak->v_next == ai->v_now){
      // occupied_next[u->id] = nullptr;
      ak->is_conflicting = true;
      ai->is_conflicting = true;
      continue;
    } 

    // reserve
    occupied_next[u->id] = ai;
    ai->v_next = u;

    if (ak != nullptr && ak->v_next == nullptr) {
      if (!funcPIBT(ak, ai)){
        ak->is_conflicting = true;
        ai->is_conflicting = true;
        // means occupied_next will be equal to ak 
        // since all other planning for ak failed
        // occupied_next[u->id] = nullptr;
        ai->v_next = nullptr;
        continue;  // replanning failed
      }
      if (tsp != timestep_penalty){
        ak->is_conflicting = true;
        ai->is_conflicting = true;
      } 
    }

    // find penalty
    actual_dist = pathDist(ai->id, ai->v_next);
    diff = actual_dist - ideal_dist;
    timestep_penalty += diff; //0 if equal
    // success to plan next one step
    return true;
  }

  // failed to secure node
  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;

  // find penalty
  actual_dist = pathDist(ai->id, ai->v_next);
  diff = actual_dist - ideal_dist;
  timestep_penalty += diff; //0 if equal
  return false;
}

void PIBT::setParams(int argc, char* argv[])
{
  struct option longopts[] = {
      {"disable-dist-init", no_argument, 0, 'd'},
      {0, 0, 0, 0},
  };
  optind = 1;  // reset
  int opt, longindex;
  while ((opt = getopt_long(argc, argv, "d", longopts, &longindex)) != -1) {
    switch (opt) {
      case 'd':
        disable_dist_init = true;
        break;
      default:
        break;
    }
  }
}

void PIBT::printHelp()
{
  std::cout << PIBT::SOLVER_NAME << "\n"
            << "  -d --disable-dist-init"
            << "        "
            << "disable initialization of priorities "
            << "using distance from starts to goals" << std::endl;
}
