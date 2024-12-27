#include "../include/pibt.hpp"

const std::string PIBT::SOLVER_NAME = "PIBT";

PIBT::PIBT(MAPF_Instance* _P)
    : MAPF_Solver(_P),
      occupied_now(Agents(G->getNodesSize(), nullptr)),
      occupied_next(Agents(G->getNodesSize(), nullptr))
{
  solver_name = PIBT::SOLVER_NAME;
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

int PIBT::OptiPIBT(Agents A, int accumulated_penalty, int best_penalty){

  // empty A return condition
  if (A.empty()) {
    return accumulated_penalty;
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
    return timestep_penalty + accumulated_penalty;
  }

  // so far, current best is this
  if (timestep_penalty + accumulated_penalty < best_penalty){
    for (auto a: A){
      a->v_next_best = a->v_next;
    }
    best_penalty = timestep_penalty + accumulated_penalty;
  }

  // pick highest priority agent 
  auto ai = A[0];

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
  std::shuffle(C.begin(), C.end(), *MT);
  // sort
  std::sort(C.begin(), C.end(), compare);

  // calculate ideal dist for penalty purposes
  int ideal_dist = pathDist(ai->id, C[0]);  // Distance to goal if taking ideal move
  int actual_dist;

  for (auto u: C){   //include best action for completeness
    refresh_lists(A); // clear the results from the last PIBT call
    actual_dist = pathDist(ai->id, u);
    int action_penalty =  (actual_dist - ideal_dist); //0 if equal
    if (action_penalty + accumulated_penalty >= best_penalty){
      continue; //bad action
    }
    if (occupied_next[u->id] != nullptr && occupied_next[u->id] != ai){
      if (std::find(A.begin(), A.end(), occupied_next[u->id]) == A.end()) {
        // The agent at vertex 'id' in occupied_next is not nullptr and not in A
        // and it's not the current agent
        // means it's been reserved by a previous fixed agent
        continue;
      }
    }
    // get subset of agents
    Agents agents_subset;
    // Copy all pointers except for agent ai
    std::copy_if(A.begin(), A.end(), std::back_inserter(agents_subset), 
                  [ai](Agent* agent) { return agent != ai; });

    // if action is causing swap conflict, continue
    auto ak = occupied_now[u->id];
    if (ak != nullptr && ak != ai){
      if (std::find(A.begin(), A.end(), ak) == A.end()) {
        // The agent at vertex 'id' in occupied_now is not nullptr and not in A
        // and it's not the current agent
        // means it's been reserved by a previous fixed agent
        // check if we have a swap conflict
        if (ak->v_next == ai->v_now){
          // the node we are trying to move to is currently occupied by an agent that 
          // wants to move to us. since we are lower priority, we sacrifice this action.
          continue;
        }
      }
      // refresh_lists(A); // clear the results from the last PIBT call
      // fix the action by updating occupied next
      // reserve
      occupied_next[u->id] = ai;
      ai->v_next = u; //TODO:remove if not required by PIBT
      // option 2: we have a non-planned agent here, we try to call funcPIBT to move the agent
      // same priority swapping logic as before
      if (!funcPIBT(ak, ai)){
        // means ak has nowhere else to go. this agent should try going somewhere else
        // also, since ak is stuck, we don't need to recurse for it?
        // ak->v_next_best = ak->v_next;
        // occupied_next[u->id] = ak;
        // A.erase(std::remove(A.begin(), A.end(), ak), A.end());
        continue;
      }
      else{
        refresh_lists(A);
      }
    }
    // fix the action by updating occupied next
    // reserve
    occupied_next[u->id] = ai;
    ai->v_next = u; //TODO:remove if not required by PIBT
    
    // run OptiPIBT recursive call
    int best_after_subset = OptiPIBT(agents_subset, accumulated_penalty + action_penalty, best_penalty);
    // this should give it some new v_next values
    if (best_after_subset < best_penalty){
      best_penalty = best_after_subset;
      ai->v_next_best = u;
    }
  }
  return best_penalty;
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
    A.push_back(a);
    occupied_now[s->id] = a;
  }
  solution.add(P->getConfigStart());

  // main loop
  int timestep = 0;
  while (true) {
    info(" ", "elapsed:", getSolverElapsedTime(), ", timestep:", timestep);
    
    // plan one step using optipibt
    OptiPIBT(A, 0, 100000000);
    refresh_lists(A);

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

  // get candidates
  Nodes C = ai->v_now->neighbor;
  C.push_back(ai->v_now);
  // randomize
  std::shuffle(C.begin(), C.end(), *MT);
  // sort
  std::sort(C.begin(), C.end(), compare);

  // calculate ideal dist for penalty purposes
  int ideal_dist = pathDist(ai->id, C[0]);  // Distance to goal if taking ideal move
  // std::cout << ideal_dist << std::endl;
  int actual_dist;
  int diff;

  for (auto u : C) {
    // avoid conflicts
    if (occupied_next[u->id] != nullptr) continue;
    if (aj != nullptr && u == aj->v_now) continue;  //prevents swap conflict

    auto ak = occupied_now[u->id];
    // explicitly prevent swap conflict in globally planned agents
    // means u = v_now for pre-planned agent, so we can't swap
    // aka this node is not valid, don't even think about it.
    if (ak != nullptr && ak->v_next == ai->v_now){
      // occupied_next[u->id] = nullptr;
      continue;
    } 

    // reserve
    occupied_next[u->id] = ai;
    ai->v_next = u;

    if (ak != nullptr && ak->v_next == nullptr) {
      if (!funcPIBT(ak, ai)){
        // means occupied_next will be equal to ak 
        // since all other planning for ak failed
        // occupied_next[u->id] = nullptr;
        ai->v_next = nullptr;
        continue;  // replanning failed
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
