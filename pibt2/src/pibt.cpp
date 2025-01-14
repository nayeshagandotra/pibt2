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

int PIBT::calculate_penalty(Agent* a) {

  // A two nodes
  auto compare = [&](Node* const v, Node* const u) {
    int d_v = pathDist(a->id, v);
    int d_u = pathDist(a->id, u);
    if (d_v != d_u) return d_v < d_u;
    // tie break
    if (occupied_now[v->id] != nullptr && occupied_now[u->id] == nullptr)
      return false;
    if (occupied_now[v->id] == nullptr && occupied_now[u->id] != nullptr)
      return true;
    return false;
  };

  Nodes C = a->v_now->neighbor;
  C.push_back(a->v_now);

  std::sort(C.begin(), C.end(), compare);

  // calculate ideal dist for penalty purposes
  int ideal_dist = pathDist(a->id, C[0]);  // Distance to goal if taking ideal move
  int actual_dist = pathDist(a->id, a->v_next_best); // Distance to goal based on suggested move
  return actual_dist - ideal_dist;
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

void PIBT::group_optipibt(Agents A){
  this->timestep_penalty = 0;
  group_no = 0;
  std::sort(A.begin(), A.end(), compareAgents);
  plan_one_step(A);
  print_penalty("costs.txt", timestep_penalty);  // LOGGING

  // Use an iterator to traverse the groups vector
  for (size_t i = 0; i < groups.size(); ++i) {
    size_t initial_size = groups.size();
    group_no = i;
    Agents* g = groups[i];
    A_copy = *g;
    best_penalty = 100000;
    OptiPIBT(A_copy, nullptr, 0);
    refresh_lists(A_copy);
    // If new groups are added, they will be processed in the next iteration
    for (auto a: A_copy){
      occupied_next[a->v_next_best->id] = a; //reserve
      a->v_next = a->v_next_best;
    }
  }

  for (auto a : A){
    if (a->v_next_best == nullptr){
      a->v_next_best = a->v_next; // for non-conflicting agents, this should be populated with the pibt answers
    }
  }
}

bool PIBT::addToGroup(Agent* ai, Agent* aj){
  Agents* ng;
  if (ai->group != nullptr && aj->group == nullptr){
    ng = ai->group;
    // add aj to group- assume ai is already in g
    ng->push_back(aj);
    aj->group = ng;
    return true;
  } else if (aj->group != nullptr && ai->group == nullptr){
    ng = aj->group;
    ng->push_back(ai);
    ai->group = ng;
    return true;
  } else if (aj->group == nullptr && ai->group == nullptr){
    // neither have group
    ng = new Agents();
    ng->push_back(ai);
    ng->push_back(aj);
    ai->group = ng;
    aj->group = ng;
    groups.push_back(ng);
  } else if ((aj->group != nullptr && ai->group != nullptr) && aj->group != ai->group){
    // merge groups
    Agents* group1 = ai->group;
    Agents* group2 = aj->group;

    // Create a new group that contains all agents from both groups
    Agents* new_group = new Agents();
    std::set<Agent*> unique_agents;

    // Insert all agents from group1 into the set
    for (const auto& agent : *group1) {
        unique_agents.insert(agent);
    }

    // Insert all agents from group2 into the set, avoiding duplicates
    for (const auto& agent : *group2) {
        unique_agents.insert(agent);
    }

    // Reserve space in new_group to avoid unnecessary reallocations
    new_group->reserve(unique_agents.size());

    // Copy unique agents back to new_group
    for (const auto& agent : unique_agents) {
        new_group->push_back(agent);
    }

    // Update group pointers for all agents in both groups
    for (auto& agent : *group1) {
      agent->group = new_group;
    }
    for (auto& agent : *group2) {
      agent->group = new_group;
    }

    // Add the new group to the groups list
    groups.push_back(new_group);
  }
  return false;
}

std::pair<bool, int> PIBT::OptiPIBT(Agents A, Agent* aj, int accumulated_penalty){

  std::sort(A.begin(), A.end(), compareAgents);

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
  int round_bestp = 100000;
  bool group_exists = false;

  // counting number of skipped actions
  int n_avail_acts = 0;
  int n_skipped_acts = 0;

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
      group_exists = false;
      for (size_t i = group_no + 1; i < groups.size(); ++i) {
        if (groups[i] == ai->group) {
            group_exists = true;
            break;
        }
      }
      if (addToGroup(occupied_next[u->id], ai) && !group_exists){
        // need to add this new group to the groups list because something new has been added
        groups.push_back(ai->group);
      }
      n_skipped_acts += 1;
      continue;
    }

    // if action is causing swap conflict, continue
    auto ak = occupied_now[u->id];
    if (ak != nullptr && ak != ai){
      // The agent at vertex 'id' in occupied_now is not nullptr and not in A
      // and it's not the current agent
      // means it's been reserved by a previous fixed agent
      // // check if we have a swap conflict
      if (ak->v_next == ai->v_now){
        group_exists = false;
        for (size_t i = group_no + 1; i < groups.size(); ++i) {
            if (groups[i] == ai->group) {
                group_exists = true;
                break;
            }
        }
        if (addToGroup(ak, ai) && !group_exists){
          // need to add this new group to the groups list because something new has been added
          groups.push_back(ai->group);
        }
        // the node we are trying to move to is currently occupied by an agent that 
        // wants to move to us or stay at u. since we are lower priority, we sacrifice this action.
        n_skipped_acts += 1;
        continue;
      }
    }
    // option 2: try recursing through ak and skip action if it doesn't improve
    occupied_next[u->id] = ai;
    ai->v_next = u; 
    // there is an unplanned agent here, let's see if we can comfortably move it
    if (ak != nullptr && ak->v_next == nullptr){
      group_exists = false;
      for (size_t i = group_no + 1; i < groups.size(); ++i) {
        if (groups[i] == ai->group) {
            group_exists = true;
            break;
        }
      }
      if (addToGroup(ak, ai) && !group_exists){
        // need to add this new group to the groups list because something new has been added
        groups.push_back(ai->group);
      }
      auto [failed, bas] = OptiPIBT(agents_subset, ak, accumulated_penalty + action_penalty);
      if (failed){
        n_skipped_acts += 1;
        // we tried moving to this action and moving other agents accordingly, but agent ak is stuck
        ai->v_next = nullptr;
        continue;
      }
      round_bestp = std::min(round_bestp, action_penalty + bas);
      continue; // regardless, try another action now, this one has been explored already (if it's best, itll get saved at the bottom)
    }
   
    
    // run OptiPIBT recursive call if not last agent
    int best_after_subset = 0;
    bool f = true;
    if (!agents_subset.empty()) {
      // agents_subset is not empty
      std::pair<bool, int> result = OptiPIBT(agents_subset, nullptr, accumulated_penalty + action_penalty);
      f = result.first;
      best_after_subset = result.second;
    } 
    round_bestp = std::min(round_bestp, action_penalty + best_after_subset);
    // this should give it some new v_next values
    if (accumulated_penalty + action_penalty + best_after_subset < best_penalty){
      best_penalty = accumulated_penalty + action_penalty + best_after_subset;
      for (auto agent : A_copy) {
        // if (agent != nullptr && agent->group == ai->group) {
            // Set the agent's v_next_best to v_next (which should be correct atm)
            agent->v_next_best = agent->v_next; 
        // }
      }
    }
  }
  // either all moves have failed or next best has been found
  if (n_skipped_acts == n_avail_acts){
    // failed to secure node
    occupied_next[ai->v_now->id] = ai;
    ai->v_next = ai->v_now;
    return std::make_pair(true, round_bestp);
  }
  return std::make_pair(false, round_bestp);
}

void PIBT::refresh_lists(Agents A){
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
    group_optipibt(A);
    refresh_lists(A);
    groups.clear();

    // acting
    bool check_goal_cond = true;
    Config config(P->getNum(), nullptr);
    int best_tsp = 0;
    for (auto a : A) {
      best_tsp += calculate_penalty(a);
      // clear
      a->group = nullptr;
      if (occupied_now[a->v_now->id] == a) occupied_now[a->v_now->id] = nullptr;
      // set next location
      config[a->id] = a->v_next_best;
      occupied_now[a->v_next_best->id] = a;
      // check goal condition
      check_goal_cond &= (a->v_next_best == a->g);

      if (timestep > 50 && a->v_now != a->g){
        std::cout << "the problem is" << a->id << " " << a->g->pos.x << " " << a->g->pos.y << std::endl;
      }
      // update priority
      a->elapsed = (a->v_next_best == a->g) ? 0 : a->elapsed + 1;
      // reset params
      a->v_now = a->v_next_best;
      a->v_next_best = nullptr;
    }
    // update plan
    solution.add(config);
    print_penalty("costs.txt", best_tsp);
  
    // success
    if (check_goal_cond) {
      solved = true;
    }

    if (solved){
      break;
    }

    ++timestep;

    // failed  
    if (timestep >= max_timestep || overCompTime()) {
      break;
    }
  }

  // memory clear
  for (auto a : A) delete a;
}

bool PIBT::funcPIBT(Agent* ai, Agent* aj)
{
  // A two nodes
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

  // int tsp = timestep_penalty;

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
  int diff;

  for (auto u : C) {
    // avoid conflicts
    if (occupied_next[u->id] != nullptr){
      addToGroup(ai, occupied_next[u->id]);
      continue;
    } 
    if (aj != nullptr && u == aj->v_now){
      // addToGroup(ai, aj);
      continue;  //prevents swap conflict
    } 

    auto ak = occupied_now[u->id];
    // explicitly prevent swap conflict in globally planned agents
    // means u = v_now for pre-planned agent, so we can't swap
    // aka this node is not valid, don't even think about it.
    if (ak != nullptr){
      if (ak->v_next == ai->v_now){
        addToGroup(ai, ak);
        continue;
      }
    } 

    // reserve
    occupied_next[u->id] = ai;
    ai->v_next = u;

    if (ak != nullptr && ak->v_next == nullptr) {
      addToGroup(ai, ak);
      if (!funcPIBT(ak, ai)){
        // means occupied_next will be equal to ak 
        // since all other planning for ak failed
        continue;  // replanning failed
      }
    }

    // find penalty
    actual_dist = pathDist(ai->id, ai->v_next);
    diff = actual_dist - ideal_dist;
    this->timestep_penalty += diff; //0 if equal
    // success to plan next one step
    return true;
  }

  // failed to secure node
  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;

  // find penalty
  actual_dist = pathDist(ai->id, ai->v_next);
  diff = actual_dist - ideal_dist;
  this->timestep_penalty += diff; //0 if equal
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
