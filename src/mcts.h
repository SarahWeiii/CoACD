#pragma once

#include <algorithm>
#include <assert.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>
#include <cstdlib>
#include <time.h>
#include <assert.h>
#include <algorithm>
#include <set>
#include <map>
#include <unordered_map>

#include "config.h"
#include "model_obj.h"
#include "cost.h"
#include "clip.h"

namespace coacd
{

  constexpr int MCTS_RANDOM_CUT = 1;

  void ComputeAxesAlignedClippingPlanes(Model &m, const int downsampling, vector<Plane> &planes, bool shuffle = false);

  class Part
  {
  public:
    Params params;
    Model current_mesh;
    int next_choice;
    vector<Plane> available_moves;

    Part(Params _params, Model mesh);
    Part operator=(const Part &_part);
    Plane get_one_move();
  };

  double ComputeReward(Params &params, double meshCH_v, vector<double> &current_costs, vector<Part> &current_parts, int &worst_part_idx, double ori_mesh_area, double ori_mesh_volume);

  class State
  {
  public:
    double terminal_threshold;
    pair<Plane, int> current_value;
    double current_cost;
    double current_score;
    int current_round;
    Model initial_part;
    double ori_mesh_area;
    double ori_mesh_volume;
    double ori_meshCH_volume;
    vector<double> current_costs;
    vector<Part> current_parts;
    int worst_part_idx;
    Params params;

    State();
    State(Params _params);
    State(Params _params, Model &_initial_part);
    State(Params _params, vector<double> &_current_costs, vector<Part> &_current_parts, Model &_initial_part);

    State operator=(const State &_state);

    void set_current_value(pair<Plane, int> value);
    pair<Plane, int> get_current_value();
    void set_current_round(int round);
    int get_current_round();
    bool is_terminal();
    double compute_reward();
    Plane one_move(int worst_part_idx);
    State get_next_state_with_random_choice();
  };

  class Node
  {
  public:
    int idx;
    vector<Node *> children;

    int visit_times;
    double quality_value;

    State *state;

    Params params;
    Node *parent;

    Node(Params _params);
    ~Node();
    Node operator=(const Node &_node);
    void set_state(State _state);
    State *get_state();
    void set_parent(Node *_parent);
    Node *get_parent();
    vector<Node *> get_children();
    double get_visit_times();
    void set_visit_times(double _visit_times);
    void visit_times_add_one();
    void set_quality_value(double _quality_value);
    double get_quality_value();
    void quality_value_add_n(double n);
    bool is_all_expand();
    void add_child(Node *sub_node);
  };

  Node *tree_policy(Node *node);
  double default_policy(Node *node, Params &params);
  Node *expand(Node *node);
  Node *best_child(Node *node, bool is_exploration, double initial_cost = 0.1);
  void backup(Node *node, double reward);
  Node *MonteCarloTreeSearch(Params &params, Node *node, vector<Plane> &best_path);

  bool clip_by_path(Model &m, double &final_cost, Params &params, Plane &first_plane, vector<Plane> &best_path);
  bool TernaryMCTS(Model &m, Params &params, Plane &bestplane, vector<Plane> &best_path, double best_cost, bool mode = 1, double epsilon = 0.0001);
  void RefineMCTS(Model &m, Params &params, Plane &bestplane, vector<Plane> &best_path, double best_cost, double epsilon = 0.0001);
  void ComputeAxesAlignedClippingPlanes(Model &m, const int downsampling, vector<Plane> &planes, bool shuffle);
  bool ComputeBestRvClippingPlane(Model &m, Params &params, vector<Plane> &planes, Plane &bestplane, double &bestcost);
  double ComputeReward(Params &params, double meshCH_v, vector<double> &current_costs, vector<Part> &current_parts, int &worst_part_idx, double ori_mesh_area, double ori_mesh_volume);
  void free_tree(Node *root, int idx);
}