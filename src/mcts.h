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

void ComputeAxesAlignedClippingPlanes(Model &m, const int downsampling, vector<Plane> &planes, bool shuffle = false);

class Part
{
public:
  Params params;
  Model current_mesh;
  int next_choice;
  vector<Plane> available_moves;

  Part(Params _params, Model mesh)
  {
    params = _params;
    current_mesh = mesh;
    next_choice = 0;
    ComputeAxesAlignedClippingPlanes(mesh, params.downsampling, available_moves, true);
  }
  Part operator=(const Part &_part)
  {
    params = _part.params;
    current_mesh = _part.current_mesh;
    next_choice = _part.next_choice;
    available_moves = _part.available_moves;

    return (*this);
  }

  Plane get_one_move()
  {
    return available_moves[next_choice++];
  }
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

  State()
  {
    current_cost = 0;
    current_score = INF;
    current_round = 0;
    worst_part_idx = 0;
  }
  State(Params _params)
  {
    params = _params;
    terminal_threshold = params.threshold;
    current_cost = 0;
    current_score = INF;
    current_round = 0;
    worst_part_idx = 0;
  }
  State(Params _params, Model &_initial_part)
  {
    params = _params;
    terminal_threshold = params.threshold;
    current_score = INF;
    current_round = 0;
    worst_part_idx = 0;
    Part p(params, _initial_part);
    current_costs.push_back(INF); // costs for every part
    current_parts.push_back(p);
    initial_part = _initial_part;
    ori_mesh_area = MeshArea(initial_part);
    ori_mesh_volume = MeshVolume(initial_part);
    Model ch;
    initial_part.ComputeCH(ch);
    ori_meshCH_volume = MeshVolume(ch);
    current_cost = 0; // accumulated score
  }
  State(Params _params, vector<double> &_current_costs, vector<Part> &_current_parts, Model &_initial_part)
  {
    params = _params;
    terminal_threshold = params.threshold;
    current_score = INF;
    current_round = 0;
    current_costs = _current_costs;
    current_parts = _current_parts;
    worst_part_idx = 0;
    initial_part = _initial_part;
    ori_mesh_area = MeshArea(initial_part);
    ori_mesh_volume = MeshVolume(initial_part);
    Model ch;
    initial_part.ComputeCH(ch);
    ori_meshCH_volume = MeshVolume(ch);
    current_cost = 0;
  }

  State operator=(const State &_state)
  {
    params = _state.params;
    terminal_threshold = _state.terminal_threshold;
    current_value = _state.current_value;
    current_cost = _state.current_cost;
    current_score = _state.current_score;
    current_round = _state.current_round;
    current_costs = _state.current_costs;
    current_parts = _state.current_parts;
    worst_part_idx = _state.worst_part_idx;
    initial_part = _state.initial_part;
    ori_mesh_area = _state.ori_mesh_area;
    ori_mesh_volume = _state.ori_mesh_volume;
    ori_meshCH_volume = _state.ori_meshCH_volume;

    return (*this);
  }

  void set_current_value(pair<Plane, int> value)
  {
    current_value = value;
  }
  pair<Plane, int> get_current_value()
  {
    return current_value;
  }
  void set_current_round(int round)
  {
    current_round = round;
  }
  int get_current_round()
  {
    return current_round;
  }
  bool is_terminal()
  {
    if (current_round >= params.mcts_max_depth || (int)current_parts[worst_part_idx].available_moves.size() == 0)
      return true;
    return false;
  }
  double compute_reward()
  {
    current_score = ComputeReward(params, ori_meshCH_volume, current_costs, current_parts, worst_part_idx, ori_mesh_area, ori_mesh_volume);
    return current_score;
  }
  Plane one_move(int worst_part_idx)
  {
    return current_parts[worst_part_idx].get_one_move();
  }
  State get_next_state_with_random_choice()
  {
    // choose the mesh with highest score and pick one available move
    Plane cutting_plane = one_move(worst_part_idx);
    Model pos, neg, posCH, negCH;
    double cut_area;
    bool flag = Clip(current_parts[worst_part_idx].current_mesh, pos, neg, cutting_plane, cut_area);
    if (!flag)
    {
      State next_state(params, current_costs, current_parts, initial_part);
      next_state.current_cost = INF;
      next_state.current_round = params.mcts_max_depth;

      return next_state;
    }
    else
    {
      vector<double> _current_costs;
      vector<Part> _current_parts;
      for (int i = 0; i < (int)current_parts.size(); i++)
      {
        if (i != worst_part_idx)
        {
          _current_costs.push_back(current_costs[i]);
          _current_parts.push_back(current_parts[i]);
        }
      }
      pos.ComputeCH(posCH);
      neg.ComputeCH(negCH);
      double cost_pos = ComputeRv(pos, posCH, params.rv_k);
      double cost_neg = ComputeRv(neg, negCH, params.rv_k);
      Part part_pos(params, pos);
      Part part_neg(params, neg);
      _current_parts.push_back(part_pos);
      _current_parts.push_back(part_neg);
      _current_costs.push_back(cost_pos);
      _current_costs.push_back(cost_neg);

      State next_state(params, _current_costs, _current_parts, initial_part);
      _current_costs.clear();
      _current_parts.clear();

      next_state.current_value = make_pair(cutting_plane, worst_part_idx);
      double single_reward = next_state.compute_reward();
      next_state.current_cost = current_cost + single_reward;
      next_state.current_round = current_round + 1;

      return next_state;
    }
  }
};

int node_idx;

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
  Node(Params _params)
  {
    params = _params;
    parent = NULL;
    visit_times = 0;
    quality_value = INF;
    // quality_value = 0;
    state = NULL;
    idx = node_idx++;
  }
  ~Node()
  {
    if (state != NULL)
      delete state;
  }
  Node operator=(const Node &_node)
  {
    params = _node.params;
    children = _node.children;
    visit_times = _node.visit_times;
    quality_value = _node.quality_value;
    state = _node.state;
    parent = _node.parent;

    return (*this);
  }
  void set_state(State _state)
  {
    state = new State(params);
    *state = _state;
  }
  State *get_state()
  {
    return state;
  }
  void set_parent(Node *_parent)
  {
    parent = _parent;
  }
  Node *get_parent()
  {
    return parent;
  }
  vector<Node *> get_children()
  {
    return children;
  }
  double get_visit_times()
  {
    return visit_times;
  }
  void set_visit_times(double _visit_times)
  {
    visit_times = _visit_times;
  }
  void visit_times_add_one()
  {
    visit_times += 1;
  }
  void set_quality_value(double _quality_value)
  {
    quality_value = _quality_value;
  }
  double get_quality_value()
  {
    return quality_value;
  }
  void quality_value_add_n(double n)
  {
    quality_value = min(quality_value, n);
  }
  bool is_all_expand()
  {
    State *_state = get_state();
    int current_max_expand_nodes = (int)_state->current_parts[_state->worst_part_idx].available_moves.size();
    return (int)children.size() == current_max_expand_nodes;
  }
  void add_child(Node *sub_node)
  {
    sub_node->set_parent(this);
    children.push_back(sub_node);
  }
};

Node *tree_policy(Node *node);
double default_policy(Node *node, Params &params);
Node *expand(Node *node);
Node *best_child(Node *node, bool is_exploration, double initial_cost = 0.1);
void backup(Node *node, double reward);
Node *MonteCarloTreeSearch(Params &params, Node *node);

bool clip_by_path(Model &m, double &final_cost, Params &params, Plane &first_plane, vector<Plane> &best_path)
{
  int worst_idx = 0;
  vector<double> scores;
  vector<Model> parts;
  bool flag;
  double tmp;
  double max_cost;

  Model ch;
  m.ComputeCH(ch);

  Model pos, neg, posCH, negCH;
  flag = Clip(m, pos, neg, first_plane, tmp);
  if (!flag)
  {
    final_cost = INF;
    return false;
  }
  pos.ComputeCH(posCH);
  neg.ComputeCH(negCH);
  double pos_cost = ComputeRv(pos, posCH, params.rv_k);
  double neg_cost = ComputeRv(neg, negCH, params.rv_k);
  scores.push_back(pos_cost);
  scores.push_back(neg_cost);
  parts.push_back(pos);
  parts.push_back(neg);

  if (pos_cost > neg_cost)
    worst_idx = 0;
  else
    worst_idx = 1;

  final_cost = max(pos_cost, neg_cost);
  int N = (int)best_path.size();
  for (int i = 1; i < N; i++)
  {
    Model _pos, _neg, _posCH, _negCH;
    flag = Clip(parts[worst_idx], _pos, _neg, best_path[N - 1 - i], tmp);
    if (!flag)
    {
      final_cost = INF;
      return false;
    }
    _pos.ComputeCH(_posCH);
    _neg.ComputeCH(_negCH);
    double _pos_cost = ComputeRv(_pos, _posCH, params.rv_k);
    double _neg_cost = ComputeRv(_neg, _negCH, params.rv_k);

    vector<double> _scores;
    vector<Model> _parts;
    for (int j = 0; j < (int)parts.size(); j++)
    {
      if (j != worst_idx)
      {
        _scores.push_back(scores[j]);
        _parts.push_back(parts[j]);
      }
    }
    scores = _scores;
    parts = _parts;
    _scores.clear();
    _parts.clear();

    scores.push_back(_pos_cost);
    scores.push_back(_neg_cost);
    parts.push_back(_pos);
    parts.push_back(_neg);

    max_cost = scores[0];
    worst_idx = 0;
    for (int j = 1; j < (int)scores.size(); j++)
      if (scores[j] > final_cost)
      {
        worst_idx = j;
        max_cost = scores[j];
      }

    final_cost += max_cost;
  }
  final_cost /= N;

  return true;
}

bool TernaryMCTS(Model &m, Params &params, Plane &bestplane, vector<Plane> &best_path, double best_cost, bool mode = 1, double epsilon = 0.0001)
{
  double *bbox = m.GetBBox();
  double interval;
  double minItv = 0.01;
  size_t thres = params.tri_thres;
  double best_within_three = INF;
  Plane best_plane_within_three;
  double Hmin;
  bool flag;

  if (fabs(bestplane.a - 1.0) < 1e-4 || !mode)
  {
    double left, right;
    interval = max(0.01, abs(bbox[0] - bbox[1]) / ((double)params.downsampling + 1));
    if (mode == true)
    {
      left = max(bbox[0] + minItv, -1.0 * bestplane.d - interval);
      right = min(bbox[1] - minItv, -1.0 * bestplane.d + interval);
    }
    else
    {
      left = bbox[0] + minItv;
      right = bbox[1] - minItv;
    }
    if (mode && left > right)
      return false;
    size_t iter = 0;
    double res;
    while (left + epsilon < right && iter++ < thres)
    {
      Model pos1, neg1, posCH1, negCH1, pos2, neg2, posCH2, negCH2;
      double margin = (right - left) / 3.0;
      double m1 = left + margin;
      double m2 = m1 + margin;
      Plane p1 = Plane(1.0, 0.0, 0.0, -m1), p2 = Plane(1.0, 0.0, 0.0, -m2);

      double E1;
      flag = clip_by_path(m, E1, params, p1, best_path);

      double E2;
      flag = clip_by_path(m, E2, params, p2, best_path);

      if (E1 < E2)
      {
        right = m2;
        res = m1;
      }
      else
      {
        left = m1;
        res = m2;
      }
    }
    Plane tp;
    tp = Plane(1.0, 0.0, 0.0, -res);
    flag = clip_by_path(m, Hmin, params, tp, best_path);

    if (Hmin < best_cost)
      bestplane = tp;
    if (!mode)
    {
      if (Hmin < best_within_three)
      {
        best_within_three = Hmin;
        best_plane_within_three = bestplane;
      }
    }
  }
  if (fabs(bestplane.b - 1.0) < 1e-4 || !mode)
  {
    double left, right;
    interval = max(0.01, abs(bbox[2] - bbox[3]) / ((double)params.downsampling + 1));
    if (mode == true)
    {
      left = max(bbox[2] + minItv, -1.0 * bestplane.d - interval);
      right = min(bbox[3] - minItv, -1.0 * bestplane.d + interval);
    }
    else
    {
      left = bbox[2] + minItv;
      right = bbox[3] - minItv;
    }
    if (mode && left > right)
      return false;
    size_t iter = 0;
    double res;
    while (left + epsilon < right && iter++ < thres)
    {
      Model pos1, neg1, posCH1, negCH1, pos2, neg2, posCH2, negCH2;
      double margin = (right - left) / 3.0;
      double m1 = left + margin;
      double m2 = m1 + margin;
      Plane p1 = Plane(0.0, 1.0, 0.0, -m1), p2 = Plane(0.0, 1.0, 0.0, -m2);

      double E1;
      flag = clip_by_path(m, E1, params, p1, best_path);

      double E2;
      flag = clip_by_path(m, E2, params, p2, best_path);
      if (E1 < E2)
      {
        right = m2;
        res = m1;
      }
      else
      {
        left = m1;
        res = m2;
      }
    }
    Plane tp;
    tp = Plane(0.0, 1.0, 0.0, -res);
    flag = clip_by_path(m, Hmin, params, tp, best_path);

    if (Hmin < best_cost)
      bestplane = tp;
    if (!mode)
    {
      if (Hmin < best_within_three)
      {
        best_within_three = Hmin;
        best_plane_within_three = bestplane;
      }
    }
  }
  if (fabs(bestplane.c - 1.0) < 1e-4 || !mode)
  {
    double left, right;
    interval = max(0.01, abs(bbox[4] - bbox[5]) / ((double)params.downsampling + 1));
    if (mode == true)
    {
      left = max(bbox[4] + minItv, -1.0 * bestplane.d - interval);
      right = min(bbox[5] - minItv, -1.0 * bestplane.d + interval);
    }
    else
    {
      left = bbox[4] + minItv;
      right = bbox[5] - minItv;
    }
    if (mode && left > right)
      return false;
    size_t iter = 0;
    double res;
    while (left + epsilon < right && iter++ < thres)
    {
      Model pos1, neg1, posCH1, negCH1, pos2, neg2, posCH2, negCH2;
      double margin = (right - left) / 3.0;
      double m1 = left + margin;
      double m2 = m1 + margin;
      Plane p1 = Plane(0.0, 0.0, 1.0, -m1), p2 = Plane(0.0, 0.0, 1.0, -m2);

      double E1;
      flag = clip_by_path(m, E1, params, p1, best_path);

      double E2;
      flag = clip_by_path(m, E2, params, p2, best_path);
      if (E1 < E2)
      {
        right = m2;
        res = m1;
      }
      else
      {
        left = m1;
        res = m2;
      }
    }
    Plane tp;
    tp = Plane(0.0, 0.0, 1.0, -res);
    flag = clip_by_path(m, Hmin, params, tp, best_path);

    if (Hmin < best_cost)
      bestplane = tp;
    if (!mode)
    {
      if (Hmin < best_within_three)
      {
        best_within_three = Hmin;
        best_plane_within_three = bestplane;
      }
    }
  }
  if (!mode)
  {
    if (best_within_three > INF - 1)
      return false;
    bestplane = best_plane_within_three;
  }

  return true;
}
void RefineMCTS(Model &m, Params &params, Plane &bestplane, vector<Plane> &best_path, double best_cost, double epsilon = 0.0001)
{
  double *bbox = m.GetBBox();
  double downsample;
  double interval = 0.01;
  bool flag;
  if (fabs(bestplane.a - 1.0) < 1e-4)
  {
    double left, right;
    downsample = max(0.01, abs(bbox[0] - bbox[1]) / ((double)params.downsampling + 1));
    left = max(bbox[0] + interval, -1.0 * bestplane.d - downsample);
    right = min(bbox[1] - interval, -1.0 * bestplane.d + downsample);

    double min_cost = INF;
    for (double i = left; i <= right; i += interval)
    {
      double E;
      Plane pl = Plane(1.0, 0.0, 0.0, -i);
      flag = clip_by_path(m, E, params, pl, best_path);
      if (E < best_cost && E < min_cost)
      {
        min_cost = E;
        bestplane = pl;
      }
    }
  }
  else if (fabs(bestplane.b - 1.0) < 1e-4)
  {
    double left, right;
    downsample = max(0.01, abs(bbox[2] - bbox[3]) / ((double)params.downsampling + 1));
    left = max(bbox[2] + interval, -1.0 * bestplane.d - downsample);
    right = min(bbox[3] - interval, -1.0 * bestplane.d + downsample);

    double min_cost = INF;
    for (double i = left; i <= right; i += interval)
    {
      double E;
      Plane pl = Plane(0.0, 1.0, 0.0, -i);
      flag = clip_by_path(m, E, params, pl, best_path);
      if (E < best_cost && E < min_cost)
      {
        min_cost = E;
        bestplane = pl;
      }
    }
  }
  else if (fabs(bestplane.c - 1.0) < 1e-4)
  {
    double left, right;
    downsample = max(0.01, abs(bbox[4] - bbox[5]) / ((double)params.downsampling + 1));
    left = max(bbox[4] + interval, -1.0 * bestplane.d - downsample);
    right = min(bbox[5] - interval, -1.0 * bestplane.d + downsample);

    double min_cost = INF;
    for (double i = left; i <= right; i += interval)
    {
      double E;
      Plane pl = Plane(0.0, 0.0, 1.0, -i);
      flag = clip_by_path(m, E, params, pl, best_path);
      if (E < best_cost && E < min_cost)
      {
        min_cost = E;
        bestplane = pl;
      }
    }
  }
  else
  {
    cout << "RefineMCTS Error!" << endl;
    assert(0);
  }
}

void ComputeAxesAlignedClippingPlanes(Model &m, const int downsampling, vector<Plane> &planes, bool shuffle)
{
  double *bbox = m.GetBBox();
  double interval;
  double eps = 1e-6;
  interval = max(0.01, abs(bbox[0] - bbox[1]) / ((double)downsampling + 1));
  for (double i = bbox[0] + max(0.015, interval); i <= bbox[1] - max(0.015, interval) + eps; i += interval)
  {
    planes.push_back(Plane(1.0, 0.0, 0.0, -i));
  }
  interval = max(0.01, abs(bbox[2] - bbox[3]) / ((double)downsampling + 1));
  for (double i = bbox[2] + max(0.015, interval); i <= bbox[3] - max(0.015, interval) + eps; i += interval)
  {
    planes.push_back(Plane(0.0, 1.0, 0.0, -i));
  }
  interval = max(0.01, abs(bbox[4] - bbox[5]) / ((double)downsampling + 1));
  for (double i = bbox[4] + max(0.015, interval); i <= bbox[5] - max(0.015, interval) + eps; i += interval)
  {
    planes.push_back(Plane(0.0, 0.0, 1.0, -i));
  }

  if (shuffle)
    random_shuffle(planes.begin(), planes.end());
}

bool ComputeBestRvClippingPlane(Model &m, Params &params, vector<Plane> &planes, Plane &bestplane, double &bestcost)
{
  if ((int)planes.size() == 0)
    return false;
  double H_min = INF;
  double cut_area;
  bool flag;
  for (int i = 0; i < (int)planes.size(); i++)
  {
    Model pos, neg, posCH, negCH;

    flag = Clip(m, pos, neg, planes[i], cut_area);
    double H;
    if (!flag)
      H = INF;
    else
    {
      if (pos.points.size() <= 0 || neg.points.size() <= 0)
        continue;

      pos.ComputeCH(posCH);
      neg.ComputeCH(negCH);

      H = ComputeTotalRv(m, pos, posCH, neg, negCH, params.rv_k, planes[i]);
    }

    if (H < H_min)
    {
      H_min = H;
      bestplane = planes[i];
      bestcost = H;
    }
  }

  return true;
}

double ComputeReward(Params &params, double meshCH_v, vector<double> &current_costs, vector<Part> &current_parts, int &worst_part_idx, double ori_mesh_area, double ori_mesh_volume)
{
  double reward = 0;
  double h_max = 0;
  for (int i = 0; i < (int)current_costs.size(); i++)
  {
    double h = current_costs[i];
    if (h > h_max)
    {
      h_max = h;
      worst_part_idx = i;
    }
    reward += h;
  }

  return h_max;
}

Node *tree_policy(Node *node, double initial_cost, bool &flag)
{
  while (node->get_state()->is_terminal() == false)
  {
    if (node->is_all_expand())
    {
      node = best_child(node, true, initial_cost);
    }
    else
    {
      Node *sub_node = expand(node);
      return sub_node;
    }
  }

  return node;
}

double default_policy(Node *node, Params &params, vector<Plane> &current_path) // evaluate the quality until the mesh is all cut MAX_ROUND times
{
  State *original_state = node->get_state();
  State current_state = *original_state;
  double current_state_reward;
  original_state->worst_part_idx = current_state.worst_part_idx;

  while (current_state.is_terminal() == false)
  {
    vector<Plane> planes;
    Plane bestplane;
    double bestcost, cut_area;
    ComputeAxesAlignedClippingPlanes(current_state.current_parts[current_state.worst_part_idx].current_mesh, params.mcts_downsample, planes);
    if ((int)planes.size() == 0)
    {
      break;
    }
    ComputeBestRvClippingPlane(current_state.current_parts[current_state.worst_part_idx].current_mesh, params, planes, bestplane, bestcost);

    Model pos, neg, posCH, negCH;
    bool clipf = Clip(current_state.current_parts[current_state.worst_part_idx].current_mesh, pos, neg, bestplane, cut_area);
    if (!clipf)
    {
      cout << "Wrong MCTS clip proposal!" << endl;
      exit(0);
    }
    current_path.push_back(bestplane);
    vector<double> _current_costs;
    vector<Part> _current_parts;
    for (int i = 0; i < (int)current_state.current_parts.size(); i++)
    {
      if (i != current_state.worst_part_idx)
      {
        _current_costs.push_back(current_state.current_costs[i]);
        _current_parts.push_back(current_state.current_parts[i]);
      }
    }
    pos.ComputeCH(posCH);
    neg.ComputeCH(negCH);
    double cost_pos = ComputeRv(pos, posCH, params.rv_k);
    double cost_neg = ComputeRv(neg, negCH, params.rv_k);

    Part part_pos(params, pos);
    Part part_neg(params, neg);
    _current_parts.push_back(part_pos);
    _current_parts.push_back(part_neg);
    _current_costs.push_back(cost_pos);
    _current_costs.push_back(cost_neg);

    current_state.current_costs = _current_costs;
    current_state.current_parts = _current_parts;

    _current_costs.clear();
    _current_parts.clear();

    current_state_reward = current_state.compute_reward();
    current_state.current_cost += current_state_reward;

    current_state.current_round = current_state.current_round + 1;
  }

  return current_state.current_cost / params.mcts_max_depth; // mean
}

Node *expand(Node *node)
{
  State new_state = node->get_state()->get_next_state_with_random_choice();

  Node *sub_node = new Node(node->params);
  sub_node->set_state(new_state);
  node->add_child(sub_node);

  return sub_node;
}

Node *best_child(Node *node, bool is_exploration, double initial_cost)
{
  double best_score = INF;
  Node *best_sub_node = NULL;

  vector<Node *> children = node->get_children();
  for (int i = 0; i < (int)children.size(); i++)
  {
    double C;
    Node *sub_node = children[i];
    if (is_exploration)
      C = initial_cost / sqrt(2.0);
    else
      C = 0.0;

    double left = sub_node->get_quality_value();
    double right = 2.0 * log(node->get_visit_times()) / sub_node->get_visit_times();
    double score = left - C * sqrt(right);

    if (score < best_score)
    {
      best_sub_node = sub_node;
      best_score = score;
    }
  }

  return best_sub_node;
}

void backup(Node *node, double reward, vector<Plane> &current_path, vector<Plane> &best_path)
{
  vector<Plane> tmp_path;
  int N = (int)current_path.size();
  for (int i = 0; i < N; i++)
    tmp_path.push_back(current_path[N - 1 - i]);

  while (node != NULL)
  {
    if (node->get_state()->current_round == 0 && node->quality_value > reward)
      best_path = tmp_path;
    tmp_path.push_back(node->get_state()->current_value.first);

    node->visit_times_add_one();
    node->quality_value_add_n(reward);
    node = node->parent;
  }

  tmp_path.clear();
}

void free_tree(Node *root, int idx)
{
  if (root->get_children().size() == 0)
  {
    delete root;
    return;
  }

  vector<Node *> children = root->get_children();
  while (idx < (int)children.size())
  {
    free_tree(children[idx++], 0);
  }
  delete root;
  return;
}

Node *MonteCarloTreeSearch(Params &params, Node *node, vector<Plane> &best_path)
{
  int computation_budget = params.mcts_iteration;
  Model initial_mesh = node->get_state()->current_parts[0].current_mesh, initial_ch;
  initial_mesh.ComputeCH(initial_ch);
  double cost = ComputeRv(initial_mesh, initial_ch, params.rv_k) / params.mcts_max_depth;
  node_idx = 1;
  vector<Plane> current_path;
  srand(params.seed);

  for (int i = 0; i < computation_budget; i++)
  {
    current_path.clear();
    bool flag = false;
    Node *expand_node = tree_policy(node, cost, flag);
    double reward = default_policy(expand_node, params, current_path);
    backup(expand_node, reward, current_path, best_path);
  }
  Node *best_next_node = best_child(node, false);

  return best_next_node;
}
