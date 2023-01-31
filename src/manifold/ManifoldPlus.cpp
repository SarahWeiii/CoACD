#include "ManifoldPlus.h"

int ManifoldPlus(ofstream &of, string input_model, Model &output, int depth)
{
  clock_t start, end;
  MatrixD V, out_V;
  MatrixI F, out_F;
  ReadOBJ(input_model.c_str(), &V, &F);

  of << " - Pre-processing (manifoldPlus)" << endl;
  of << "\tdepth: " << depth << endl;
  cout << " - Pre-processing (manifoldPlus)" << endl;
  cout << "\tdepth: " << depth << endl;

  ManifoldP manifold;
  start = clock();
  manifold.ProcessManifold(V, F, depth, &out_V, &out_F);
  end = clock();
  of << "ManifoldPlus time: " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;
  cout << "ManifoldPlus time: " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;

  output.Load(out_V, out_F);

  return 0;
}

int ManifoldPlus(Model &input, Model &output, bool if_cout, bool if_log, string logfile, int depth)
{
  clock_t start, end;
  MatrixD V, out_V;
  MatrixI F, out_F;

  V.resize(input.points.size(), 3);
  F.resize(input.points.size(), 3);
  memcpy(V.data(), input.points.data(), sizeof(double) * input.points.size());
  memcpy(F.data(), input.triangles.data(),
         sizeof(int) * input.triangles.size());

  logger(false, if_log, logfile) << " - Pre-processing (manifoldPlus)" << endl;
  logger(false, if_log, logfile) << "\tdepth: " << depth << endl;
  logger(if_cout, false) << " - Pre-processing (manifoldPlus)" << endl;
  logger(if_cout, false) << "\tdepth: " << depth << endl;

  ManifoldP manifold;
  start = clock();
  manifold.ProcessManifold(V, F, depth, &out_V, &out_F);
  end = clock();

  output.Load(out_V, out_F);

  return 0;
}