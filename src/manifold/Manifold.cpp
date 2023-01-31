#include "Manifold.h"

int Manifold(ofstream &of, string input_model, Model &output, int resolution)
{
  clock_t start, end;

  Model_OBJ obj;
  char *cstr = new char[input_model.length() + 1];
  strcpy(cstr, input_model.c_str());
  obj.Load(cstr);
  delete[] cstr;

  of << " - Pre-processing (manifold)" << endl;
  of << "\tresolution: " << resolution << endl;
  cout << " - Pre-processing (manifold)" << endl;
  cout << "\tresolution: " << resolution << endl;

  start = clock();
  obj.Process_Manifold(resolution);
  end = clock();
  of << "Manifold time: " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;
  cout << "Manifold time: " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;

  output.Load(obj.vertices, obj.face_indices);

  return 0;
}

int Manifold(Model &input, Model &output, int resolution)
{
  clock_t start, end;

  Model_OBJ obj{};
  for (auto v : input.points)
  {
    obj.vertices.push_back({v[0], v[1], v[2]});
  }
  for (auto f : input.triangles)
  {
    obj.face_indices.push_back({f[0], f[1], f[2]});
  }

  start = clock();
  obj.Process_Manifold(resolution);
  end = clock();

  output.Load(obj.vertices, obj.face_indices);
  return 0;
}