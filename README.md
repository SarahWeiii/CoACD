# Approximate Convex Decomposition for 3D Meshes with Collision-Aware Concavity and Tree Search [SIGGRAPH2022]
 [\[project\]](https://colin97.github.io/CoACD/) [\[paper\]](https://arxiv.org/pdf/2205.02961.pdf)

[***News***] We have replaced the original non-commercial dependencies. All of our code is under MIT license, and all dependencies allow commercial use now!

Approximate convex decomposition enables efficient geometry processing algorithms specifically designed for convex shapes (e.g., collision detection). We propose a method that is better to preserve collision conditions of the input shape with fewer components. It thus supports delicate and efficient object interaction in downstream applications.

![avatar](examples/teaser.png)

## Compile from source

### (1) Clone the code

```
git clone --recurse-submodules https://github.com/SarahWeiii/CoACD.git
```

### (2) Install dependencies

```
sudo apt update \
&& sudo apt install -y libtbb-dev libboost-all-dev libopenvdb-dev libspdlog-dev
```
Note: If you are using Ubuntu 18.04, you may need to install [spdlog](https://github.com/gabime/spdlog) from source (```make && make install```). 

### (3) Compile

```
cd CoACD \
&& mkdir build \
&& cd build \
&& cmake .. -DCMAKE_BUILD_TYPE=Release \
&& make
```

### (4) Quick start
We provide a set of default parameters, and you only need to specify the input and output path. You can take an arbitrary mesh as input (in `.obj` format, no need to be a manifold) and run the algorithm by the following command:
```
./main -i PATH_OF_YOUR_MESH -o PATH_OF_OUTPUT
```

The generated convex components (in both `.obj` and `.wrl` formats) will be saved in PATH_OF_OUTPUT.

## Examples

We provide some example meshes and a `run_example.sh`, and the results will be saved in the `outputs` folder.
```
bash run_example.sh
```
* You can adjust the threshold by `-t` to see results with different quality.
* Three of the examples are from [PartNet-M](https://sapien.ucsd.edu/browse) (Bottle.obj, Kettle.obj, KitchenPot.obj), which are non-manifold. Two of them are from [Thingi10K](https://ten-thousand-models.appspot.com/) (Octocat-v2.obj, SnowFlake.obj), which are both 2-manifold.

## Parameters

Here is the description of the parameters (sorted by importance).

* `-i/--input`: path for input mesh (`.obj`).
* `-o/--output`: path for output (`.obj` or `.wrl`).
* `-t/--threshold`:  concavity threshold for terminating the decomposition (0.01~1), default = 0.05.
* `-np/--no-prepocess`: flag to disable manifold preprocessing, default = false. If your input is already manifold mesh, disabling the preprocessing can avoid introducing extra artifacts.
* `-nm/--no-merge`: flag to disable merge postprocessing, default = false.
* `-c/--max-convex-hull`: max # convex hulls in the result, -1 for no maximum limitation, works **only when merge is enabled**, default = -1 (may introduce convex hull with a concavity larger than the threshold)
* `-mi/--mcts-iteration`: number of search iterations in MCTS (60~2000), default = 100.
* `-md/--mcts-depth`: max search depth in MCTS (2~7), default = 3.
* `-mn/--mcts-node`: max number of child nodes in MCTS (10~40), default = 20.
* `-pr/--prep-resolution`: resolution for manifold preprocess (20~100), default = 50.
* `-r/--resolution`: sampling resolution for Hausdorff distance calculation (1e3~1e4), default = 2000.
* `--pca`: flag to enable PCA pre-processing, default = false.
* `-k`: value of $k$ for $\operatorname{R_v}$ calculation, default = 0.3.
* `--seed`: random seed used for sampling, default = random().

An example of changing the parameters:
```
./main -i PATH_OF_YOUR_MESH -o PATH_OF_OUTPUT -t 0.05 -mi 200 -md 4 -mn 25
```

Parameter tuning *tricks*: 
1. In most cases, you only need to adjust the `threshold` (0.01~1) to balance the level of detail and the number of decomposed components. A higher value gives coarser results, and a lower value gives finer-grained results. You can refer to Fig. 14 in our paper for more details.
2. The default parameters are fast versions. If you care less about running time but more about the number of components, try to increase `searching depth (-md)`, `searching node (-mn)` and `searching iteration (-mi)` for better cutting strategies.
3. `-pr` controls the quality of manifold preprocessing. A larger value can make the preprocessed mesh closer to the original mesh but also lead to more triangles and longer runtime.
4. Make sure your input mesh is 2-manifold solid if you want to use the `-np` flag. Skipping manifold pre-processing can better preserve input details, but please don't specify the `-np` flag if your input mesh is non-manifold (the algorithm may crush or generate wrong results).
5. `--seed` is used for reproduction of the same results as our algorithm is stochastic.

## Use in SAPIEN

CoACD is also included in [SAPIEN](https://sapien.ucsd.edu/). You can install SAPIEN with pip:

```
pip install "sapien>=2.2.0"
```

### Example Usage

```
# print help info
coacd -h

# using default parameters
coacd INPUT_FILE OUTPUT_FILE
```

You can also use `CoACD` in python scripts. Find the example file by the following command:
```
which coacd
# Output the example path
```

## Citation

If you find our code helpful, please cite our paper:

```
@article{wei2022approximate,
  title={Approximate convex decomposition for 3d meshes with collision-aware concavity and tree search},
  author={Wei, Xinyue and Liu, Minghua and Ling, Zhan and Su, Hao},
  journal={ACM Transactions on Graphics (TOG)},
  volume={41},
  number={4},
  pages={1--18},
  year={2022},
  publisher={ACM New York, NY, USA}
}
```
