# Geometry Processing - Registration
This is my implementation of Registration assignment in [CSC419/CSC2520 Geometry Processing](https://github.com/alecjacobson/geometry-processing-csc2520/).

## Build
```
git submodule update --init --recursive
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make 
```

## Execution

Once built, you can execute the assignment from inside the `build/` using 

    ./registration [path to mesh1.obj] [path to mesh2.obj]


_**ICP using the point-to-point matching energy linearization is slow to converge.**_
![](images/max-point-to-point.gif)

_**ICP using the point-to-plane matching energy linearization is faster.**_
![](images/max-point-to-plane.gif)