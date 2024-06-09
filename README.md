# ARCH2024 NLN

This is the JuliaReach repeatability evaluation (RE) package for the ARCH-COMP
2024 category report: Continuous and Hybrid Systems with Nonlinear Dynamics
of the 8th International Competition on Verifying Continuous and Hybrid Systems
(ARCH-COMP '24).

*Note:* Running the full benchmark suite should take no more than four hours
with a reasonable internet connection.

## Installation

There are two ways to install and run this RE: either using the Julia script or
using the Docker script.
In both cases, first clone this repository.


**Using the Julia script.**
First install the Julia compiler following the instructions
[here](http://julialang.org/downloads).
Once you have installed Julia, execute

```shell
$ julia startup.jl
```

to run all the benchmarks.


**Using the Docker container.**
To build the container, you need the program `docker`.
For installation instructions on different platforms, consult
[the Docker documentation](https://docs.docker.com/install/).
For general information about `Docker`, see
[this guide](https://docs.docker.com/get-started/).
Once you have installed Docker, start the `submit.sh` script:

```shell
$ ./submit.sh
```

---

The Docker container can also be run interactively:

```shell
$ docker run -it juliareach bash

$ julia

julia> include("startup.jl")
```

## Outputs

The results will be stored in the folder `result`.

---

## How the Julia environment was created

```julia
julia> ]

(@v1.10) pkg> activate .
  Activating new environment at `.../ARCH2024_NLN/Project.toml`

pkg> add BenchmarkTools
pkg> add CSV
pkg> add DataFrames
pkg> add LaTeXStrings
pkg> add Plots
pkg> add ReachabilityAnalysis
```
