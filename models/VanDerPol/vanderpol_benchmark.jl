using BenchmarkTools, Plots, Plots.PlotMeasures, LaTeXStrings
import ReachabilityAnalysis
using BenchmarkTools: minimum, median

SUITE = BenchmarkGroup()
model = "CVDP23"
cases = [""]
SUITE[model] = BenchmarkGroup()

include("vanderpol.jl")
validation = Int[]

ReachabilityAnalysis.LazySets.deactivate_assertions()

# ----------------------------------------
# Case 1: μ = 1, b = [1, 3]
# ----------------------------------------

prob = vanderpolN2_b(b=interval(1.0, 3.0))
alg = TMJets(abstol=1e-4, orderT=4, orderQ=1)

# warm-up run
sol = solve(prob, T=7.0, alg=alg)

# verify that specification holds
# hyperrectangles are sufficient for the property, but zonotopes look better in the plot
solz = overapproximate(sol, Zonotope)
property = (ρ(ey₁, solz) < 2.75) && (ρ(ey₂, solz) < 2.75)
push!(validation, Int(property))

# benchmark
SUITE[model][cases[1]] = @benchmarkable solve($prob, T=7.0, alg=$alg)

# ==============================================================================
# Execute benchmarks and save benchmark results
# ==============================================================================

# tune parameters
tune!(SUITE)

# run the benchmarks
results = run(SUITE, verbose=true)

# return the sample with the smallest time value in each test
println("minimum time for each benchmark:\n", minimum(results))

# return the median for each test
println("median time for each benchmark:\n", median(results))

# export runtimes
runtimes = Dict()
for (i, c) in enumerate(cases)
    t = median(results[model][c]).time * 1e-9
    runtimes[c] = t
end

if !@isdefined io
    io = stdout
end

for (i, c) in enumerate(cases)
    print(io, "$model,$c,$(validation[i]),$(runtimes[c])\n")
end

# ==============================================================================
# Plot
# ==============================================================================

if !@isdefined TARGET_FOLDER
    TARGET_FOLDER = @__DIR__
end

fig = plot()
plot!(fig, solz,  vars=(1, 2), lw=0.0, alpha=1.0, color=:blue,
      tickfont=font(30, "Times"), guidefontsize=45,
      xlab=L"x_{1}",
      ylab=L"y_1",
      xtick=[-2.0, 0.0, 2.0], ytick=[-4.0, -2.0, 0.0, 2.0, 4.0],
      xlims=(-2.5, 2.5), ylims=(-3, 3),
      bottom_margin=0mm, left_margin=0mm, right_margin=0mm, top_margin=0mm,
      size=(1000, 1000))
hline!(fig, [2.75], lc=:red, ls=:dash, lw=2, lab="")

savefig(fig, joinpath(TARGET_FOLDER, "ARCH-COMP24-JuliaReach-$model.png"))
