using BenchmarkTools, Plots, Plots.PlotMeasures, LaTeXStrings
import ReachabilityAnalysis
using BenchmarkTools: minimum, median

SUITE = BenchmarkGroup()
model = "LOVO21"
cases = [""]
SUITE[model] = BenchmarkGroup()

include("lotka_volterra.jl")
validation = ones(Int, 1)  # nothing to verify
final_area = []
# intersect_time = []

ReachabilityAnalysis.LazySets.deactivate_assertions()

# ----------------------------------------
#  Case 1:
# ----------------------------------------
const num_divs = 5
const X0minced = unique(mince(X0, num_divs))

# warm-up run
lotka_volterra(X0) = @ivp(x'= lotka_volterra!(x), dim:2, x(0) ∈ X0)
prob = lotka_volterra(X0)
alg = TMJets(abstol=1e-4, orderT=3, orderQ=1);
# alg = TMJets21b(abstol=1e-23, orderT=18, orderQ=1)
sol = solve(prob, T=T_lv, alg=alg);

prob_minced = @. lotka_volterra(X0minced);
telapsed = @elapsed sols_minced = @. solve(prob_minced, T=T_lv, alg=alg);
@show(telapsed)

# Results: get times not spent strictly outside and final areas
res = lv_property.(sols_minced, r₀)
# time1 = diam(hull(getindex.(res, 1)...))
# time2 = diam(hull(getindex.(res, 2)...))
areas = sum(getindex.(res, 3))
ib1 = getindex.(res, 4)
ib2 = getindex.(res, 5)

push!(final_area, trunc(areas, sigdigits=3))
# push!(intersect_time, trunc(time1, sigdigits=3), trunc(time2, sigdigits=3))
println("Final area : $(areas)")
# println("Time spent in guard : $(time1), $(time2)")

# benchmark
# SUITE[model][cases[1]] = @benchmarkable :(@. solve($prob_minced, T=$T_lv, alg=$alg))
# SUITE[model][cases[1]] = @benchmarkable solve.($prob, T = $T_lv, alg = $alg)

# const ε_ext = 1e-4
# const r₀ = 0.161
# prob = lotka_volterra(; nsplit=7, ε_ext=ε_ext, r₀=r₀)
# alg = TMJets(abstol=1e-14, orderT=7, orderQ=1)

# # warm-up run
# sol_lv = solve(prob, T=T_lv,
#                   alg=alg,
#                   max_jumps=100,
#                   intersect_source_invariant=false,
#                   intersection_method=BoxIntersection(),
#                   clustering_method=BoxClustering(3),
#                   disjointness_method=BoxEnclosure());
# solz_lv = overapproximate(sol_lv, Zonotope);

# # obtain area
# area, time_in_guard = lv_property(solz_lv, ε_ext, r₀)
# push!(final_area, trunc(area, sigdigits=3))
# # push!(intersect_time, trunc(time_in_guard, sigdigits=3))
# println("Final area : $(area)")
# println("Time spent in guard : $(time_in_guard)")

# # benchmark
# SUITE[model][cases[1]] = @benchmarkable solve($prob,
#                   T = $T_lv,
#                   alg = $alg,
#                   max_jumps = 100,
#                   intersect_source_invariant = false,
#                   intersection_method = BoxIntersection(),
#                   clustering_method = BoxClustering(3),
#                   disjointness_method = BoxEnclosure())


# # ==============================================================================
# # Execute benchmarks and save benchmark results
# # ==============================================================================

# # tune parameters
# tune!(SUITE)

# # run the benchmarks
# results = run(SUITE, verbose=true)

# # return the sample with the smallest time value in each test
# println("minimum time for each benchmark:\n", minimum(results))

# # return the median for each test
# println("median time for each benchmark:\n", median(results))

# # export runtimes
runtimes = Dict()
for (i, c) in enumerate(cases)
    # t = median(results[model][c]).time * 1e-9
    runtimes[c] = telapsed
end

if !@isdefined io
    io = stdout
end

for (i, c) in enumerate(cases)
    print(io, "$model,$c,$(validation[i]),$(runtimes[c]),$(final_area[i])\n")
end

# ==============================================================================
# Create plots
# ==============================================================================

if !@isdefined TARGET_FOLDER
    TARGET_FOLDER = @__DIR__
end

const θ = range(0.0, 2*pi, length=1000)

fig = plot()
plot!(fig, xlims=(0.6, 1.45), ylims=(0.6, 1.4), aspect_ratio=:equal, legend=:none,
      palette = :seaborn_colorblind,
      tickfont=font(30, "Times"), guidefontsize=45,
      xtick=0.6:0.2:1.4, ytick=0.6:0.2:1.4,
      bottom_margin=6mm, left_margin=2mm, right_margin=8mm, top_margin=3mm,
      size=(1000, 1000))

# Flowpipes
for ind in 1:num_divs
    plot!(fig, sols_minced[ind], vars=(1,2), alpha=1.0, color=:blue, linecolor=:match)
end
# Flowpipes crossing guards
for ind in 1:num_divs
    plot!(fig, ib1[ind], alpha=0.9, color=:lightgreen, linecolor=:match)
    plot!(fig, ib2[ind], alpha=0.9, color=:orange, linecolor=:match)
end
plot!(fig, 1 .+ r₀.*cos.(θ), 1 .+ r₀.*sin.(θ), color=:red, label=:none)

xlabel!(fig, L"x")
ylabel!(fig, L"y")

# outside_idx = findall(x -> x == 1, location.(sol_lv))
# inside_idx = findall(x -> x == 2 || x == 3, location.(sol_lv))

# for i in outside_idx
#     plot!(fig, solz_lv[i], vars=(1, 2), lw=0.0, alpha=1.0, color=:blue)
# end

# for i in inside_idx
#     plot!(fig, solz_lv[i], vars=(1, 2), lw=0.0, alpha=1.0, color=:lightgreen)
# end

# B = Ball2([1.0, 1.0], 0.15) # "exact"
# B_ext = overapproximate(B, ε_ext) # outer approximation
# plot!(fig, B, 1e-4, color=:white, lw=2.0, linecolor=:red, tickfont=font(30, "Times"),
#         guidefontsize=45,
#         xlab=L"x",
#         ylab=L"y",
#         xtick=[0.8, 1.0, 1.2], ytick=[0.6, 0.8, 1.0, 1.2],
#         xlims=(0.6, 1.4), ylims=(0.6, 1.4),
#         bottom_margin=6mm, left_margin=2mm, right_margin=8mm, top_margin=3mm,
#         size=(1000, 1000))

savefig(fig, joinpath(TARGET_FOLDER, "ARCH-COMP24-JuliaReach-$model.png"))
