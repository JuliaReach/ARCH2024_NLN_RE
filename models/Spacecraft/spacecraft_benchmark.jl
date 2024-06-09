using BenchmarkTools, Plots, Plots.PlotMeasures, LaTeXStrings
import ReachabilityAnalysis as RA
using BenchmarkTools: minimum, median

SUITE = BenchmarkGroup()
model = "SPRE22"
cases = [""]
SUITE[model] = BenchmarkGroup()

include("spacecraft.jl")
validation = Int[]

ReachabilityAnalysis.LazySets.deactivate_assertions()

# internal (non-exported functions)
using ReachabilityAnalysis: post, BoxVecClustering
import ReachabilityAnalysis.IntervalArithmetic as IA

function _solve_spacecraft(prob)
    # solve first mode
    T = 200.0
    p1 = IVP(prob.s.modes[1], prob.x0[1][2])
    I1 = stateset(p1)
    I1c = complement(I1)
    alg = TMJets21a(maxsteps=3_000, abstol=4e-4, orderT=3, orderQ=1, disjointness=BoxEnclosure())
    @time sol1F = post(alg, p1, IA.Interval(0, T))
    sol1F.ext[:loc_id] = 1

    # find sets that take the jump 1 -> 2
    jump_rset_idx = findall(R -> !isdisjoint(R, I1c), sol1F)
    Xc_box = cluster(sol1F, jump_rset_idx, BoxVecClustering(100))
    if length(Xc_box) != 1
        error()
    end

    # apply jump according to 1 -> 2 transition
    ot1 = first(out_transitions(prob.s, 1))
    tr12 = DiscreteTransition(prob.s, ot1)
    z12 = apply(tr12, Xc_box[1], TemplateHullIntersection(BoxDirections(5)))

    # solve second mode
    inv2 = HalfSpace([-1.0, 0, 0, 0, 0], 100.0) #  ∩ HalfSpace([0, 0, 0, 0, 1.0], 150.0)
    z12box = overapproximate(z12, Hyperrectangle)
    p2 = IVP(ConstrainedBlackBoxContinuousSystem(prob.s.modes[2].f, 5, inv2), z12box)
    alg = TMJets21a(abstol=5e-6, orderT=5, orderQ=1, disjointness=BoxEnclosure())
    Δt0 = tspan(Xc_box[1])
    tprev = tstart(Δt0)
    @time sol2F = post(alg, p2, IA.Interval(0, 150 - tprev); Δt0=Δt0)
    sol2F.ext[:loc_id] = 2
    sol2F_filtered = sol2F(120 .. 150)

    # cluster and zonotope enclosure of the sets that take the jump, using time information in the TM
    ngroups = 5
    sol2F_groups = RA.nfolds(sol2F_filtered, ngroups)
    out2_groups = [RA._cluster_decompose_zono(ai, [1:2, 3:4, 5:5], PolarDirections(20)) for ai in sol2F_groups]
    TM0_3 = [RA._overapproximate_structured(o, TaylorModelReachSet, orderT=4, orderQ=1) for o in out2_groups]

    # solve third mode
    p3 = [IVP(prob.s.modes[3], X0newi) for X0newi in TM0_3]
    alg = TMJets(abstol=1e-10, orderT=7, orderQ=1, disjointness=BoxEnclosure())
    Δt0 = tspan(sol2F_filtered)
    tprev = tstart(Δt0)
    @time sol3F = [post(alg, p3i, IA.Interval(0, T - tprev); Δt0=Δt0) for p3i in p3]
    for Fi in sol3F
        Fi.ext[:loc_id] = 3
    end

    return HybridFlowpipe(reduce(vcat, (sol1F, sol2F, sol3F)))
end

prob = spacecraft()
sol = _solve_spacecraft(prob)
solz = overapproximate(sol, Zonotope)

# verify that specifications hold
prop1 = line_of_sight(solz)
println("Line of sight property: $prop1")

prop2 = velocity_constraint(solz)
println("Velocity constraint property $prop2")

prop3 = target_avoidance(solz)
println("Target avoidance property: $prop3")

property = prop1 && prop2 && prop3
push!(validation, Int(property))

# benchmark
SUITE[model][cases[1]] = @benchmarkable _solve_spacecraft($prob)

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
    local t
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

idx_approaching = findall(==(1,), location.(sol))
idx_attempt = findall(==(2,), location.(sol))
idx_aborting = findall(==(3,), location.(sol))

sol_inner = array(sol).u  # needed because RecursiveArrayTools is currently broken

fig = plot()
plot!(legend=:bottomright, legendfontsize=18,
      tickfont=font(30, "Times"), guidefontsize=45,
      xlab=L"x", ylab=L"y",
      xtick=-800:200:0, ytick=-400:100:0,
      xlims=(-900, 100), ylims=(-500, 10.0),
      size=(1000, 1000))
[plot!(fig, s, vars=(1, 2), c=:blue, lw=0.0) for s in sol_inner[idx_approaching]]
[plot!(fig, s, vars=(1, 2), c=:lightgreen, lw=0.0) for s in sol_inner[idx_aborting]]
[plot!(fig, s, vars=(1, 2), c=:red, lw=0.0) for s in sol_inner[idx_attempt]]

savefig(fig, joinpath(TARGET_FOLDER, "ARCH-COMP24-JuliaReach-$model.png"))
