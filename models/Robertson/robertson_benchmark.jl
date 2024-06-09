using BenchmarkTools, Plots, Plots.PlotMeasures, LaTeXStrings
import ReachabilityAnalysis
using BenchmarkTools: minimum, median

# SUITE = BenchmarkGroup()
model = "ROBE21"
cases = ["1", "2", "3"]
# SUITE[model] = BenchmarkGroup()

include("robertson.jl")
num_timesteps = Int[]
final_width = Float64[]
elapsed_time = Float64[]
validation = ones(Int, 3)  # nothing to verify

ReachabilityAnalysis.LazySets.deactivate_assertions()

# ----------------------------------------
#  Case 1: alpha = 0.4, beta = 1.0e2, gamma = 1.0e3
# ----------------------------------------
println("Case 1")
const param1 = [0.4, 1.0e2, 1.0e3]

prob = robertson()
alg = TMJets21a(abstol=1e-10, orderT=5, orderQ=1, maxsteps=3_500)

sol1 = solve(prob, T=Tf, alg=alg, params=param1);  # warm-up run

eltime1 = @elapsed sol1 = solve(prob, T=Tf, alg=alg, params=param1);

# For later use
dom1 = @. getindex(getfield(sol1,:Δt),1);
# Compute the sum by summing the Taylor models, and then evaluating
sum1 = @. evaluate(evaluate(sum(getfield(sol1,:X)),
    domain(getindex(getfield(sol1,:X),1))), (normalized_box,))
# Evaluate the Taylor models first, and then sum; yields a much wider result!
# sum1 = [sum(evaluate.(evaluate.(getfield(sol1[ind], :X),
#     domain(getindex(getfield(sol1[ind],:X),1))),
#     (normalized_box,))) for ind in eachindex(sol1)];

# Store results
push!(num_timesteps, length(sol1)-1)
push!(final_width, diam(sum1[end]))
push!(elapsed_time, eltime1)
println("Time steps   $(cases[1]) : $(num_timesteps[1])")
println("Final width  $(cases[1]) : $(final_width[1])")
println("Elapsed time $(cases[1]) : $(elapsed_time[1])")

# # benchmark
# SUITE[model][cases[1]] = @benchmarkable solve($prob, T=$Tf, alg=$alg, params=$param1)


# ----------------------------------------
#  Case 2: alpha = 0.4, beta = 1.0e3, gamma = 1.0e5
# ----------------------------------------
println("Case 2")
const param2 = [0.4, 1.0e3, 1.0e5]

# prob = robertson()
alg = TMJets21a(abstol=1e-9, orderT=6, orderQ=1, maxsteps=32_000)

eltime2 = @elapsed sol2 = solve(prob, T=Tf, alg=alg, params=param2);

# For later use
dom2 = @. getindex(getfield(sol2,:Δt),1);
# Compute the sum by summing the Taylor models, and then evaluating
sum2 = @. evaluate(evaluate(sum(getfield(sol2,:X)),
    domain(getindex(getfield(sol2,:X),1))), (normalized_box,))
# Evaluate the Taylor models first, and then sum; yields a much wider result!
# sum2 = [sum(evaluate.(evaluate.(getfield(sol2[ind], :X),
#     domain(getindex(getfield(sol2[ind],:X),1))),
#     (normalized_box,))) for ind in eachindex(sol2)];

# Store results
push!(num_timesteps, length(sol2)-1)
push!(final_width, diam(sum2[end]))
push!(elapsed_time, eltime2)
println("Time steps   $(cases[2]) : $(num_timesteps[2])")
println("Final width  $(cases[2]) : $(final_width[2])")
println("Elapsed time $(cases[2]) : $(elapsed_time[2])")

# # benchmark
# SUITE[model][cases[2]] = @benchmarkable solve($prob, T=$Tf, alg=$alg, params=$param2)


# ----------------------------------------
#  Case 3: alpha = 0.4, beta = 1.0e3, gamma = 1.0e7
# ----------------------------------------
println("Case 3")
const param3 = [0.4, 1.0e3, 1.0e7]

# prob = robertson()
alg = TMJets21a(abstol=1e-10, orderT=10, orderQ=1, maxsteps=75000)

eltime3 = @elapsed sol3 = solve(prob, T=Tf, alg=alg, params=param3);

# For later use
dom3 = @. getindex(getfield(sol3,:Δt),1);
# Compute the sum by summing the Taylor models, and then evaluating
sum3 = @. evaluate(evaluate(sum(getfield(sol3,:X)),
    domain(getindex(getfield(sol3,:X),1))), (normalized_box,))
# Evaluate the Taylor models first, and then sum; yields a much wider result!
# sum3 = [sum(evaluate.(evaluate.(getfield(sol3[ind], :X),
#     domain(getindex(getfield(sol3[ind],:X),1))),
#     (normalized_box,))) for ind in eachindex(sol3)];

# Store results
push!(num_timesteps, length(sol3)-1)
push!(final_width, diam(sum3[end]))
push!(elapsed_time, eltime3)
println("Time steps   $(cases[3]) : $(num_timesteps[3])")
println("Final width  $(cases[3]) : $(final_width[3])")
println("Elapsed time $(cases[3]) : $(elapsed_time[3])")

# # benchmark
# SUITE[model][cases[3]] = @benchmarkable solve($prob, T=$Tf, alg=$alg, params=$param3)


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
# runtimes = Dict()
# for (i, c) in enumerate(cases)
#      t = median(results[model][c]).time * 1e-9
#      runtimes[c] = t
# end

if !@isdefined io
    io = stdout
end

for (i, c) in enumerate(cases)
    print(io, "$model,$c,$(validation[i]),$(elapsed_time[i])," *
              "$(final_width[i]),$(num_timesteps[i])\n")
end

# ==============================================================================
# Plot
# ==============================================================================

if !@isdefined TARGET_FOLDER
    TARGET_FOLDER = @__DIR__
end

fig = plot()
plot!(legend=:topright, legendfontsize=18,
      tickfont=font(30, "Times"), guidefontsize=45,
      xlab=L"t", # \raisebox{-0.5mm}{\textcolor{white}{.}}",
      ylab=L"s", # \raisebox{2mm}{\textcolor{white}{.}}",
      xtick=0:10:40, ytick=0.999:0.0002:1.001,
      xlims=(0., 42.), ylims=(0.999, 1.00105),
      # bottom_margin=6mm, left_margin=2mm, right_margin=4mm, top_margin=3mm,
      size=(1000, 1000))
plot!(fig, IntervalBox.(dom1,sum1),
      color=:blue,  linecolor=:match, alpha=0.8, label="Case 1")
plot!(fig, IntervalBox.(dom2,sum2),
      color=:green, linecolor=:match, alpha=0.8, label="Case 2")
plot!(fig, IntervalBox.(dom3,sum3),
      color=:red,   linecolor=:match, alpha=0.8, label="Case 3")

savefig(fig, joinpath(TARGET_FOLDER, "ARCH-COMP24-JuliaReach-$model.png"))
