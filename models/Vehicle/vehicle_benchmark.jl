using Plots, Plots.PlotMeasures, LaTeXStrings
using ReachabilityAnalysis
using ReachabilityAnalysis.ReachabilityBase.Timing: print_timed

model = "TRAF22"
case = ""

ReachabilityAnalysis.LazySets.deactivate_assertions()

if !@isdefined TARGET_FOLDER
    TARGET_FOLDER = @__DIR__
end

include("vehicle.jl")

# problem definition

ivp = vehicle()
T = ctrl.dat[end].t

# ==============================================================================
# Execute benchmarks and save benchmark results
# ==============================================================================

# reachability

alg = TMJets(abstol=3e-11, orderT=5, orderQ=1)
solve(ivp, T=T, alg=alg)  # warm-up run
res = @timed solve(ivp, T=T, alg=alg)
sol = res.value
runtime = res.time
print_timed(res)

# specification

## property 1: u_fb := u_ref + K(x + x_err - x_ref) ∈ U (at all times)

function property1()
    U = Hyperrectangle(zeros(2), U_radius)
    V = Hyperrectangle(zeros(5), V_radius)
    t0 = ctrl.t0
    for di in ctrl.dat
        t1 = di.t
        if t1 > tend(sol)
            @warn "flowpipe stops at $(tend(sol)), which is before the final time horizon"
            break
        end

        u_ref = Singleton(di.uref)
        K = di.K
        Ufb_const = minkowski_sum(u_ref, linear_map(K, V))

        Δt = t0 .. t1
        Rs = sol(Δt)
        for R in Rs
            X = set(overapproximate(R, Zonotope; Δt=intersect(Δt, tspan(R))))
            x = project(X, 1:5)
            x_ref = project(X, 6:10)

            Ufb = Ufb_const + K * (x + reflect(x_ref))

            if !(Ufb ⊆ U)
                return false
            end
        end

        t0 = t1
    end
    return true
end

property1()  # warm-up run
res = @timed property1()
validation = Int(res.value)
runtime += res.time
print_timed(res)

## property 2: write occupancy set

include("writer.jl")

println("writing output file ... ")
write(joinpath(TARGET_FOLDER, "BEL_Putte-4_2_T-1_occupancies.csv"), sol)

# benchmark table

if !@isdefined io
    io = stdout
end

print(io, "$model,$case,$validation,$runtime\n")

# ==============================================================================
# Plot
# ==============================================================================

fig = plot(sol, vars=(4, 5), xlab=L"s_x", ylab=L"s_y",
           lw=0.0, alpha=1, color=:blue,
           tickfont=font(20, "Times"), guidefontsize=35,
           bottom_margin=-3mm, left_margin=0mm, right_margin=3mm, top_margin=0mm,
           size=(1000, 1000))

savefig(fig, joinpath(TARGET_FOLDER, "ARCH-COMP24-JuliaReach-$model.png"))
