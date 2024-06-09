# =================================================================
# Lotka-Volterra with nonlinear guard crossing model
# See https://easychair.org/publications/paper/nrdD
# =================================================================

using ReachabilityAnalysis

const T_lv = 3.64
const r₀ = 0.161
const ε  = 0.012
const X0 = IntervalBox( 1.3 ± ε, 1 .. 1)
const SB2 = symmetric_box(2, Float64)


@taylorize function lotka_volterra!(du, u, p, t)
    x, y = u
    xy = x * y
    du[1] = 3.0 * (x - xy)
    du[2] = xy - y
    return du
end

circle(x,y) = (x-1)^2 + (y-1)^2

function conditionNotOutside_Up(x,y; r₀=r₀)
    r₀² = r₀^2
    if y > 1.1
        return !(circle(x,y) > r₀²)
    else
        return false
    end
end
function conditionNotOutside_Down(x,y; r₀=r₀)
    r₀² = r₀^2
    if y < 0.9
        return !(circle(x,y) > r₀²)
    else
        return false
    end
end

lotka_volterra(Xini) = @ivp(x'= lotka_volterra!(x), dim:2, x(0) ∈ Xini)

@inline function lv_property(solz, r₀)
    # Time, time domain, x- and y-interval of the (minced) solutions
    timeI = getindex.(getfield.(solz,:Δt),1);
    domTs = domain.(getindex.(getfield.(solz,:X),1));
    xs = evaluate.(evaluate.(getindex.(getfield.(solz, :X),1), domTs), (SB2,));
    ys = evaluate.(evaluate.(getindex.(getfield.(solz, :X),2), domTs), (SB2,));

    # Get the indexes of the sets that are not outside the circle, i.e.,
    # that intersect the nonlinear guard
    rng_Up   = findall(conditionNotOutside_Up.(xs, ys, r₀=r₀));
    rng_Down = findall(conditionNotOutside_Down.(xs, ys, r₀=r₀));

    # Compute time intervals where the boxes are not outside the non-linear guard,
    # and the area of the region not outside the circle
    area = 0.0
    hull1 = emptyinterval()
    hull2 = emptyinterval()
    ib1 = [IntervalBox(0..0, 2)]
    ib2 = [IntervalBox(0..0, 2)]
    if !isempty(rng_Up)
        hull1 = hull(timeI[rng_Up]...)
        area = sum(diam.(xs[rng_Up]) .* diam.(ys[rng_Up]))
        ib1 = IntervalBox.(xs[rng_Up[1]:end], ys[rng_Up[1]:end])
    end
    if !isempty(rng_Down)
        hull2 = hull(timeI[rng_Down]...)
        area += sum(diam.(xs[rng_Down]) .* diam.(ys[rng_Down]))
        ib2 = IntervalBox.(xs[rng_Down[1]:end], ys[rng_Down[1]:end])
    end

    return (hull1, hull2, area, ib1, ib2)
end

# function lotka_volterra(; nsplit=4,
#                           ε = 0.012,
#                           ε_ext=1e-4, # threshold for the outer approximation
#                           n_int=50,   # number of directions for the inner approximation
#                           r₀ = 0.161) # radius circle

#     # generate external / internal polytopic approximations of the guard
#     B = Ball2([1.0, 1.0], r₀) # "exact"
#     B_ext = overapproximate(B, ε_ext) # outer approximation
#     B_int = underapproximate(B, PolarDirections(n_int)) # inner approximation
#     B_int = tohrep(convert(VPolygon, B_int)) # cast to Hrep
#     B_intᶜ = complement(B_int)

#     # define modes
#     aut = GraphAutomaton(3)
#     outside = @system(x' = lotka_volterra!(x), dim: 2, x ∈ B_intᶜ)
#     inside = @system(x' = lotka_volterra!(x), dim: 2, x ∈ B_ext)
#     outside_unconstrained = @system(x' = lotka_volterra!(x), dim: 2, x ∈ Universe(2))

#     # define the transition graph
#     add_transition!(aut, 1, 2, 1)
#     add_transition!(aut, 2, 3, 2)
#     T_out_in = @map(x -> x, dim:2, x ∈ B_ext)
#     T_in_out = @map(x -> x, dim:2, x ∈ B_intᶜ)

#     # initial-value problem
#     H = HybridSystem(automaton=aut, modes=[outside, inside, outside_unconstrained],
#                                            resetmaps=[T_out_in, T_in_out])

#     # initial states with splitting
#     X0 = Hyperrectangle(low=[1.3-ε, 1.], high=[1.3+ε, 1.])
#     X0s = split(X0, [nsplit, 1])
#     X0st = [(X0s_i, 1) for X0s_i in X0s]

#     return InitialValueProblem(H, X0st)
# end

# @inline function lv_property(solz, ε_ext, r₀)

#     # Sets intersecting the nonlinear guard
#     B = Ball2([1.0, 1.0], r₀) # "exact"
#     B_ext = overapproximate(B, ε_ext) # outer approximation
#     intersecting_reachsets = []
#     for (i, Fi) in enumerate(solz)
#         for (j, Xj) in enumerate(Fi)
# 	        Xjp = convert(VPolygon, Xj)
#             !is_intersection_empty(Xjp, B_ext) && push!(intersecting_reachsets, (i, j))
#         end
#     end

#     # Compute time spent inside non-linear guard
#     times = [tspan(solz[ind[1]][ind[2]]) for ind in intersecting_reachsets]
#     tmin = minimum(tstart, times)
#     tmax = maximum(tend, times)
#     @show(tmin, tmax, tmax-tmin)

#     indxs = Int[]
#     for (i, e) in enumerate(tspan.(solz))
#         T_lv ∈ tspan(e) && push!(indxs, i)
#     end
#     chsolz = [solz[i](T_lv) |> convexify for i in indxs]
#     chlasth = ConvexHullArray([set(Ri) for Ri in chsolz]) |> box_approximation
#     return ReachabilityAnalysis.LazySets.area(chlasth), tmax-tmin
# end
