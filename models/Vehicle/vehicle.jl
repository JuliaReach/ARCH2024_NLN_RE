# =================================================================
# Autonomous vehicle
# See https://easychair.org/publications/paper/cqFP
# =================================================================

using ReachabilityAnalysis
using ReachabilityAnalysis: Taylor1

include("parser.jl")
ctrl = parse_control(joinpath(@__DIR__, "BEL_Putte-4_2_T-1_controls.csv"))

const U_radius = [0.7, 11]
const V_radius = [0.0004, 0.0004, 0.006, 0.002, 0.002]
const lwb⁻¹ = 1.0 / 2.578

function u(i::Int, idx::Int)
    return ctrl.dat[idx].uref[i]
end

function K(i::Int, j::Int, idx::Int)
    return ctrl.dat[idx].K[i, j]
end

function find_ctrl_index(t::Real)
    @assert t ≥ ctrl.t0
    controls = ctrl.dat
    idx = 1
    while idx < length(controls) && t > controls[idx].t
        idx += 1
    end
    return idx
end

function parameters(t::Real)
    idx = find_ctrl_index(t)
    @inbounds begin
        u₁ = u(1, idx)
        u₂ = u(2, idx)
        K11 = K(1, 1, idx)
        K12 = K(1, 2, idx)
        K13 = K(1, 3, idx)
        K14 = K(1, 4, idx)
        K15 = K(1, 5, idx)
        K21 = K(2, 1, idx)
        K22 = K(2, 2, idx)
        K23 = K(2, 3, idx)
        K24 = K(2, 4, idx)
        K25 = K(2, 5, idx)
    end
    return (u₁, u₂, K11, K12, K13, K14, K15, K21, K22, K23, K24, K25)
end

function parameters(t1::Taylor1)
    t = evaluate(t1)  # convert Taylor1 to Float64
    return parameters(t)
end

@taylorize function vehicle!(dx, x, p, t)
    δ, ψ, v, sx, sy,
    δ_ref, ψ_ref, v_ref, sx_ref, sy_ref,
    δ_err, ψ_err, v_err, sx_err, sy_err, w₁, w₂ = x
    u₁_tmp, u₂_tmp, K11, K12, K13, K14, K15, K21, K22, K23, K24, K25 = parameters(t)
    u₁_ref = u₁_tmp * one(x[1])  # convert to right type
    u₂_ref = u₂_tmp * one(x[2])

    Δδ = δ + δ_err - δ_ref
    Δψ = ψ + ψ_err - ψ_ref
    Δv = v + v_err - v_ref
    Δsx = sx + sx_err - sx_ref
    Δsy = sy + sy_err - sy_ref

    u₁ = u₁_ref + (K11 * Δδ) + (K12 * Δψ) + (K13 * Δv) + (K14 * Δsx) + (K15 * Δsy)
    u₂ = u₂_ref + (K21 * Δδ) + (K22 * Δψ) + (K23 * Δv) + (K24 * Δsx) + (K25 * Δsy)

    dx[1] = u₁ + w₁                       # δ'
    dx[2] = (v * lwb⁻¹) * tan(δ)          # ψ'
    dx[3] = u₂ + w₂                       # v'
    dx[4] = v * cos(ψ)                   # sx'
    dx[5] = v * sin(ψ)                   # sy'
    dx[6] = u₁_ref                        # δ_ref'
    dx[7] = (v_ref * lwb⁻¹) * tan(δ_ref)  # ψ_ref'
    dx[8] = u₂_ref                        # v_ref'
    dx[9] = v_ref * cos(ψ_ref)           # sx_ref'
    dx[10] = v_ref * sin(ψ_ref)          # sy_ref'
    dx[11] = zero(x[11])                 # δ_err'
    dx[12] = zero(x[12])                 # ψ_err'
    dx[13] = zero(x[13])                 # v_err'
    dx[14] = zero(x[14])                 # sx_err'
    dx[15] = zero(x[15])                 # sy_err'
    dx[16] = zero(x[16])                 # w₁'
    dx[17] = zero(x[17])                 # w₂'

    return dx
end

function vehicle()
    x0 = [ctrl.δt0, ctrl.ψt0, ctrl.vt0, ctrl.st0[1], ctrl.st0[2]]
    W_radius = [0.02, 0.3]
    X0 = Hyperrectangle(vcat(      x0,       x0, zeros(5), zeros(2)),
                        vcat(V_radius, zeros(5), V_radius, W_radius))
    prob = @ivp(x' = vehicle!(x), dim:17, x(0) ∈ X0)
    return prob
end
