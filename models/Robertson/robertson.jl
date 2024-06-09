# =================================================================
# Robertson chemical model
# =================================================================

using ReachabilityAnalysis

const Tf = 40.0
const normalized_box = symmetric_box(3, Float64)

@taylorize function robertson!(du, u , p, t)
    local α, β, γ = p
    x, y, z = u

    ax = α * x
    yz = y * z
    byz = β * yz
    aux1 = ax - byz
    y2 = y^2
    aux2 = γ * y2

    du[1] = -aux1
    du[2] =  aux1 - aux2
    du[3] =  aux2

    return nothing
end

function robertson()
    # initial state
    X0= (1 .. 1) × (0 .. 0) × (0 .. 0)

    return @ivp(x'= robertson!(x), dim:3, x(0) ∈ X0)
end
