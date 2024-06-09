# =================================================================
# Coupled Van der Pol model
# See https://easychair.org/publications/paper/nrdD
# =================================================================

using ReachabilityAnalysis

const ey₁ = [0.0, 1.0, 0.0, 0.0, 0.0]
const ey₂ = [0.0, 0.0, 0.0, 1.0, 0.0]

@taylorize function vdp_N2_mu1_b!(dx, x, p, t)
    local μ = 1.0
    x₁, y₁, x₂, y₂, b = x

    aux0 = x₂ - x₁
    aux00 = b * aux0
    #
    aux11 = 1 - x₁^2
    aux12 = μ * y₁
    aux13 = aux11 * aux12
    aux15 = aux13 + aux00
    #
    aux21 = 1 - x₂^2
    aux22 = μ * y₂
    aux23 = aux21 * aux22
    aux25 = aux23 - aux00

    dx[1] = y₁
    dx[2] = aux15 - x₁ # (1 - x₁^2)*(μ * y₁) + b * (x₂ - x₁) - x₁
    dx[3] = y₂
    dx[4] = aux25 - x₂ # (1 - x₂^2) * (μ * y₂) - b * (x₂ - x₁) - x₂
    dx[5] = zero(b)

    return dx
end

function vanderpolN2_b(; b=interval(60, 80))
    X0 = (1.25 .. 1.55) × (2.35 .. 2.45) × (1.25 .. 1.55) × (2.35 .. 2.45) × b
    @ivp(x' = vdp_N2_mu1_b!(x), dim: 5, x(0) ∈ X0)
end
