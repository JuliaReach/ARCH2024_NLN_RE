using CSV, DataFrames

struct ControlInput
    t::Float64
    uref::Vector{Float64}
    K::Matrix{Float64}
end

struct ControllerData
    t0::Float64
    δt0::Float64
    ψt0::Float64
    vt0::Float64
    st0::Vector{Float64}
    dat::Vector{ControlInput}
end

function parse_control(file::String)
    df = CSV.read(file, DataFrame)

    t₀, δ_t₀, ψ_t₀, v_t₀, sx_t₀, sy_t₀ = [parse(Float64, x) for x in names(df)[1:6]]

    L = size(df, 1)
    K = [Matrix(reshape([xi for xi in df[i, 4:end]], (5, 2))') for i in 1:L]

    time_points = df[:, 1]
    uref_1 = df[:, 2]
    uref_2 = df[:, 3]
    dat = [ControlInput(ti, [uref_1_i, uref_2_i], Ki) for (ti, uref_1_i, uref_2_i, Ki) in zip(time_points, uref_1, uref_2, K)]

    ControllerData(t₀, δ_t₀, ψ_t₀, v_t₀, [sx_t₀, sy_t₀], dat)
end
