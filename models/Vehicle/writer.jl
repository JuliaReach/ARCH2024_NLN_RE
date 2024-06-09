using CSV, DataFrames

# write occupancy set as CSV file and run Python script; each reach set is
#   approximated with a polygon in V-rep and stored together with its time
#   interval, as follows:
# [t_start_1, p_1_1^1, p_2_1^1, p_3_1^1]
# [  t_end_1, p_1_2^1, p_2_2^1, p_3_2^1]
# [t_start_2, p_1_1^2, p_2_1^2]
# [  t_end_2, p_1_2^2, p_2_2^2]
# ...
function write(file::String, sol)
    Vs = [convert(VPolygon, project(set(overapproximate(R, Zonotope)), 4:5)) for R in sol]
    m = length(sol)
    n = maximum(length(vertices_list(V)) for V in Vs)
    M = Matrix(undef, 2*m, n+1)
    i = 1
    @inbounds for k in 1:m
        V = Vs[k]
        vs = vertices_list(V)

        M[i, 1] = tstart(sol[k])
        for (j, vj) in enumerate(vs)
            M[i, j+1] = vj[1]
        end
        M[i+1, 1] = tend(sol[k])
        for (j, vj) in enumerate(vs)
            M[i+1, j+1] = vj[2]
        end
        # fill empty matrix cells with `missing`
        for j in length(vs)+1:n
            M[i, j+1] = missing
            M[i+1, j+1] = missing
        end
        i += 2
    end
    df = DataFrame(M, :auto)
    CSV.write(file, df; writeheader=false)
end
