using JuMP
using Gurobi # COMENTAR ESSA LINHA SE FOR USAR O CPLEX
#using CPLEX # COMENTAR ESSA LINHA SE FOR USAR O GUROBI
using MathOptInterface

ϵ = 0.000001

# u->v
function adiciona_arco(arcos, vizinhos_saida, vizinhos_entrada, u, v, custo)
    push!(vizinhos_saida[u], v)
    push!(vizinhos_entrada[v], u)
    arcos[u, v] = custo
end

# Formato dos arquivos de entrada:
# num_vertices
# num_arcos
# s
# t
# u_1 v_1 custo_1
# u_2 v_2 custo_2
# ...
# u_n v_n custo_n  ### ONDE n = num_arcos
function le_dados_entrada(nome_arq)
    linha_arq = []

    open(nome_arq) do arq
        linha_arq = readlines(arq)
    end

    num_vertices = parse(Int, linha_arq[1])
    num_arcos = parse(Int, linha_arq[2])
    s  = parse(Int, linha_arq[3])
    t  = parse(Int, linha_arq[4])
    arcos = Dict()
    vizinhos_entrada = Array{Array{Int}}(undef, num_vertices)
    vizinhos_saida = Array{Array{Int}}(undef, num_vertices)

    for u in 1:num_vertices
        vizinhos_entrada[u] = Int[]
        vizinhos_saida[u] = Int[]
    end

    for i in 1:num_arcos
        arco = parse.(Int, split(linha_arq[i + 4]))
        u, v, custo = arco[1], arco[2], arco[3]
        adiciona_arco(arcos, vizinhos_saida, vizinhos_entrada, u, v, custo)
    end

    return num_vertices, s, t, arcos, vizinhos_saida, vizinhos_entrada

end

function resolve_min_cost_st_path(num_vertices, s, t, arcos, vizinhos_saida, vizinhos_entrada)
    # criando um modelo "vazio"

    modelo = Model(Gurobi.Optimizer) # COMENTAR ESSA LINHA SE FOR USAR O CPLEX
    #modelo = Model(CPLEX.Optimizer) # COMENTAR ESSA LINHA SE FOR USAR O GUROBI

    # --- adicionando as variáveis e função objetivo ao modelo ---
    x = Dict()
    soma_custos_arcos = AffExpr()

    for ((u, v), custo) in arcos
        x[u, v] = @variable(modelo, binary=true)
        add_to_expression!(soma_custos_arcos, custo, x[u, v])
    end

    @objective(modelo, Min, soma_custos_arcos)
    # -------------------------------------------------------------

    # --- adicionando a restrição que proibe a escolha de arestas com pontas em comum ---
    for u in 1:num_vertices
        soma_vizinhos_entrada = AffExpr()
        soma_vizinhos_saida = AffExpr()

        for v in vizinhos_entrada[u]
            add_to_expression!(soma_vizinhos_entrada, 1, x[v, u])
        end

        for v in vizinhos_saida[u]
            add_to_expression!(soma_vizinhos_saida, 1, x[u, v])
        end

        if u == s
            @constraint(modelo, soma_vizinhos_entrada == 0)
            @constraint(modelo, soma_vizinhos_saida == 1)
        elseif u == t
            @constraint(modelo, soma_vizinhos_entrada == 1)
            @constraint(modelo, soma_vizinhos_saida == 0)
        else
            @constraint(modelo, soma_vizinhos_entrada == soma_vizinhos_saida)
        end
    end
    # -----------------------------------------------------------------------------------

    # pede para o solver resolver o modelo
    optimize!(modelo)

    # se encontrou solução ótima, imprime solução
    if termination_status(modelo) == MathOptInterface.OPTIMAL
        imprime_solucao(x, vizinhos_saida, arcos, s, t)
    else
        println()
        println_in_yellow(string("Erro: Solver não encontrou solução ótima. Status = ", termination_status(modelo)))
    end
end

function imprime_solucao(x, vizinhos_saida, arcos, s, t)
    println()
    print_in_yellow("Arcos do st-caminho mínimo: ")
    u = s

    while u != t
        for v in vizinhos_saida[u]
            if value(x[u, v]) >= 1 - ϵ
                print_in_yellow(string("(", u, ", ", v, ", ", arcos[u,v], ") "))
                u = v
                break
            end
        end
    end
end

function print_in_yellow(texto)
    print("\e[1m\e[38;2;255;225;0;249m", texto)
end

function println_in_yellow(texto)
    println("\e[1m\e[38;2;255;255;0;249m", texto)
end

function executa_teste()
    arq_instancia = joinpath(@__DIR__, "../instancias/instancia1.txt")
    dados_entrada = le_dados_entrada(arq_instancia)
    resolve_min_cost_st_path(dados_entrada[1], dados_entrada[2], dados_entrada[3], dados_entrada[4], dados_entrada[5], dados_entrada[6])
end

executa_teste()
