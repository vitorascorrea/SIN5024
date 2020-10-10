using JuMP
using Gurobi
using MathOptInterface

ϵ = 0.000001

mutable struct Aresta
    u::Int
    v::Int
    peso::Int
end

function adiciona_aresta(arestas, lista_adj, u, v, peso)
    push!(lista_adj[u], v)
    push!(lista_adj[v], u)

    if u < v
        push!(arestas, Aresta(u, v, peso))
    else
        push!(arestas, Aresta(v, u, peso))
    end
end

# Formato dos arquivos de entrada:
# num_vertices
# num_arestas
# u_1 v_1 peso_1
# u_2 v_2 peso_2
# ...
# u_n v_n peso_n  ### ONDE n = num_arestas
function le_dados_entrada(nome_arq)
    linha_arq = []

    open(nome_arq) do arq
        linha_arq = readlines(arq)
    end

    num_vertices = parse(Int, linha_arq[1])
    num_arestas = parse(Int, linha_arq[2])
    arestas = []
    lista_adj = Array{Array{Int}}(undef, num_vertices)

    for u in 1:num_vertices
        lista_adj[u] = Int[]
    end

    for i in 1:num_arestas
        aresta = parse.(Int, split(linha_arq[i + 2]))
        u, v, peso = aresta[1], aresta[2], aresta[3]
        adiciona_aresta(arestas, lista_adj, u, v, peso)
    end

    return num_vertices, arestas, lista_adj

end

function resolve_max_weighted_matching(num_vertices, arestas, lista_adj)
    # criando um modelo "vazio" no gurobi
    modelo = Model(Gurobi.Optimizer)

    # --- adicionando as variáveis e função objetivo ao modelo ---
    x = Dict()
    soma_pesos_arestas = AffExpr()

    for a in arestas
        x[a.u, a.v] = @variable(modelo, binary=true)
        add_to_expression!(soma_pesos_arestas, a.peso, x[a.u, a.v])
    end

    @objective(modelo, Max, soma_pesos_arestas)
    # -------------------------------------------------------------

    # --- adicionando a restrição que proibe a escolha de arestas com pontas em comum ---
    for u in 1:num_vertices
        soma_leque = AffExpr()

        for v in lista_adj[u]
            if u < v
                add_to_expression!(soma_leque, 1, x[u, v])
            else
                add_to_expression!(soma_leque, 1, x[v, u])
            end
        end

        @constraint(modelo, soma_leque <= 1)
    end
    # -----------------------------------------------------------------------------------

    # pede para o solver resolver o modelo
    optimize!(modelo)

    # se encontrou solução ótima, imprime solução
    if termination_status(modelo) == MathOptInterface.OPTIMAL
        imprime_solucao(x, arestas)
    else
        println()
        println_in_yellow(string("Erro: Solver não encontrou solução ótima. Status = ", termination_status(modelo)))
    end
end

function imprime_solucao(x, arestas)
    println()
    print_in_yellow("Arestas do emparelhamento de peso máximo: ")

    for a in arestas
        if value(x[a.u, a.v]) >= 1 - ϵ
            print_in_yellow(string("(", a.u, ",", a.v, ",", a.peso, ") "))
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
    resolve_max_weighted_matching(dados_entrada[1], dados_entrada[2], dados_entrada[3])
end

executa_teste()
