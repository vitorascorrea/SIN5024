using JuMP
using Gurobi
using MathOptInterface

ϵ = 0.000001

# Formato dos arquivos de entrada:
# V K (num vertices, num cores)
# v_1
# v_2
# ...
# v_n (cor do vertice n)

function le_dados_entrada(nome_arq)
    linha_arq = []

    open(nome_arq) do arq
        linha_arq = readlines(arq)
    end

    num_vertices_cores = parse.(Int, split(linha_arq[1]))
    num_vertices = num_vertices_cores[1]
    num_cores = num_vertices_cores[2]
    cor_vertices = []

    for i = 2:length(linha_arq)
        append!(cor_vertices, parse(Int, linha_arq[i]))
    end

    return num_vertices, num_cores, cor_vertices
end

function solve_convex_recoloration(num_vertices, num_cores, cor_vertices)
    # criando um modelo "vazio" no gurobi
    modelo = Model(Gurobi.Optimizer)

    # --- adicionando as variáveis no modelo e a função objetivo ---
    x = []

    soma_trocas_vertice = AffExpr()
    for i = 1:num_vertices
        push!(x, [])
        for k = 1:num_cores
            push!(x[i], @variable(modelo, binary=true))
            add_to_expression!(soma_trocas_vertice, x[i][k], cor_vertices[i] != k)
        end
    end
    @objective(modelo, Min, soma_trocas_vertice)
    # ----------------------------------------------

    # --- adicionando a restrição de só poder ter uma cor por vértice ---
    for i = 1:num_vertices
        soma_cores_por_vertice = AffExpr()
        for k = 1:num_cores
            add_to_expression!(soma_cores_por_vertice, x[i][k])
        end
        @constraint(modelo, soma_cores_por_vertice == 1)
    end
    # ----------------------------------------------

    # --- adicionando a restrição de não poder ter uma cor diferente em um vértice no meio de dois outros com cores iguais. ---
    for k = 1:num_cores
        for p = 1:(num_vertices - 2)
            for q = (p + 1):(num_vertices - 1)
                for r = (q + 1):(num_vertices)
                    soma_cores_entre_vertices = AffExpr()
                    somatoria = x[p][k] - x[q][k] + x[r][k]
                    add_to_expression!(soma_cores_entre_vertices, somatoria)
                    @constraint(modelo, soma_cores_entre_vertices <= 1)
                end
            end
        end
    end
    # ----------------------------------------------

    # pede para o solver resolver o modelo
    optimize!(modelo)

    # se encontrou solução ótima, imprime solução
    if termination_status(modelo) == MathOptInterface.OPTIMAL
        imprime_solucao(x, num_vertices, num_cores, cor_vertices)
    else
        println()
        println_in_yellow(string("Erro: Solver não encontrou solução ótima. Status = ", termination_status(modelo)))
    end
end

function imprime_solucao(x, num_vertices, num_cores, cor_vertices)
    println()
    num_trocas = 0
    for i = 1:num_vertices
        cor_original = cor_vertices[i]
        for j = 1:num_cores
            print_in_yellow(string(value(x[i][j]), " "))
            if value(x[i][j]) == 0.0 && cor_original == j
                num_trocas += 1
            end
        end
        println()
    end

    print_in_yellow(string("Número de trocas: ", num_trocas))
end

function print_in_yellow(texto)
    print("\e[1m\e[38;2;255;225;0;249m", texto)
end

function println_in_yellow(texto)
    println("\e[1m\e[38;2;255;255;0;249m", texto)
end

function executa_teste()
    # se der "LoadError: SystemError: opening file", altere o valor desta variável
    # PARA USAR CAMINHO RELATIVO: Para que o diretório raiz de execução do REPL do Julia
    # seja o seu projeto, clique em "File->Add Project Folder..." e selecione a pasta
    # "binary_knapsack". Execute a função "pwd()" no REPL do Julia para saber
    # qual é o diretório raiz de execução.
    arq_instancia = joinpath(@__DIR__, "../instancias/rand_10_10.txt")
    dados_entrada = le_dados_entrada(arq_instancia)
    solve_convex_recoloration(dados_entrada[1], dados_entrada[2], dados_entrada[3])
end

executa_teste()
