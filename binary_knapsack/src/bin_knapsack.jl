using JuMP
using Gurobi
using MathOptInterface

ϵ = 0.000001

# Formato dos arquivos de entrada:
# capacidade da mochila
# n_itens
# v_1 v_2 ... v_n (valores dos itens)
# p_1 p_2 ... p_n (pesos dos itens)
function le_dados_entrada(nome_arq)
    linha_arq = []

    open(nome_arq) do arq
        linha_arq = readlines(arq)
    end

    capacidade_mochila = parse(Int, linha_arq[1])
    num_itens = parse(Int, linha_arq[2])
    valor_item = parse.(Int, split(linha_arq[3]))
    peso_item =  parse.(Int, split(linha_arq[4]))

    return capacidade_mochila, num_itens, valor_item, peso_item
end

function resolve_binary_knapsack(capacidade_mochila, num_itens, valor_item, peso_item)
    # criando um modelo "vazio" no gurobi
    modelo = Model(Gurobi.Optimizer)

    # --- adicionando as variáveis no modelo ---
    x = Dict()
    for i in 1:num_itens
        x[i] = @variable(modelo, binary=true)
    end
    # ------------------------------------------

    # --- adicionando a função objetivo ao modelo ---
    soma_valores_itens = AffExpr()
    for i in 1:num_itens
        add_to_expression!(soma_valores_itens, valor_item[i] , x[i])
    end

    @objective(modelo, Max, soma_valores_itens)
    # ----------------------------------------------

    # --- adicionando a restrição de capacidade ao modelo ---
    soma_pesos_itens = AffExpr()
    for i in 1:num_itens
        add_to_expression!(soma_pesos_itens, peso_item[i], x[i])
    end
    @constraint(modelo, soma_pesos_itens <= capacidade_mochila)
    # ------------------------

    # pede para o solver resolver o modelo
    optimize!(modelo)

    # se encontrou solução ótima, imprime solução
    if termination_status(modelo) == MathOptInterface.OPTIMAL
        imprime_solucao(x, num_itens, valor_item, peso_item)
    else
        println()
        println_in_yellow(string("Erro: Solver não encontrou solução ótima. Status = ", termination_status(modelo)))
    end
end

function imprime_solucao(x, num_itens, valor_item, peso_item)
    peso_total_itens_escolhidos=0
    valor_total_itens_escolhidos=0
    println()
    print_in_yellow("Itens escolhidos: ")

    for i in 1:num_itens
        if value(x[i]) >= 1 - ϵ
            peso_total_itens_escolhidos += peso_item[i]
            valor_total_itens_escolhidos += valor_item[i]
            print_in_yellow(string(i, " (valor: ", valor_item[i], ", peso: ", peso_item[i], ") "))
        end
    end

    println()
    println_in_yellow(string("Peso total dos itens escolhidos: ", peso_total_itens_escolhidos))
    println_in_yellow(string("Valor total dos itens escolhidos: ",valor_total_itens_escolhidos))
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
    arq_instancia = joinpath(@__DIR__, "../instancias/instancia1.txt")
    dados_entrada = le_dados_entrada(arq_instancia)
    resolve_binary_knapsack(dados_entrada[1], dados_entrada[2], dados_entrada[3], dados_entrada[4])
end

executa_teste()
