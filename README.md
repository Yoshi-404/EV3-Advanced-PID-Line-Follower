# EV3 Advanced PID Line Follower

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-LEGO%20Mindstorms%20EV3-red.svg)](https://www.lego.com/mindstorms)
[![Firmware](https://img.shields.io/badge/Firmware-Pybricks%20MicroPython-blueviolet.svg)](https://pybricks.com/)
[![Python](https://img.shields.io/badge/Python-3.8%2B-brightgreen.svg)](https://www.python.org/)

Sistema de navegação autônoma de alta performance para robôs seguidores de linha utilizando o bloco LEGO Mindstorms EV3 e Pybricks (MicroPython). O projeto conta com controle PID no domínio do tempo real ($dt$), frenagem adaptativa em curvas, calibração óptica assistida, proteção anti-windup e um **simulador cinemático em Python com autoajuste de ganhos (Twiddle)**.

---

<p align="center">
  <img src="assets/robot_assembly.jpg" alt="Montagem do Robô Seguidor de Linha EV3" width="780">
  <br>
  <em>Montagem física do robô seguidor de linha LEGO Mindstorms EV3 em pista de testes.</em>
</p>

---

## Demonstração e Visualização

<p align="center">
  <img src="assets/simulation.gif" alt="Animação da Simulação Cinemática 2D" width="700">
  <br>
  <em>Simulação cinemática 2D com telemetria instantânea de erro, turn e velocidade base.</em>
</p>

### Vídeo de Demonstração
> [!NOTE]
> Para assistir ao robô navegando na pista física em velocidade máxima com frenagem adaptativa em curvas, confira o vídeo de demonstração:
> - **Vídeo do Robô em Pista:** [Assistir no YouTube / Demonstração](https://youtube.com) *(substitua com o link do seu vídeo gravado)*

---

## Funcionalidades

* **Controle PID no Domínio do Tempo:** Cálculo do termo derivativo utilizando a variação real de tempo ($dt$) via `StopWatch`, eliminando instabilidades decorrentes de oscilações na taxa de amostragem.
* **Velocidade Dinâmica (Frenagem Adaptativa):** Redução automática e proporcional da velocidade linear base ($T_p$) em curvas acentuadas, com retomada de aceleração em retas.
* **Calibração Interativa na Tela:** Procedimento guiado no display LCD do EV3 para medição das leituras de superfície branca e preta, calculando o *setpoint* (offset) de transição ideal.
* **Proteção Anti-Windup:** Limite estrito de saturação do termo integral ($\pm 50$) para prevenir *overshoot* e oscilações severas após curvas prolongadas ou perda de linha.
* **Simulador Python Integrado:** Permite modelar, testar e analisar a resposta do robô sem depender do hardware físico.
* **Autoajuste de Ganhos (Twiddle):** Algoritmo de otimização automática (*Coordinate Ascent*) para convergência autônoma dos melhores ganhos $K_p$, $K_i$ e $K_d$.

---

## Hardware e Mapeamento de Portas

<p align="center">
  <img src="assets/robot_assembly.jpg" alt="Hardware EV3" width="520">
</p>

| Componente | Quantidade | Descrição / Função |
| :--- | :---: | :--- |
| **Bloco EV3** | 1x | Unidade inteligente de processamento com Pybricks MicroPython |
| **Motores Grandes EV3** | 2x | Tração diferencial direta com rodas de borracha de 56 mm |
| **Sensor de Cor EV3** | 1x | Leitura óptica de reflexão voltado ao solo (distância ~65 mm do eixo) |
| **Cabos de Conexão** | 3x | Cabos padrão RJ12 EV3 |

**Conexão de Portas:**
* **Porta A:** Motor Direito
* **Porta D:** Motor Esquerdo
* **Porta S1:** Sensor de Cor (orientado perpendicularmente ao piso)

---

## Como Rodar

### 1. No Robô EV3 Físico

1. Certifique-se de que o cartão MicroSD com [Pybricks MicroPython](https://pybricks.com/) está inserido no bloco EV3.
2. Conecte os motores e o sensor conforme o mapeamento de portas acima.
3. Abra este repositório no VS Code com a extensão oficial do Pybricks EV3 instalada.
4. Conecte o bloco via Bluetooth ou cabo USB e transfira o arquivo `main.py`.
5. Execute o script e siga a rotina de calibração interativa exibida na tela do EV3:
   * Posicione o sensor sobre a superfície **BRANCA** e aperte qualquer botão.
   * Posicione o sensor sobre a linha **PRETA** e aperte qualquer botão.
6. O robô emitirá um sinal sonoro duplo e iniciará a navegação autônoma imediatamente.

### 2. No Simulador Python (Sem Robô Físico)

O simulador em `simulator/pid_simulator.py` utiliza Python 3 (`numpy`, `matplotlib` e `Pillow`):

```bash
# Instalar dependências para o simulador (se necessário)
pip install numpy matplotlib pillow

# Executar a simulação padrão e gerar os gráficos em assets/
python3 simulator/pid_simulator.py --run

# Exibir a tabela comparativa de sintonia com métricas (ISE, Chatter)
python3 simulator/pid_simulator.py --tune-table

# Executar o algoritmo Twiddle para autoajuste dos ganhos PID
python3 simulator/pid_simulator.py --autotune
```

---

## Gráfico de Erro e Saída do PID

A execução do simulador gera uma análise gráfica detalhada em `assets/pid_graph.png`:

<p align="center">
  <img src="assets/pid_graph.png" alt="Gráficos de Telemetria e Desempenho do PID" width="900">
</p>

### Análise dos Painéis:
1. **Dinâmica da Linha (Painel Superior):** Mostra a reflexão instantânea do sensor em relação ao ponto de transição (*offset* = 30%) e a evolução do erro de rastreamento $e(t)$.
2. **Resposta Interna do PID (Painel Central):** Apresenta a decomposição dos termos Proporcional ($K_p \cdot e$), Integral com anti-windup ($K_i \cdot \int e\,dt$) e Derivativo ($K_d \cdot \frac{de}{dt}$), demonstrando a síntese do comando `turn`.
3. **Atuação nos Motores (Painel Inferior):** Ilustra a frenagem adaptativa — quando o erro absoluto $|e(t)|$ se eleva em curvas, a velocidade base $T_p$ decresce até o limite seguro, evitando destracionamento.

---

## Tabela de Ajuste Experimental (PID Tuning Table)

Resultados coletados em pista de teste (linha preta de 19 mm sobre base branca, offset 30%):

| # | $K_p$ | $K_i$ | $K_d$ | Velocidade Base ($T_p$) | ISE | Trepidação (Chatter) | Comportamento Observado / Diagnóstico | Classificação |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :--- | :---: |
| **1** | 5.0 | 0.0 | 0.0 | 200 | 3870.9 | Baixa (24.4) | Correção excessivamente lenta; perde a linha em curvas médias. | Sub-amortecido |
| **2** | 25.0 | 0.0 | 0.0 | 250 | 470.0 | Muito Alta (6884.6) | Oscilação contínua (*hunting*) em retas; derrapagem mecânica. | Instável |
| **3** | 12.0 | 0.0 | 30.0 | 250 | 1199.0 | Média (1386.1) | Estável em retas, mas oscila ao sair de curvas de raio reduzido. | $K_d$ Insuficiente |
| **4** | 12.0 | 0.0 | 180.0 | 250 | 1179.2 | Severa (27099.4) | Ruído derivativo amplificado; vibração perceptível nos motores. | $K_d$ Excessivo |
| **5** | 12.0 | 5.0 | 116.0 | 250 | 473.3 | Alta (8738.8) | Acúmulo integral excessivo (*windup*); forte overshoot em retas pós-curva. | $K_i$ Excessivo |
| **6** | **12.0** | **1.0** | **116.0** | **300** | **1045.4** | **Suave (6151.3)** | **Transição suave, resposta rápida a perturbações e frenagem dinâmica ativa.** | **Configuração Ótima** |

### Metodologia de Ajuste Recomendada:
1. Zere $K_i$ e $K_d$. Eleve $K_p$ até que o robô consiga seguir retas e curvas suaves, iniciando leve oscilação.
2. Introduza o termo derivativo $K_d$ para amortecer a oscilação e antecipar curvas fechadas.
3. Adicione uma parcela moderada de $K_i$ para eliminar o erro em regime permanente em curvas longas, mantendo o limite de anti-windup ativo.
4. Ajuste `self.fator_frenagem` e `self.tp_maximo` para compatibilizar a aderência das rodas à velocidade linear.

---

## Modo de Autoajuste (Twiddle)

O simulador inclui o algoritmo de busca por coordenadas (*Twiddle / Coordinate Ascent*). Ao executar:

```bash
python3 simulator/pid_simulator.py --autotune
```

O otimizador testa iterações em malha fechada sobre a pista sintética, minimizando o custo composto (Integral do Erro Quadrático somada à penalidade por trepidação). Os parâmetros resultantes podem ser aplicados diretamente em `main.py`.

---

## Estrutura do Repositório

```text
EV3-Advanced-PID-Line-Follower/
├── .gitignore               # Arquivos ignorados (Python, Pybricks, IDEs, SO)
├── LICENSE                  # Licença MIT
├── README.md                # Documentação em Português
├── README_EN.md             # Documentação em Inglês
├── main.py                  # Firmware principal em MicroPython para o EV3
├── assets/
│   ├── pid_graph.png        # Gráficos técnicos de desempenho e telemetria
│   ├── robot_assembly.jpg   # Fotografia detalhada da montagem física
│   └── simulation.gif       # Animação gráfica da simulação cinemática 2D
└── simulator/
    └── pid_simulator.py     # Simulador em Python com modelo cinemático e Twiddle
```

---

## Licença

Este projeto está licenciado sob os termos da licença [MIT](LICENSE).