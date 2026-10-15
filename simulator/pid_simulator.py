#!/usr/bin/env python3
"""
Simulador Avançado de Controlador PID para LEGO Mindstorms EV3 (Pybricks MicroPython).
Simula a cinemática diferencial de robô seguidor de linha em pista de testes,
oferece modo de autoajuste (Twiddle) e exporta gráficos e animações para documentação.
"""

import argparse
import math
import os
import sys
from typing import Dict, List, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


class LineFollowerKinematics:
    """
    Simulador de cinemática diferencial para robô seguidor de linha EV3.
    Considera distância entre rodas (wheelbase), avanço do sensor de cor e
    conversão de velocidade angular dos motores (deg/s) para deslocamento linear (mm/s).
    """

    def __init__(
        self,
        wheelbase: float = 120.0,     # Distância entre rodas em mm
        sensor_dist: float = 65.0,    # Distância do eixo ao sensor de cor em mm
        wheel_diameter: float = 56.0, # Diâmetro da roda padrão EV3 em mm
        offset: float = 30.0,         # Reflexão no ponto de transição branco/preto
    ):
        self.wheelbase = wheelbase
        self.sensor_dist = sensor_dist
        self.deg_to_mms = (math.pi * wheel_diameter) / 360.0  # ~0.4886 mm/s por deg/s
        self.offset = offset
        self.reset()

    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def step(
        self, motor_dir: float, motor_esq: float, y_track: float, dt_s: float
    ) -> Tuple[float, float, float]:
        """
        Avança um passo na cinemática do robô.
        Retorna (light_value, lat_err, robot_y)
        """
        # Converte deg/s dos motores para mm/s
        v_r = motor_dir * self.deg_to_mms
        v_l = motor_esq * self.deg_to_mms

        # Velocidade linear e velocidade angular
        v = (v_r + v_l) / 2.0
        omega = (v_l - v_r) / self.wheelbase

        # Atualiza orientação e posição
        self.theta += omega * dt_s
        self.x += v * math.cos(self.theta) * dt_s
        self.y += v * math.sin(self.theta) * dt_s

        # Posição do sensor avançado
        sensor_y = self.y + self.sensor_dist * math.sin(self.theta)
        lat_err = sensor_y - y_track

        # Modelo óptico do sensor de reflexão:
        # offset (30%) na borda; varia entre 10% (preto) e 50% (branco) numa faixa de ~10mm
        light_value = self.offset + (lat_err * 2.0)
        light_value = float(np.clip(light_value, 10.0, 50.0))

        return light_value, lat_err, self.y


class PIDController:
    """
    Controlador PID idêntico ao implementado em main.py.
    """

    def __init__(
        self,
        kp: float = 12.0,
        ki: float = 1.0,
        kd: float = 116.0,
        offset: float = 30.0,
        tp_maximo: float = 300.0,
        fator_frenagem: float = 4.0,
        vel_minima: float = 100.0,
        anti_windup_limit: float = 50.0,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.offset = offset
        self.tp_maximo = tp_maximo
        self.fator_frenagem = fator_frenagem
        self.vel_minima = vel_minima
        self.anti_windup_limit = anti_windup_limit
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.lasterror = 0.0

    def compute(self, light_value: float, dt_ms: float) -> Tuple[float, float, float, float, float, float, float, float]:
        """
        Executa a iteração de controle exatamente como no Pybricks.
        dt_ms: tempo decorrido em milissegundos (retornado por StopWatch.time()).
        """
        error = light_value - self.offset

        # Termo Integral com Anti-Windup
        self.integral += error
        if self.integral > self.anti_windup_limit:
            self.integral = self.anti_windup_limit
        elif self.integral < -self.anti_windup_limit:
            self.integral = -self.anti_windup_limit

        # Termo Derivativo no domínio do tempo
        if dt_ms > 0:
            derivative = (error - self.lasterror) / dt_ms
        else:
            derivative = 0.0

        p_term = self.kp * error
        i_term = self.ki * self.integral
        d_term = self.kd * derivative
        turn = p_term + i_term + d_term

        # Velocidade dinâmica adaptativa com frenagem proporcional
        tp = self.tp_maximo - (abs(error) * self.fator_frenagem)
        if tp < self.vel_minima:
            tp = self.vel_minima

        motor_dir = tp - turn
        motor_esq = tp + turn

        self.lasterror = error
        return error, p_term, i_term, d_term, turn, tp, motor_dir, motor_esq


class TrackGenerator:
    """
    Gera o traçado de referência de uma pista mista:
    - Retas de aceleração
    - Curva contínua suave
    - Perturbação abrupta em degrau
    - Curva em S reversa
    """

    @staticmethod
    def create_track(total_s: float = 12.0, dt_s: float = 0.01) -> Tuple[np.ndarray, np.ndarray]:
        t = np.arange(0, total_s, dt_s)
        y_track = np.zeros_like(t)

        for i, ti in enumerate(t):
            if ti < 2.0:
                y_track[i] = 0.0
            elif ti < 5.0:
                # Curva para a direita
                y_track[i] = 22.0 * math.sin((ti - 2.0) * math.pi / 3.0)
            elif ti < 7.5:
                # Degrau abrupto simula desvio ou junção de placas
                y_track[i] = 18.0
            elif ti < 10.5:
                # Curva em S
                y_track[i] = 18.0 - 24.0 * math.sin((ti - 7.5) * math.pi / 3.0)
            else:
                y_track[i] = 0.0

        return t, y_track


def simulate(
    controller: PIDController,
    robot: LineFollowerKinematics,
    t: np.ndarray,
    y_track: np.ndarray,
    dt_ms: float = 10.0,
) -> Dict[str, np.ndarray]:
    controller.reset()
    robot.reset()
    dt_s = dt_ms / 1000.0
    n = len(t)

    hist = {
        "t": t,
        "y_track": y_track,
        "light_value": np.zeros(n),
        "error": np.zeros(n),
        "p_term": np.zeros(n),
        "i_term": np.zeros(n),
        "d_term": np.zeros(n),
        "turn": np.zeros(n),
        "tp": np.zeros(n),
        "motor_dir": np.zeros(n),
        "motor_esq": np.zeros(n),
        "robot_x": np.zeros(n),
        "robot_y": np.zeros(n),
        "theta": np.zeros(n),
    }

    # Inicialização: sensor começa no offset
    light_val = controller.offset
    m_dir, m_esq = controller.tp_maximo, controller.tp_maximo

    for i in range(n):
        err, p, it, d, turn, tp, m_dir, m_esq = controller.compute(light_val, dt_ms)
        light_val, lat_err, rob_y = robot.step(m_dir, m_esq, y_track[i], dt_s)

        hist["light_value"][i] = light_val
        hist["error"][i] = err
        hist["p_term"][i] = p
        hist["i_term"][i] = it
        hist["d_term"][i] = d
        hist["turn"][i] = turn
        hist["tp"][i] = tp
        hist["motor_dir"][i] = m_dir
        hist["motor_esq"][i] = m_esq
        hist["robot_x"][i] = robot.x
        hist["robot_y"][i] = robot.y
        hist["theta"][i] = robot.theta

    return hist


def evaluate_score(hist: Dict[str, np.ndarray], dt_s: float = 0.01) -> Tuple[float, float, float]:
    """
    Avalia a qualidade do controle:
    - ISE: Integral do Erro Quadrático
    - Chatter: Suavidade angular (penaliza trepidação de motores)
    - Custo Composto: ISE + penalização de trepidação
    """
    error = hist["error"]
    turn = hist["turn"]
    ise = float(np.sum(error ** 2) * dt_s)
    turn_diff = np.diff(turn)
    chatter = float(np.sum(turn_diff ** 2) * dt_s)
    cost = ise + (0.015 * chatter)
    return ise, chatter, cost


def autotune_twiddle(
    initial_params: Tuple[float, float, float] = (10.0, 0.5, 80.0),
    tolerance: float = 0.08,
    max_iterations: int = 50,
) -> Tuple[float, float, float, List[Dict[str, float]]]:
    """
    Algoritmo Twiddle (Coordinate Ascent) para autoajuste dos ganhos PID.
    """
    p = list(initial_params)
    dp = [p[0] * 0.3, max(0.2, p[1] * 0.5), p[2] * 0.3]

    t, y_track = TrackGenerator.create_track()
    ctrl = PIDController(kp=p[0], ki=p[1], kd=p[2])
    rob = LineFollowerKinematics()
    hist = simulate(ctrl, rob, t, y_track)
    _, _, best_cost = evaluate_score(hist)

    history = [{"iter": 0, "kp": p[0], "ki": p[1], "kd": p[2], "cost": best_cost}]
    it = 0

    while sum(dp) > tolerance and it < max_iterations:
        for i in range(len(p)):
            p[i] += dp[i]
            ctrl = PIDController(kp=p[0], ki=p[1], kd=p[2])
            rob = LineFollowerKinematics()
            hist = simulate(ctrl, rob, t, y_track)
            _, _, cost = evaluate_score(hist)

            if cost < best_cost:
                best_cost = cost
                dp[i] *= 1.15
            else:
                p[i] -= 2.0 * dp[i]
                if p[i] < 0:
                    p[i] = 0.0
                ctrl = PIDController(kp=p[0], ki=p[1], kd=p[2])
                rob = LineFollowerKinematics()
                hist = simulate(ctrl, rob, t, y_track)
                _, _, cost = evaluate_score(hist)

                if cost < best_cost:
                    best_cost = cost
                    dp[i] *= 1.08
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.85

            it += 1
            history.append({"iter": it, "kp": p[0], "ki": p[1], "kd": p[2], "cost": best_cost})

    return p[0], p[1], p[2], history


def plot_performance_graphs(hist: Dict[str, np.ndarray], output_file: str):
    """
    Gera gráfico com 3 painéis técnicos de alto padrão:
    1. Erro de Rastreamento e Leitura do Sensor
    2. Decomposição dos Termos PID (P, I, D) e Ação de Correção Turn
    3. Velocidade Dinâmica (Frenagem) e Potência dos Motores A e D
    """
    t = hist["t"]
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11.5, 9.5), sharex=True)
    plt.subplots_adjust(hspace=0.28)

    # 1. Erro e Sensor
    ax1.plot(t, hist["light_value"], color="#2c3e50", lw=2.0, label="Reflexão do Sensor (%)")
    ax1.axhline(30.0, color="#7f8c8d", linestyle="--", lw=1.5, label="Setpoint (Offset = 30%)")
    ax1.plot(t, hist["error"], color="#e74c3c", lw=2.0, label="Erro Instantâneo e(t)")
    ax1.axhline(0.0, color="#bdc3c7", linestyle=":", lw=1.0)
    ax1.set_title("1. Dinâmica da Linha: Resposta Óptica do Sensor e Erro em Relação ao Setpoint", fontsize=11, fontweight="bold")
    ax1.set_ylabel("Valor (%)", fontsize=10)
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.legend(loc="upper right", framealpha=0.9, fontsize=9)

    # 2. Decomposição P, I, D
    ax2.plot(t, hist["p_term"], color="#3498db", lw=1.6, alpha=0.9, label="Termo Proporcional (Kp · e)")
    ax2.plot(t, hist["i_term"], color="#9b59b6", lw=1.6, alpha=0.9, label="Termo Integral (Ki · ∫e dt [Anti-Windup])")
    ax2.plot(t, hist["d_term"], color="#e67e22", lw=1.6, alpha=0.9, label="Termo Derivativo (Kd · de/dt)")
    ax2.plot(t, hist["turn"], color="#27ae60", lw=2.2, label="Comando Total Turn (P + I + D)")
    ax2.axhline(0.0, color="#bdc3c7", linestyle=":", lw=1.0)
    ax2.set_title("2. Resposta Interna do PID: Contribuição Individual dos Termos e Sinal de Controle", fontsize=11, fontweight="bold")
    ax2.set_ylabel("Sinal de Controle", fontsize=10)
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend(loc="upper right", framealpha=0.9, fontsize=9)

    # 3. Velocidade e Motores
    ax3.plot(t, hist["tp"], color="#f39c12", lw=2.4, label="Velocidade Base Dinâmica Tp (Frenagem Adaptativa)")
    ax3.plot(t, hist["motor_dir"], color="#16a085", lw=1.6, linestyle="-.", label="Motor Direito [Porta A] (Tp - Turn)")
    ax3.plot(t, hist["motor_esq"], color="#2c3e50", lw=1.6, linestyle="--", label="Motor Esquerdo [Porta D] (Tp + Turn)")
    ax3.set_title("3. Atuação nos Motores: Diferencial de Curva e Redução de Velocidade em Trechos Críticos", fontsize=11, fontweight="bold")
    ax3.set_xlabel("Tempo Decorrido (segundos)", fontsize=10)
    ax3.set_ylabel("Velocidade (deg/s)", fontsize=10)
    ax3.grid(True, linestyle="--", alpha=0.5)
    ax3.legend(loc="upper right", framealpha=0.9, fontsize=9)

    fig.suptitle("Análise de Controle e Desempenho - LEGO Mindstorms EV3 PID Line Follower", fontsize=13, fontweight="bold", y=0.99)
    plt.tight_layout()
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    plt.savefig(output_file, dpi=300)
    plt.close()


def generate_simulation_gif(hist: Dict[str, np.ndarray], output_file: str, fps: int = 15):
    """
    Gera um GIF animado mostrando o trajeto 2D do robô e a telemetria em tempo real.
    """
    t = hist["t"]
    n = len(t)
    step = max(1, int(n / 60))

    frames = []

    for i in range(0, n, step):
        fig, (ax_track, ax_bar) = plt.subplots(1, 2, figsize=(7.2, 3.0), gridspec_kw={"width_ratios": [1.4, 1.0]})
        fig.patch.set_facecolor("#1e272e")

        # 1. Trajeto 2D do Robô
        ax_track.set_facecolor("#2f3640")

        # Pista: linha branca central e fundo preto
        ax_track.axhline(0, color="#ffffff", lw=16)
        ax_track.axhline(0, color="#111111", lw=8)

        # Robô
        rob_y = hist["robot_y"][i] * 0.08
        th = hist["theta"][i]

        rect = plt.Rectangle((-0.4, rob_y - 0.22), 0.8, 0.44, color="#f5f6fa", ec="#dcdde1", lw=1.5, zorder=5)
        ax_track.add_patch(rect)

        # Rodas
        ax_track.plot([-0.25, 0.25], [rob_y + 0.28, rob_y + 0.28], color="#718093", lw=5, zorder=6)
        ax_track.plot([-0.25, 0.25], [rob_y - 0.28, rob_y - 0.28], color="#718093", lw=5, zorder=6)

        # Sensor de cor frontal
        sensor_x = 0.45
        sensor_y = rob_y + (0.3 * math.sin(th))
        ax_track.scatter(sensor_x, sensor_y, color="#e84118", s=100, edgecolors="#ffffff", lw=1.8, zorder=7)

        ax_track.set_xlim(-1.2, 1.2)
        ax_track.set_ylim(-1.4, 1.4)
        ax_track.set_aspect("equal")
        ax_track.axis("off")
        ax_track.set_title(f"Simulação Cinemática 2D (t={t[i]:.2f}s)", color="#ffffff", fontsize=10, fontweight="bold")

        # 2. Painel de Telemetria
        ax_bar.set_facecolor("#2f3640")
        labels = ["Erro e(t)", "Turn", "Velocidade Tp"]
        values = [hist["error"][i], hist["turn"][i] * 0.1, hist["tp"][i] * 0.08]
        colors = ["#e84118", "#00a8ff", "#4cd137"]
        ax_bar.barh(labels, values, color=colors, height=0.45)
        ax_bar.set_xlim(-30, 30)
        ax_bar.tick_params(colors="#dcdde1", labelsize=8)
        for spine in ax_bar.spines.values():
            spine.set_color("#718093")
        ax_bar.set_title("Telemetria em Tempo Real", color="#ffffff", fontsize=10, fontweight="bold")
        ax_bar.grid(axis="x", linestyle=":", alpha=0.3, color="#ffffff")

        plt.tight_layout()
        fig.canvas.draw()
        rgba = np.asarray(fig.canvas.buffer_rgba())
        frames.append(Image.fromarray(rgba))
        plt.close(fig)

    if frames:
        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        frames[0].save(
            output_file,
            save_all=True,
            append_images=frames[1:],
            duration=int(1000 / fps),
            loop=0,
            optimize=True,
        )


def run_benchmark_table():
    """
    Executa a bateria de testes reproduzindo a tabela de sintonia experimental.
    """
    tests = [
        {"nome": "1. Somente P (Sub-amortecido)", "kp": 5.0, "ki": 0.0, "kd": 0.0, "tp": 200.0, "fb": 0.0, "diag": "Resposta lenta; atraso em curvas"},
        {"nome": "2. P Excessivo (Oscilação)", "kp": 25.0, "ki": 0.0, "kd": 0.0, "tp": 250.0, "fb": 2.0, "diag": "Hunting contínuo; trepidação severa"},
        {"nome": "3. PD com Kd Baixo", "kp": 12.0, "ki": 0.0, "kd": 30.0, "tp": 250.0, "fb": 2.0, "diag": "Amortecimento insuficiente ao sair de curvas"},
        {"nome": "4. PD com Kd Excessivo", "kp": 12.0, "ki": 0.0, "kd": 180.0, "tp": 250.0, "fb": 3.0, "diag": "Ruído derivativo e esforço mecânico alto"},
        {"nome": "5. PID com Ki Elevado", "kp": 12.0, "ki": 5.0, "kd": 116.0, "tp": 250.0, "fb": 3.0, "diag": "Windup severo; overshoot ao retornar à linha"},
        {"nome": "6. PID Otimizado Final", "kp": 12.0, "ki": 1.0, "kd": 116.0, "tp": 300.0, "fb": 4.0, "diag": "Rastreamento suave com frenagem dinâmica"},
    ]

    t, y_track = TrackGenerator.create_track()
    print("=" * 110)
    print(f"{'Caso de Teste':<30} | {'Kp':<4} | {'Ki':<4} | {'Kd':<5} | {'Tp':<4} | {'ISE':<8} | {'Chatter':<8} | {'Diagnóstico Experimental'}")
    print("=" * 110)

    for item in tests:
        ctrl = PIDController(
            kp=item["kp"],
            ki=item["ki"],
            kd=item["kd"],
            tp_maximo=item["tp"],
            fator_frenagem=item["fb"],
        )
        rob = LineFollowerKinematics()
        hist = simulate(ctrl, rob, t, y_track)
        ise, chatter, _ = evaluate_score(hist)

        print(
            f"{item['nome']:<30} | {item['kp']:<4.1f} | {item['ki']:<4.1f} | {item['kd']:<5.1f} | "
            f"{item['tp']:<4.0f} | {ise:<8.1f} | {chatter:<8.1f} | {item['diag']}"
        )
    print("=" * 110)


def main():
    parser = argparse.ArgumentParser(description="Simulador PID e Autoajuste para LEGO Mindstorms EV3")
    parser.add_argument("--run", action="store_true", help="Executa simulação padrão e gera gráficos em assets/")
    parser.add_argument("--autotune", action="store_true", help="Executa otimizador Twiddle para cálculo automático de Kp, Ki, Kd")
    parser.add_argument("--tune-table", action="store_true", help="Exibe tabela comparativa de sintonia de parâmetros")
    parser.add_argument("--graph-path", type=str, default="assets/pid_graph.png", help="Caminho do gráfico de desempenho")
    parser.add_argument("--gif-path", type=str, default="assets/simulation.gif", help="Caminho da animação GIF")

    args = parser.parse_args()

    # Se nenhum argumento for passado ou --run for especificado, executa a simulação padrão
    if len(sys.argv) == 1 or args.run:
        print("[Simulador] Executando simulação de trajetória com parâmetros ideais (Kp=12, Ki=1, Kd=116)...")
        t, y_track = TrackGenerator.create_track()
        ctrl = PIDController(kp=12.0, ki=1.0, kd=116.0, tp_maximo=300.0, fator_frenagem=4.0)
        rob = LineFollowerKinematics()
        hist = simulate(ctrl, rob, t, y_track)

        print(f"[Simulador] Gerando gráfico técnico em: {args.graph_path}...")
        plot_performance_graphs(hist, args.graph_path)

        print(f"[Simulador] Gerando animação do robô em: {args.gif_path}...")
        generate_simulation_gif(hist, args.gif_path)

        ise, chatter, cost = evaluate_score(hist)
        print(f"[Simulador] Concluído! Métricas: ISE = {ise:.2f} | Chatter = {chatter:.2f} | Custo = {cost:.2f}")

    if args.tune_table:
        run_benchmark_table()

    if args.autotune:
        print("\n[Autoajuste] Iniciando algoritmo Twiddle (Coordinate Ascent)...")
        best_kp, best_ki, best_kd, hist = autotune_twiddle()
        print(f"[Autoajuste] Finalizado com sucesso após {len(hist)} avaliações.")
        print(f"[Autoajuste] Ganhos Ótimos Sugeridos:")
        print(f"  -> Kp = {best_kp:.2f}")
        print(f"  -> Ki = {best_ki:.2f}")
        print(f"  -> Kd = {best_kd:.2f}")
        print(f"  -> Custo Inicial: {hist[0]['cost']:.2f} | Custo Otimizado: {hist[-1]['cost']:.2f}")


if __name__ == "__main__":
    main()
