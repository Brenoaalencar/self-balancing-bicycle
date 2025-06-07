import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import re

# Configuração da porta serial
ser = serial.Serial('COM9', 115200, timeout=0.1)  # Altere COM9 conforme sua porta

# Dados a plotar
x_vals = []
u_vl_vals = []
u_vr_vals = []
state_vals = [[] for _ in range(7)]  # Listas para cada um dos 7 estados

# Expressão regular para extrair u_vl, u_vr e os 7 valores de x
pattern = re.compile(
    r"u_vl: ([\-\d\.eE]+), u_vr: ([\-\d\.eE]+), x: \[([\-\d\.eE]+), ([\-\d\.eE]+), ([\-\d\.eE]+), "
    r"([\-\d\.eE]+), ([\-\d\.eE]+), ([\-\d\.eE]+), ([\-\d\.eE]+)\]"
)

# Nomes dos estados para exibição
state_labels = [
    "θ (roll) [rad]",
    "θ̇ (roll_d) [rad/s]",
    "α (heading) [rad]",
    "α̇ (heading_d) [rad/s]",
    "ẏ (vel. linear) [m/s]",
    "∫e_α (erro int heading)",
    "∫e_y (erro int pos)"
]

# Setup do gráfico
fig, axs = plt.subplots(4, 2, figsize=(12, 8))
axs = axs.flatten()

def update(frame):
    global x_vals, u_vl_vals, u_vr_vals, state_vals

    try:
        line = ser.readline().decode(errors='ignore').strip()
        match = pattern.search(line)
        if match:
            u_vl = float(match.group(1))
            u_vr = float(match.group(2))
            states = [float(match.group(i)) for i in range(3, 10)]

            x_vals.append(len(x_vals))
            u_vl_vals.append(u_vl)
            u_vr_vals.append(u_vr)
            for i in range(7):
                state_vals[i].append(states[i])

            # Manter 200 amostras
            max_len = 200
            if len(x_vals) > max_len:
                x_vals = x_vals[-max_len:]
                u_vl_vals = u_vl_vals[-max_len:]
                u_vr_vals = u_vr_vals[-max_len:]
                for i in range(7):
                    state_vals[i] = state_vals[i][-max_len:]

            # Plot dos controles
            axs[0].clear()
            axs[0].plot(x_vals, u_vl_vals, label="u_vl")
            axs[0].plot(x_vals, u_vr_vals, label="u_vr")
            axs[0].set_title("Comando de Controle")
            axs[0].set_ylim(-1.1, 1.1)
            axs[0].legend()
            axs[0].grid()

            # Plot dos estados
            for i in range(7):
                axs[i+1].clear()
                axs[i+1].plot(x_vals, state_vals[i], label=state_labels[i])
                axs[i+1].set_title(state_labels[i])
                axs[i+1].legend()
                axs[i+1].grid()

            fig.tight_layout()

    except Exception as e:
        print(f"Erro: {e}")

# Iniciar animação
ani = FuncAnimation(fig, update, interval=100)
plt.show()
