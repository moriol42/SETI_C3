import numpy as np
import matplotlib.pyplot as plt


list_x = np.arange(-2, 2.2, 0.2, dtype=float)

def mlp(weights, x):
    """
    weigths is a 3D-array contaning layers, which are arrays containing perceptron weight
    """
    out = np.asarray([x])
    for layer in weights:
        out = np.clip(layer @ out, -1, 1)
    return out[0]


weights_q3 = [np.array([[1], [0.5]]), np.array([[1, -1]])]
list_out = []

for x in list_x:
    list_out.append(mlp(weights_q3, [x]))

# Plot the graph
plt.figure(figsize=(10, 6))
plt.plot(list_x, list_out, linewidth=2, label=f"Sortie couche de sortie")
plt.xlabel("x")
plt.ylabel("Perceptron output")
plt.title("Sortie des différentes couches")
plt.legend()
plt.grid(True)
plt.show()
