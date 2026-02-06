import numpy as np
import matplotlib.pyplot as plt


def perceptron(w, x):
    return np.clip(w.T @ x, -1, 1)


def test_table(w, t):
    for x, res in t:
        pred = perceptron(w, x)
        if pred != res:
            return False

    print("test passed")
    return True

list_x = np.arange(-2, 2.2, 0.2, dtype=float)
layer_1_out = [[], []]

def mlp(weights, x):
    """
    weigths is a 3D-array contaning layers, which are arrays containing perceptron weight
    """
    out = x
    for layer in weights:
        tmp = np.zeros(len(layer))
        for i, neuron in enumerate(layer):
            tmp[i] = perceptron(neuron, out)
        out = tmp
    return out


weights_q3 = [[np.array([1]), np.array([0.5])], [np.array([1, -1])]]
list_out = []

for x in list_x:
    list_out.append(mlp(weights_q3, [x]))

# Plot the graph
plt.figure(figsize=(10, 6))
plt.plot(list_x, layer_1_out[0], linewidth=2, label=f"Sortie couche d'entrée 1")
plt.plot(list_x, layer_1_out[1], linewidth=2, label=f"Sortie couche d'entrée 2")
plt.plot(list_x, list_out, linewidth=2, label=f"Sortie couche de sortie")
plt.xlabel("x")
plt.ylabel("Perceptron output")
plt.title("Sortie des différentes couches")
plt.legend()
plt.grid(True)
plt.show()
