import numpy as np
import matplotlib.pyplot as plt

x = np.array([1, 0])


def sum(w, x):
    return w.T @ np.concatenate(([1], x))


def step(s):
    if s >= 0:
        return 1
    else:
        return 0


def perceptron(w, x):
    return np.clip(sum(w, x), -1, 1)


def test_table(w, t):
    for x, res in t:
        pred = perceptron(w, x)
        if pred != res:
            return False

    print("test passed")
    return True


list_x = np.arange(-2, 2.2, 0.2, dtype=float)

list_w = np.arange(-1, 1, 1 / 15, dtype=float)
list_y = []

for w in list_w:
    w1 = np.array([0, w])

    list_y.append(np.array([perceptron(w1, [x]) for x in list_x]))

# Plot the graph
plt.figure(figsize=(10, 6))
colors = plt.cm.viridis(np.linspace(0, 1, len(list_y)))
for i in range(len(list_y)):
    plt.plot(list_x, list_y[i], linewidth=2, label=f"w = {list_w[i]:.2f}", color=colors[i])
plt.xlabel("x")
plt.ylabel("Perceptron output")
plt.title("Output for $w \in [-1, 1]$")
plt.legend()
plt.grid(True)
plt.show()
