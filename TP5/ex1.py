import numpy as np

x = np.array([1, 0])

def sum(w, x):
    return w.T @ np.concatenate(([1], x))

def step(s):
    if s >= 0:
        return 1
    else:
        return 0
    
def perceptron(w, x):
    return step(sum(w, x))

def test_table(w, t):
    for x, res in t:
        pred = perceptron(w, x)
        if pred != res:
            return False       
    
    print("test passed")
    return True


w_and = np.array([-1.5, 1, 1])
truth_table_and = [
    (np.array([0, 0]), 0),
    (np.array([0, 1]), 0),
    (np.array([1, 0]), 0),
    (np.array([1, 1]), 1),
]

w_or = np.array([-0.5, 1, 1])
truth_table_or = [
    (np.array([0, 0]), 0),
    (np.array([0, 1]), 1),
    (np.array([1, 0]), 1),
    (np.array([1, 1]), 1),
]

test_table(w_and, truth_table_and)
test_table(w_or, truth_table_or)

