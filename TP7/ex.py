from sklearn.linear_model import Perceptron

# liste d'entrées possibles pour l'opérateur logique
data = [[0, 0], [0, 1], [1, 0], [1, 1]]

# labels correspondants selon liste d'entrées
ORlabels = [0, 1, 1, 1]
ANDlabels = [0, 0, 0, 1]
XORlabels = [0, 1, 1, 0]

# Création du perceptron
operateurOR = Perceptron(max_iter=40, tol=1e-3)
operateurAND = Perceptron(max_iter=40, tol=1e-3)
operateurXOR = Perceptron(max_iter=40, tol=1e-3)

# Entrainement
operateurOR.fit(data, ORlabels)
operateurAND.fit(data, ANDlabels)
operateurXOR.fit(data, XORlabels)

print("OR", operateurOR.predict(data) == ORlabels)
print("AND", operateurAND.predict(data) == ANDlabels)
print("XOR", operateurXOR.predict(data) == XORlabels)

print("Score OR:", operateurOR.score(data, ORlabels))
print("Score AND:", operateurAND.score(data, ANDlabels))
print("Score XOR:", operateurXOR.score(data, XORlabels))

import numpy as np
from sklearn.neural_network import MLPClassifier

classifier = MLPClassifier(max_iter=10000, hidden_layer_sizes=(2,), activation="tanh")
classifier.fit(data, XORlabels)
print(classifier.predict(data) == XORlabels)
print(classifier.score(data, XORlabels))
print(classifier.coefs_)
print(classifier.intercepts_)


from sklearn.neural_network import MLPRegressor

regressor = MLPRegressor(
    max_iter=10000, hidden_layer_sizes=(2, 2), activation="tanh", solver="lbfgs"
)
regressor.fit(data, XORlabels)
print(data, regressor.predict(data), XORlabels)
print(np.isclose(regressor.predict(data), XORlabels, atol=1e-3))
print(regressor.predict(data) - XORlabels)
print(regressor.score(data, XORlabels))
print(regressor.coefs_)
print(regressor.intercepts_)
