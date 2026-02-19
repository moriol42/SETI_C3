import h5py
import numpy as np
import pickle

filename = "ai_controller_model_hyper_prox.model"

from sklearn.model_selection import train_test_split
from sklearn.neural_network import MLPRegressor

with h5py.File("dataset_webots_v2.hdf5", "r") as f:
    prox = f["thymio_prox"][:]
    cmds = f["thymio_commands"][:]

    print(f"Nombre échantillons : {len(prox)}")

prox_x_train, prox_x_test, prox_y_train, prox_y_test = train_test_split(prox, cmds)

regr = MLPRegressor(
    max_iter=50000, tol=1e-10, hidden_layer_sizes=(5, 10, 5), activation="tanh"
)
regr.fit(prox_x_train, prox_y_train)
print(regr.score(prox_x_test, prox_y_test))

pickle.dump(regr, open(filename, "wb"))
