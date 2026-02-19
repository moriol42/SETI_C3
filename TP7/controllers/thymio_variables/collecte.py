from common import *


def step(s):
    if s >= 0:
        return 1
    else:
        return 0


def clip(x):
    return np.clip(x, -1, 1)


def mlp(weights, x):
    """
    weigths is a 3D-array contaning layers,
    which are matrices containing perceptron weight
    """
    out = np.asarray(x)
    for layer in weights:
        out = clip(layer @ np.concatenate(([1], out)))
    return out


w_evitement_obstacle = [2, 1, 1.2, 1.3]
w_back, w_fwd, w_pos, w_neg = w_evitement_obstacle

weights = [
    np.array([[w_fwd, -w_neg, -w_back, w_pos], [w_fwd, w_pos, -w_back, -w_neg]]),
]

print("Sampling period : ", timestep, "ms")

storage = {"prox": [], "scans": [], "cam": [], "cmds": []}


def collecte():
    while robot.step(timestep) != -1:
        storage["prox"].append(process_prox())
        prox = [distanceVal[0] / 4500, distanceVal[2] / 4500, distanceVal[4] / 4500]

        storage["scans"].append(process_lidar())

        storage["cam"].append(process_cam())

        [speed_r, speed_l] = mlp(weights, prox)

        ## Controle clavier ##
        command = keyboard.getKey()
        if command == keyboard.LEFT:
            speed_l -= 0.2
            speed_r += 0.2
        elif command == keyboard.RIGHT:
            speed_l += 0.2
            speed_r -= 0.2
        else:
            speed_l += 0.1
            speed_r += 0.1

        storage["cmds"].append([speed_l, speed_r])

        if command == ord("X"):
            motor_left.setVelocity(0)
            motor_right.setVelocity(0)
            robot.step(timestep)
            break

        motor_left.setVelocity(5 * speed_l)
        motor_right.setVelocity(5 * speed_r)

    with h5py.File("dataset_webots.hdf5", "w") as f:
        array1 = np.array(storage["cmds"])
        array2 = np.array(storage["scans"])
        array3 = np.array(storage["prox"])
        array4 = np.array(storage["cam"])

        f.create_dataset("thymio_commands", data=array1)
        f.create_dataset("thymio_scans", data=array2)
        f.create_dataset("thymio_prox", data=array3)
        f.create_dataset("thymio_cam", data=array4)

    print("Fichier dataset_webots.hdf5 sauvegardé avec succès.")
