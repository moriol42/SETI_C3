from common import *
import pickle

filename = "ai_controller_model_hyper_prox.model"

regr = pickle.load(open(filename, "rb"))

def drive():
    while robot.step(timestep) != -1:
        prox = process_prox()
        print(prox)
        pred = regr.predict([prox])[0]
        print(pred)
        [speed_l, speed_r] = pred

        motor_left.setVelocity(5 * speed_l)
        motor_right.setVelocity(5 * speed_r)
