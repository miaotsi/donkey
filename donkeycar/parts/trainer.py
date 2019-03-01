from donkeycar.templates import train

class Trainer():
    def __init__(self, cfg, dirs, model, transfer, model_type, continuous, aug):
        self.args = (cfg, dirs, model, transfer, model_type, continuous, aug)

    def update(self):
        train.multi_train(*self.args)

    def run_threaded(self):
        pass


