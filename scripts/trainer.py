"""
part to train a keras model

Usage:
    train.py --cfg=<cfg_path> --tub=<tub_path> --model=<model_path> [--transfer=<model_path>] [--type="categorical"] [--continuous] [--aug]

Options:
    -h --help     Show this screen.
"""
from donkeycar.templates import train

class Trainer():
    def __init__(self, cfg, dirs, model, transfer, model_type, continuous, aug):
        self.args = (cfg, dirs, model, transfer, model_type, continuous, aug)
        print("------------------")
        print("starting trainer")
        for val in zip(("dirs", "model", "transfer", "model_type", "continuous", "aug"), (dirs, model, transfer, model_type, continuous, aug)):
            name, value = val
            print(name, ":", value)
        print("------------------")
        self.do_update = False

    def update(self):
        while True:
            if self.do_update:
                train.multi_train(*self.args)

    def run_threaded(self):
        self.do_update = True

if __name__ == "__main__":
    from docopt import docopt
    import donkeycar as dk

    args = docopt(__doc__)
    cfg = dk.load_config(args["--cfg"])
    trainer = Trainer(cfg, dirs=args["--tub"], model=args['--model'], transfer=args["--transfer"], model_type=args["--type"], continuous=args["--continuous"], aug=args["--aug"])

    V = dk.vehicle.Vehicle()
    V.add(trainer, threaded=True)
    V.start()