from vntpa.training import CallbackList, TrainConfig


class Recorder:
    def __init__(self):
        self.started = False
        self.finished = None

    def on_train_start(self, trainer):
        self.started = True

    def on_train_end(self, trainer, returncode):
        self.finished = returncode


def test_train_command():
    command = " ".join(TrainConfig(["--steps", "10"]).command())
    assert "thirdparty.pan.planner.train" in command


def test_callback_list():
    recorder = Recorder()
    callbacks = CallbackList([recorder])
    callbacks.on_train_start(None)
    callbacks.on_train_end(None, 0)
    assert recorder.started and recorder.finished == 0
