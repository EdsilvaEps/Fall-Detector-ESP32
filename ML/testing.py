import math
import matplotlib.pyplot as plt
import numpy as np

GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
RESET = "\033[0m"
COLOR_MAP = {"red":RED, "orange": YELLOW, "green": GREEN}

class Tester:

  def __init__(self, predictor, data, labels, title=None, size=10):
    self.predictor = predictor
    self.data = data
    self.labels = labels
    self.title = title or "fall detection model"
    self.size = size
    self.guesses = []
    self.truths = []
    self.errors = []
    self.sles = []
    self.colors = []

  # add this when we are actually doing error verification
  def color_for(self, error, truth):
    if error<40 or error/truth < 0.2:
        return "green"
    elif error<80 or error/truth < 0.4:
        return "orange"
    else:
        return "red"


  def run_datapoint(self, i):
    datapoint = np.array([self.data[i]])
    guess = self.predictor.predict(datapoint)
    truth = self.labels[i]
    error = abs(guess[0] - truth)
    print(f"{COLOR_MAP['orange']} Guess: {guess} True: {truth} Error: {error}")

  def run(self):
    self.error = 0
    for i in range(self.size):
      self.run_datapoint(i)

  @classmethod
  def test(cls, function, data, labels):
    cls(function, data, labels).run()
