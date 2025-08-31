class EMAFilter:
    def __init__(self, alpha=0.3):
        self.value = None
        self.alpha = alpha

    def update(self, new_value: float) -> float:
        if self.value is None:
            self.value = new_value
        else:
            self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        return self.value