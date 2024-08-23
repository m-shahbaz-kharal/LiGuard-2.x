import time
import matplotlib.pyplot as plt
import pickle

class Profiler:
    def __init__(self, name):
        self.name = name
        self.targets = dict()

        self.compiled = False

    def add_target(self, name):
        if self.compiled: return
        if name not in self.targets:
            self.targets[name] = dict()
            self.targets[name]['tick'] = [time.time()]
            self.targets[name]['freq'] = 1
        else:
            self.targets[name]['tick'].append(time.time())
            self.targets[name]['freq'] += 1


    def end_target(self, name):
        if self.compiled: return
        if 'tock' not in self.targets[name]:
            self.targets[name]['tock'] = [time.time()]
        else:
            self.targets[name]['tock'].append(time.time())

    def save(self, path):
        with open(path, 'wb') as f: pickle.dump(self.targets, f)

    def load(self, path):
        with open(path, 'rb') as f: self.targets = pickle.load(f)

    def compile(self):
        if self.compiled: return
        self.compiled = True
        for target in self.targets:
            duration = 0
            freq = self.targets[target]['freq']
            for i in range(freq):
                duration += self.targets[target]['tock'][i] - self.targets[target]['tick'][i]
            self.targets[target]['duration'] = duration
            avg_duration = duration / freq
            self.targets[target]['avg_duration'] = avg_duration

    def plot_durations(self):
        targets = list(self.targets.keys())
        avg_durations = [self.targets[target]['avg_duration'] for target in targets]

        plt.figure(figsize=(10, 6))
        plt.bar(targets, avg_durations)
        plt.xlabel('Targets')
        plt.ylabel('Average Duration')
        plt.title('Average Durations of Targets')
        plt.xticks(rotation='vertical')  # Rotate the x-axis labels vertically

        plt.show()

if __name__ == '__main__':
    x = Profiler('LiGuardMainLoopProfile')
    x.load('test_configs/LiGuardMainLoopProfile')
    x.compile()
    x.plot_durations()