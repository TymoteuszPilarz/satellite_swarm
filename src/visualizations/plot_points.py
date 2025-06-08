import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Lista plików CSV do wczytania
files = [
    '/Users/tymoteuszpilarz/Desktop/satellite_swarm_simulator/src/visualizations/leo_points.csv'
]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_box_aspect([1, 1, 1])

# Do automatycznego ustalania wspólnych limitów
all_points = []

# Wczytaj każdy plik i narysuj punkty
for file in files:
    df = pd.read_csv(file)
    ax.scatter(df['x'], df['y'], df['z'], label=file.split('/')[-1])
    all_points.append(df[['x', 'y', 'z']])

# Ustal zakresy osi na podstawie wszystkich punktów
all_data = pd.concat(all_points)
min_limit = all_data.min().min()
max_limit = all_data.max().max()

ax.set_xlim(min_limit, max_limit)
ax.set_ylim(min_limit, max_limit)
ax.set_zlim(min_limit, max_limit)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()

plt.show()
