import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import glob


sensor_names = ['FRONT_LEFT', 'FRONT_RIGHT', 'LEFT', 'RIGHT', 'BACK']
binwidth = 30

# used for 
x = [] # store value of (1/(V-3))
y = [] # store value of range R

def plot(csv_file):
	# print(csv_file)
	df = pd.read_csv(csv_file)
	data = df['value']
	plt.figure()
	n, b, patches = plt.hist(data)
	bin_max = np.argmax(n)
	V = b[bin_max] + (b[bin_max+1] - b[bin_max])/2
	# print(V)
	R = float(csv_file[-6:-4])

	x.append(1/V)
	y.append(R)
	plt.savefig(csv_file.replace(".csv", ".png"))
	# plt.show()


if __name__ == '__main__':
	for sensor in sensor_names:
		x = []
		y = []
		csv_files = sorted(glob.glob(sensor + "*.csv"))
		# csv_files = ["RIGHT-11.csv"]
		for csv_file in csv_files:
			plot(csv_file)

		x_np = np.array(x)
		y_np = np.array(y)
		A = np.vstack([x_np, np.ones(len(x_np))]).T
		m, c = np.linalg.lstsq(A, y_np)[0]
		

		plt.figure()
		plt.plot(x_np, y_np, 'o', label='Original data', markersize=10)
		plt.plot(x_np, m*x_np + c, 'r', label='Fitted line')
		plt.legend()
		plt.savefig(sensor + ".png")

		print(sensor)
		print(m,c)
