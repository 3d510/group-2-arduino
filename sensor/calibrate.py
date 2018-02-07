import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import glob


sensor_names = ['LEFT']
binwidth = 30

def plot(csv_file):
	print(csv_file)
	df = pd.read_csv(csv_file)
	data = df['value']
	plt.figure()
	n, b, patches = plt.hist(data)
	bin_max = np.argmax(n)
	print(b[bin_max] + (b[bin_max+1] - b[bin_max])/2)
	plt.savefig(csv_file.replace(".csv", ".png"))
	# plt.show()


if __name__ == '__main__':
	for sensor in sensor_names:
		csv_files = sorted(glob.glob(sensor + "*.csv"))
		for csv_file in csv_files:
			plot(csv_file)