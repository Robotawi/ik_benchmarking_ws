import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# Read IK benchmarking result data
data = pd.read_csv('ik_benchmarking_data.csv')

# Convert success column to boolean
data['found_ik'] = (data['found_ik'] == 1)

# Mark only success data
success_data = data[data['found_ik']]

# Create box and whisker plot for solve times
plt.figure(figsize=(10, 6))
sns.boxplot(y=success_data['solve_time'], showfliers=False)
plt.title('Solve Times for Successful Trials')
plt.ylabel('Microseconds')
plt.show()

# Calculate success rate
success_rate = data['found_ik'].mean()

# Create bar chart for success rate
plt.figure(figsize=(10, 6))
plt.bar(['Success Rate'], [success_rate])
plt.ylim(0, 1)
plt.title('Success Rate')
plt.ylabel('Rate')
plt.show()
