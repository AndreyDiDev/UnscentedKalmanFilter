import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
df = pd.read_csv('filtered_values_2D.csv')

# Plot the filtered values
plt.figure(figsize=(10, 6))

# Plot FilteredValue1
plt.plot(df['X'], df['Y'], label='Estimates', marker='o')

# Add labels and title
plt.xlabel('X')
plt.ylabel('Y')
plt.title('X vs Y')
plt.legend()
# Show the plot
plt.grid(True)
# plt.show()

# Plot XVelo
plt.figure(figsize=(10, 6))
plt.plot(df['Time'], df['XVelo'], label='XVelo', marker='x')
plt.xlabel('Time')
plt.ylabel('XVelo')
plt.title('XVelo Over Time')
plt.legend()
plt.grid(True)
# plt.show()

# Plot YVelo
plt.figure(figsize=(10, 6))
plt.plot(df['Time'], df['YVelo'], label='YVelo', marker='x')
plt.xlabel('Time')
plt.ylabel('YVelo')
plt.title('YVelo Over Time')
plt.legend()
plt.grid(True)
plt.show()



