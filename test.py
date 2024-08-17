import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
df = pd.read_csv('filtered_values.csv')

# Plot the filtered values
plt.figure(figsize=(10, 6))

# Plot FilteredValue1
plt.plot(df['Time'], df['FilteredValue_angle'], label='angle', marker='o')

# Plot FilteredValue2
plt.plot(df['Time'], df['FilteredValue_velo'], label='velo', marker='x')

plt.plot(df['Time'], df['Raw'], label='Raw', marker='x')

# Add labels and title
plt.xlabel('Time')
plt.ylabel('Filtered Values')
plt.title('Filtered Values Over Time')
plt.legend()

# Show the plot
plt.grid(True)
plt.show()