import pandas as pd

# Replace 'your_file.csv' with the path to your actual CSV file
csv_file_path = 'joint_vals.csv'

# Step 1: Read the CSV file and create a DataFrame
df = pd.read_csv(csv_file_path)
# df.columns = ['M1','M2','M3','M4','M5','M6','M7','M8','M9','M10','M11','M12']
# Now the data from the CSV file is stored in the 'df' DataFrame
print(len(df.values))

output_file = 'output.txt'
df.to_csv(output_file, index=False, sep=',', line_terminator='},\n{')