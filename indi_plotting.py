import matplotlib.pyplot as plt

x_values = []
y_columns = []
labels = []

# Read data from file
with open('output.txt', 'r') as file:
    lines = file.readlines()

# Handle header
header = lines[0].strip().replace(',', ' ').split()
labels = header  # First label is X, rest are Y

# Parse numeric data
for line in lines[1:]:
    if line.strip():
        parts = line.strip().replace(',', ' ').split()
        numbers = list(map(float, parts))

        if not numbers:
            continue

        x_values.append(numbers[0])

        # Store each Y column
        for i, y in enumerate(numbers[1:]):
            if len(y_columns) <= i:
                y_columns.append([])
            y_columns[i].append(y)

# Plot using labels
for i, y_values in enumerate(y_columns):
    label = labels[i + 1] if i + 1 < len(labels) else f'Column {i + 2}'
    plt.plot(x_values, y_values, label=label)

plt.xlabel(labels[0] if labels else 'X (Column 1)')
plt.ylabel('Y Values')
plt.title('Plot of Columns vs. First Column')
plt.legend()
plt.grid(True)
plt.show()
