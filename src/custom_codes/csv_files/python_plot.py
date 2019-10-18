import csv

with open('Joint_states.csv', newline='') as csvfile:
    data = list(csv.reader(csvfile))

print(data)
