import argparse
import csv

name_default = "EntfernungProSekunde1.csv"

def auto_string(x):
    return str(x)

def avg(list):
    sum = 0
    for i in range(len(list)):
        sum += int(list[i])
    return sum / int(len(list))

def kleinste(list):
    min = 0
    for i in range(1, len(list)):
        if int(list[i]) < int(list[i - 1]):
            min = int(list[i])
        elif int(list[i]) < min and min != 0:
            min = int(list[i])
    return min

def hoechste(list):
    min = 0
    for i in range(1, len(list)):
        if int(list[i]) > int(list[i - 1]):
            min = int(list[i])
        elif int(list[i]) > min and min != 0:
            min = int(list[i])
    return min

parser = argparse.ArgumentParser(description='Gibt an, den Durchscnitt von der Liste, der höchste und auch der kleinste Wert in der Liste.')
parser.add_argument('--name', type=auto_string, help='Der Name der .csv Datei',
                    default=name_default)
args = parser.parse_args()

name_default = args.name

results = []
with open(name_default, newline='') as csvfile:
    read = csv.reader(csvfile, delimiter=',')
    # testwriter.writerow(intensity_axis)
    # print(testreader)
    for row in read:
        results.extend(row)

print("Durchschnitt der Liste: ", avg(results))
print("Hoechster Wert der Liste: ", hoechste(results))
print("Kleinster Wert der Liste: ", kleinste(results))