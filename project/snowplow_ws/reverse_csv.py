import csv
import sys
import os

def reverse_csv_lines(filename):
    if not os.path.isfile(filename):
        print(f"Error: The file '{filename}' does not exist.")
        return
    with open(filename, 'r', newline='', encoding='utf-8') as csvfile:
        reader = list(csv.reader(csvfile))
    if not reader:
        print("The CSV file is empty.")
        return
    header = reader[0]
    data = reader[1:]
    reversed_data = list(reversed(data))
    reversed_csv = [header] + reversed_data
    with open(filename, 'a', newline='', encoding='utf-8') as csvfile:
        csvfile.write('===\n')
        writer = csv.writer(csvfile)
        writer.writerows(reversed_csv)
    print(f"Successfully appended reversed CSV to '{filename}'.")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python reverse_csv.py <filename.csv>")
    else:
        filename = sys.argv[1]
        reverse_csv_lines(filename)
