import csv
with open('vehicle1.csv', 'rb') as csvfile:
     logreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
     for row in logreader:
         print row
         print float(row[0])