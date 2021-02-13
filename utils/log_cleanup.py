import os
import re

if __name__ == '__main__':
  log_path = '/home/akramer/logs/hydro'
  # get list of csv files
  csv_files = [f for f in os.listdir(log_path) if 
                os.path.isfile(os.path.join(log_path,f)) and 
                f[-3:] == 'csv']


  for filename in csv_files:

    # get all data from file as single string
    with open(os.path.join(log_path,filename), 'r') as file:
      data = file.read()

    # remove any characters that don't fit in a csv file
    cleaned_data = re.sub('[^\d.,\n]','',data)

    # write back to file
    with open(os.path.join(log_path,filename), 'w') as file:
      file.write(cleaned_data)

