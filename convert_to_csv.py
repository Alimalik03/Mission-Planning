import csv

def map_waypoints_to_csv(input_file, output_file):
    """
    Reads a waypoints file with a structure similar to:
    
    0	1	0	16	0	0	0	0	52.7791000	-0.7068000	0.100000	1
    1	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	52.78014450	-0.70569990	100.000000	1

    and writes a CSV file with the columns: lat, lon, alt.
    
    Parameters:
        input_file (str): Path to the input waypoints text file.
        output_file (str): Path where the CSV output will be written.
    """
    
    with open(input_file, 'r') as infile, open(output_file, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        # Write CSV header
        csv_writer.writerow(['lat', 'lon', 'alt'])
        
        for line in infile:
            line = line.strip()
            # Skip empty lines
            if not line:
                continue
            # Split the line by whitespace (tabs or spaces)
            parts = line.split()
            # Ensure there are enough columns (we need at least 11)
            if len(parts) < 11:
                continue  # skip malformed lines
            # In our file, lat, lon, and alt are at indices 8, 9, and 10 respectively.
            lat = parts[8]
            lon = parts[9]
            alt = parts[10]
            csv_writer.writerow([lat, lon, alt])
    
    print(f"Conversion complete. CSV file created: {output_file}")

# Example usage:
if __name__ == "__main__":
    input_file = "competition_simulation_3.waypoints"# Replace with the path to your waypoints file.
    output_file = "Test_WP.csv"  # The CSV file to be created.
    map_waypoints_to_csv(input_file, output_file)
