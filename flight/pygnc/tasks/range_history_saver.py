# from ..configuration.pygnc import batch_range_filepath

    # print(f"Open {batch_range_filepath}")
    # read files from range filepath then read D bytes every S time steps
def read_and_save(input_file, output_file, D, S):
    with open(input_file, 'rb') as infile, open(output_file, 'wb') as outfile:
        print(f"Reading {input_file} and saving every {S}th measurement to {output_file}")
        count = 0
        while True:
            data = infile.read(D)
            if not data:
                break  # End of file
            count += 1
            if count % S == 0:
                outfile.write(data)

if __name__ == "__main__":
    # Replace 'input_file.bin' and 'output_file.bin' with your actual file names
    input_file = 'range.txt'
    output_file = 'sparse_measurements.bin'

    # Replace 100 and 5 with your desired values for D and S
    D = 120  # Number of bytes to read and save
    S = 30   # Save every S instances
    read_and_save(input_file, output_file, D, S)