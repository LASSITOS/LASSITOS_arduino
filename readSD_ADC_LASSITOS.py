import os
import csv
import sys


def main(path='./'):  # Read the first sector of the first disk as example.
    """Demo usage of function."""
    file='./test1.csv'
    args = sys.argv[1:]
    if len(args) == 2 and args[0] == '-file':
        file=args[1]
    if os.name == "nt":
        # Windows based OS normally uses '\\.\physicaldriveX' for disk drive identification.
        read_sector(r"\\.\physicaldrive1",0,path=path)
        print('done')
    else:
        # Linux based OS normally uses '/dev/diskX' for disk drive identification.
        print(read_sector("/dev/disk0"),path=path)


def read_sector(disk, sector_no=0,path='./'):
    """Read a single sector of the specified disk.
    Keyword arguments:
    disk -- the physical ID of the disk to read.
    sector_no -- the sector number to read (default: 0).
    """
    # Static typed variable
    read = None
    # File operations with `with` syntax. To reduce file handeling efforts.
    with open(disk, 'rb') as fp:
        fp.seek(sector_no * 512)
        read = fp.read(12)
        fName = read.decode()[:-2]
        read = fp.read(4)
        sAddress = int.from_bytes(read,"little")
        read = fp.read(4)
        ADC_Size = int.from_bytes(read,"little")
        read = fp.read(4)
        SD_Size = int.from_bytes(read,"little")

        print('start address = ' + str(sAddress))
        print('ADC size = ' + str(ADC_Size))
        print('SD frames = ' + str(SD_Size))
        print('File Name = ' + str(fName))
        file=path+'ADC'+str(fName[:6])+'_'+str(fName[6:])+'.csv'
        print('File path = ' + file)
        fp.seek(sAddress * 512)
        #read = fp.read(16)
        
        
        
        with open(file, 'w', encoding='UTF8', newline='') as f:
            f.write('#File Name = ' + str(fName)+'\n') 
            f.write('#ADC size = ' + str(ADC_Size)+'\n')           
            f.write('#SD frames = ' + str(SD_Size)+'\n')           
            f.write('## \n')
            
            writer = csv.writer(f)
            for i in range(sAddress, SD_Size+sAddress+1):
                read = fp.read(512)
                for j in range(0,32):
                    writer.writerow([int.from_bytes(read[j*16:j*16+4], 'little'), read[j*16+4:j*16+8].hex(), read[j*16+8:j*16+12].hex(), read[j*16+12:j*16+16].hex()])
                    
            
       
    return read


if __name__ == "__main__":
    main()