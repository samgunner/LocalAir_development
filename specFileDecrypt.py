from speck import SpeckCipher
import argparse
import sys
import os
import pdb

def create_arg_parser():
    # Creates and returns the ArgumentParser object

    parser = argparse.ArgumentParser(description='Description of your app.')
    parser.add_argument('-i', '--inputFile', type=str,
                    help='Path to the input file.')
    parser.add_argument('-k', '--speckKey', type=str, default='{0xad, 0x1c, 0x4b, 0x6, 0x5c, 0x85, 0x2a, 0x48, 0xe4, 0xed, 0x33, 0x23, 0x4c, 0x9f, 0xed, 0x56, 0x23, 0x46, 0x59, 0xfa, 0x3c, 0x70, 0x82, 0x97, 0x45, 0xbd, 0x2b, 0xf1, 0xdc, 0xf4, 0xb6, 0xce}',
                    help='The Speck Key for decryption, in the form give in the arduino code (i.e. {0xab, ...., 0xce})')
    parser.add_argument('-u', '--uploadFormat', action='store_true')
    parser.set_defaults(uploadFormat=False)
    return parser


if __name__ == "__main__":
    arg_parser = create_arg_parser()

    args = arg_parser.parse_args()
    # This encryption example uses a block size of 128, as well as the 
    # key size of 256. It is best specificy both of these when setting 
    # up the speck Cipher
    
    
    key_string = args.speckKey
    
    # we can convert into somethign this code can use by doing this:
    key = [int(i,16) for i in key_string[1:-1].split(',')]
    
    # and then put it in a speck decrypter like this:
    
    my_speck = SpeckCipher(int.from_bytes(bytes(key), byteorder='big'), key_size=256, block_size=128)
    
    line_count = 0

    with open(args.inputFile, "r") as encryptedFile:
        for line in encryptedFile:
            if line == '\n':
                continue
            if args.uploadFormat:
                # this means that the file is in the funny format that you seem to get when
                # uploading via Strings from the sensor to James's server. We have to convert 
                # it into the normal format
                while True:
                    line_part = encryptedFile.readline()
                    if (line_part == '\n' or line_part == ''):
                        break
                    line = line.strip() + line_part.strip();
            line_count = line_count + 1
            AsciiLong = ''
            for i in range(60):
                encChunk = line[i*32:(i*32)+32]
                CipherTextChunk = int(encChunk, 16)
                PlaneTextChunk = hex(my_speck.decrypt(CipherTextChunk))
    
                # now decode the hex to ascii
                
                # so the message ends with guff, so we have to look out for the nulll byte,
                # which signifies the end.
                # convert to a byte array
                # we cut off the first two characters because they are the '0x' signifing hex
                try:
                    PlaneByteArray =  bytearray.fromhex(PlaneTextChunk[2:])
                except ValueError:
                    print("Non-Hex Value found in line {}, chunk {}, could not decrypt".format(
                        line_count,i), 
                        file=sys.stderr)
                    AsciiChunk = ''
                # look for the null character
                nullLoc = PlaneByteArray.find(b'\00')
    
                # if the location returned is 0 or higher then there is a null character in there
                # and we must ignore everything after it
                if (nullLoc > -1):
                    try:
                        AsciiChunk = bytearray.fromhex(PlaneTextChunk[2:])[0:nullLoc].decode()
                    except UnicodeDecodeError:
                        print('Error in line {}, chunk {}, count not convert to ascii'.format(
                            line_count, i), file=sys.stderr)
                        AsciiChunk = ''
                    except ValueError:
                        print("Non-Hex Value found in line {}, chunk {}, could not convert to ascii".format(
                            line_count,i), 
                            file=sys.stderr)
                        AsciiChunk = ''
                else:
                    try:
                        AsciiChunk = bytearray.fromhex(PlaneTextChunk[2:]).decode()
                    except UnicodeDecodeError:
                        print('Error in line {}, chunk {}, count not convert to ascii'.format(
                            line_count, i), file=sys.stderr)
                        AsciiChunk = ''
                    except ValueError:
                        print("Non-Hex Value found in line {}, chunk {}, could not convert to ascii".format(
                            line_count,i), 
                            file=sys.stderr)
                        AsciiChunk = ''
                # and concat the new block onto the exisiting string
                AsciiLong = AsciiLong + AsciiChunk
    
                # if we found the null character then we have reached the end of the message and can 
                # break the loop
                if (nullLoc > -1):
                    break
    
            # and print decoded line
            print(AsciiLong)
    
