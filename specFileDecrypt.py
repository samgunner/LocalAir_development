from speck import SpeckCipher


# This encryption example uses a block size of 128, as well as the 
# key size of 256. It is best specificy both of these when setting 
# up the speck Cipher

# on the Teensy the key looks likes this:

key_string = '{0xad, 0x1c, 0x4b, 0x6, 0x5c, 0x85, 0x2a, 0x48, 0xe4, 0xed, 0x33, 0x23, 0x4c, 0x9f, 0xed, 0x56, 0x23, 0x46, 0x59, 0xfa, 0x3c, 0x70, 0x82, 0x97, 0x45, 0xbd, 0x2b, 0xf1, 0xdc, 0xf4, 0xb6, 0xce}'


# we can convert into somethign this code can use by doing this:
key = [int(i,16) for i in key_string[1:-1].split(',')]

# and then put it in a speck decrypter like this:

my_speck = SpeckCipher(int.from_bytes(bytes(key), byteorder='big'), key_size=256, block_size=128)

with open("LAD_800106-000011.txt", "r") as encryptedFile:
    for line in encryptedFile:
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
            PlaneByteArray =  bytearray.fromhex(PlaneTextChunk[2:])
            # look for the null character
            nullLoc = PlaneByteArray.find(b'\00')

            # if the location returned is 0 or higher then there is a null character in there
            # and we must ignore everything after it
            if (nullLoc > -1):
                AsciiChunk = bytearray.fromhex(PlaneTextChunk[2:])[0:nullLoc].decode()
            else:
                AsciiChunk = bytearray.fromhex(PlaneTextChunk[2:]).decode()

            # and concat the new block onto the exisiting string
            AsciiLong = AsciiLong + AsciiChunk

            # if we found the null character then we have reached the end of the message and can 
            # break the loop
            if (nullLoc > -1):
                break

        # and print decoded line
        print(AsciiLong)

