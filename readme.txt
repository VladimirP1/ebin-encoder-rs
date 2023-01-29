This is a draft for the specification of esplog binary format.

File structure
------------------------------------

Header
size     content          description 
6        EspLog           magic
1        0                format version (ascii 0)

Gyro setup block
size     content          description 
1        0x01             block id
1        0x01             compression algorithm revision
2        (uint16_le)      samples in gyro compressed block

Time block
size     content          description 
1        0x02             block id
4        (uint32_le)      time since last time block in us

Gyro data block
size     content          description 
1        0x03             block id
... (compressed gyro block)

Accel setup block
size     content          description 
1        0x04             block id
1        (uint8)          accel block size
1        (uint8)          accel full range as 2^p g (ex: 4 means +-16g)

Accel data block
size     content          description 
1        0x05             block id
2        (int16_le)       accel x
2        (int16_le)       accel y
2        (int16_le)       accel z
... (more accel data)

Global time offset
size     content          description 
1        0x06             block id
4        (int32_le)       time offset in us

01 gyro setup
02 gyro time
03 gyro compressed data (rANS)
04 accel setup
05 accel uncompressed data
06 global time offset

Compressed binary format for gyro
------------------------------------
The gyro processing pipeline should be built from 5 stages:
1. Gyro lowpass filter
2. Gyro integration in fixed-point quaternions
3. Quaternion decimation or interpolation
4. Encoding the quaternions as 8-bit quantized angular acceleration using a 
    closed-loop encoder (basically a P controller which tries to get the 
    decoder output as close as possible to encoder input)
5. Compression of quantized data using ryg-rans entropy coder. For the probability 
    distribution the most appropriate is selected from a pre-defined set of 16 
    laplace distributions.