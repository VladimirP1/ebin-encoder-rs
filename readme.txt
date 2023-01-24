This is a draft for the specification of esplog binary format.

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
1        0x02             block id (or 0x06 for accel)
4        (uint32_le)      time since last time block in us

Gyro data block
size     content          description 
1        0x03             block id
... (compressed gyro block)

Accel setup block
size     content          description 
1        0x04             block id
1        (uint8_t)        accel full range as 2^p g (ex: 4 means +-16g)

Accel data block
size     content          description 
1        0x05             block id
1        (uint8_t)        accel sample count in this block
2        (int16_t_le)     accel x
2        (int16_t_le)     accel y
2        (int16_t_le)     accel z
... (more accel data)


01 gyro setup
02 gyro time
03 gyro compressed data (rANS)
04 accel setup
05 accel uncompressed data
06 accel time