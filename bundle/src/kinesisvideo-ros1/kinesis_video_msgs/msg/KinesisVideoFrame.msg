# see https://docs.aws.amazon.com/kinesisvideostreams/latest/dg/how-data.html#how-data-header-frame
uint32 index

uint64 decoding_ts      # in microseconds
uint64 presentation_ts  # in microseconds
uint64 duration         # in microseconds

uint8[] codec_private_data
uint8[] frame_data
uint16 flags

# see https://docs.aws.amazon.com/kinesisvideostreams/latest/dg/how-meta.html
diagnostic_msgs/KeyValue[] metadata
