# Kinesis WebRTC
## Python implementation to start/stop rover streaming using AWS Kinesis WebRTC

## Export necessary variables

export THING_NAME=rover1
export AWS_ACCESS_KEY_ID=
export AWS_SECRET_ACCESS_KEY=
export AWS_DEFAULT_REGION=ap-south-1
export AWS_WEBRTC_CHANNEL=arn:aws:kinesisvideo:ap-south-1:783058511096:channel/iot-kinesis-demo/1731498094389


## Run script 

python3 kvsWebRTCClientMaster.py --file-path /home/joshua/video-media-samples/big-buck-bunny-1080p-30sec.mp4


python3 kvsWebRTCClientMaster.py --channel-arn arn:aws:kinesisvideo:[region]:[account-number]:channel/[channel-name]/[number] --file-path [your-video-file]

