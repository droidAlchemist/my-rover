import asyncio
import os
import sys
import logging

from iot_controller.kinesis.kinesis_webrtc_helper import KinesisWebRTCHelper

logging.basicConfig(level=logging.INFO, stream=sys.stdout)

THING_NAME = os.getenv('THING_NAME')
AWS_DEFAULT_REGION = os.getenv('AWS_DEFAULT_REGION')
AWS_ACCESS_KEY_ID = os.getenv('AWS_ACCESS_KEY_ID')
AWS_SECRET_ACCESS_KEY = os.getenv('AWS_SECRET_ACCESS_KEY')
AWS_WEBRTC_CHANNEL = os.getenv('AWS_WEBRTC_CHANNEL')

async def run_client(client):
    await client.signaling_client()
    
async def main():
    if not AWS_WEBRTC_CHANNEL:
        raise Exception("AWS_WEBRTC_CHANNEL environment variable should be configured.\ni.e. export AWS_WEBRTC_CHANNEL=arn.yxxyx")
    if not AWS_ACCESS_KEY_ID:
        raise Exception("AWS_ACCESS_KEY_ID environment variable should be configured.\ni.e. export AWS_ACCESS_KEY_ID=1234")
    if not AWS_SECRET_ACCESS_KEY:
        raise Exception("AWS_SECRET_ACCESS_KEY environment variable should be configured.\ni.e. export AWS_SECRET_ACCESS_KEY=pass")
    if not AWS_DEFAULT_REGION:
        raise Exception("AWS_DEFAULT_REGION environment variable should be configured.\ni.e. export AWS_DEFAULT_REGION=ap-south-1")

    client = KinesisWebRTCHelper(
        client_id= "MASTER",
        region=AWS_DEFAULT_REGION,
        channel_arn=AWS_WEBRTC_CHANNEL,
        credentials=None,
        file_path="/home/joshua/video-media-samples/big-buck-bunny-1080p-30sec.mp4"
    )
    
    await run_client(client)

if __name__ == '__main__':
    asyncio.run(main())