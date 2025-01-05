import asyncio
from os import getenv
import sys
import logging
from iot_controller.kinesis.kinesis_video_client import KinesisVideoClient

THING_NAME = getenv('THING_NAME')
AWS_DEFAULT_REGION = getenv('AWS_DEFAULT_REGION')
AWS_ACCESS_KEY_ID = getenv('AWS_ACCESS_KEY_ID')
AWS_SECRET_ACCESS_KEY = getenv('AWS_SECRET_ACCESS_KEY')
AWS_WEBRTC_CHANNEL = getenv('AWS_WEBRTC_CHANNEL')

class KinesisConnectionHelper:
    def __init__(self, logger):
        self.is_aws_keys_defined()
        self.logger = logger        
        logging.basicConfig(level=logging.INFO, stream=logger)   
        self.loop = asyncio.get_event_loop()   

    def is_aws_keys_defined(self):
        # check if necessary keys are exported
        if not AWS_WEBRTC_CHANNEL:
            raise Exception("AWS_WEBRTC_CHANNEL environment variable should be configured.\ni.e. export AWS_WEBRTC_CHANNEL=arn.yxxyx")
        if not AWS_ACCESS_KEY_ID:
            raise Exception("AWS_ACCESS_KEY_ID environment variable should be configured.\ni.e. export AWS_ACCESS_KEY_ID=1234")
        if not AWS_SECRET_ACCESS_KEY:
            raise Exception("AWS_SECRET_ACCESS_KEY environment variable should be configured.\ni.e. export AWS_SECRET_ACCESS_KEY=pass")
        if not AWS_DEFAULT_REGION:
            raise Exception("AWS_DEFAULT_REGION environment variable should be configured.\ni.e. export AWS_DEFAULT_REGION=ap-south-1")

    async def run_client(self):
        await self.client.signaling_client()
        
    async def main(self):
        # Main function to start kvs webrtc streaming
        self.logger.info("Initialize kvs")
        self.client = KinesisVideoClient(
            client_id= "MASTER",
            region=AWS_DEFAULT_REGION,
            channel_arn=AWS_WEBRTC_CHANNEL,
            credentials=None
        )
        await self.run_client()

    def start(self):
        # Start asyncio loop
        self.loop.run_until_complete(self.main())
        self.logger.info("Start asynio loop for web rtc")

    def stop(self):
        # Stop asyncio loop
        self.loop.stop()
        self.logger.info("Try to stop webrtc asynio loop")
