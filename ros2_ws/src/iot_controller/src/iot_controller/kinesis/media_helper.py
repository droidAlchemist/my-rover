import platform
from aiortc.contrib.media import MediaPlayer, MediaRelay
import os

class MediaHelper:
    def __init__(self):
        # Do init
        print("Init media")

    def create_media_track(self):
        relay = MediaRelay()
        options = {'framerate': '30', 'video_size': '1280x720'}
        system = platform.system()

        if system == 'Darwin':
            media = MediaPlayer('default:default', format='avfoundation', options=options)
        elif system == 'Windows':
            media = MediaPlayer('video=Integrated Camera', format='dshow', options=options)
        elif system == 'Linux':
            media = MediaPlayer('/dev/video0', format='v4l2', options=options)
        else:
            raise NotImplementedError(f"Unsupported platform: {system}")

        audio_track = relay.subscribe(media.audio) if media.audio else None
        video_track = relay.subscribe(media.video) if media.video else None

        if audio_track is None and video_track is None:
            raise ValueError("Neither audio nor video track could be created from the source.")

        return audio_track, video_track
