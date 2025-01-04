import platform
from aiortc.contrib.media import MediaBlackhole, MediaPlayer, MediaRelay
import os

class MediaHelper:
    def __init__(self, file_path=None):
        self.file_path = file_path

    def create_media_track(self):
        relay = MediaRelay()
        options = {'framerate': '30', 'video_size': '1280x720'}
        system = platform.system()

        if self.file_path and not os.path.exists(self.file_path):
            raise FileNotFoundError(f"The file {self.file_path} does not exist.")

        if system == 'Darwin':
            media = MediaPlayer('default:default', format='avfoundation', options=options) if not self.file_path else MediaPlayer(self.file_path)
        elif system == 'Windows':
            media = MediaPlayer('video=Integrated Camera', format='dshow', options=options)
        elif system == 'Linux':
            media = MediaPlayer('/dev/video0', format='v4l2', options=options) if not self.file_path else MediaPlayer(self.file_path)
        else:
            raise NotImplementedError(f"Unsupported platform: {system}")

        audio_track = relay.subscribe(media.audio) if media.audio else None
        video_track = relay.subscribe(media.video) if media.video else None

        if audio_track is None and video_track is None:
            raise ValueError("Neither audio nor video track could be created from the source.")

        return audio_track, video_track
