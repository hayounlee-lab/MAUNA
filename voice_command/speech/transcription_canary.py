import string
import time
import pyaudio
import numpy as np
import requests
import torch
import playsound
import torchaudio
import warnings
import os
import gc
import logging
import nemo.collections.asr as nemo_asr
#from constants import START_KEYWORD, END_KEYWORD

# constants ts for keywords and audio files
START_KEYWORD = "mauna"
END_KEYWORD = "over"
ACTIVATE_PATH = "speech/audio/activate.wav"
MAUNA_PATH = "speech/audio/mauna.wav"
BEEP1_PATH = "speech/audio/help.wav"
BEEP2_PATH = "speech/audio/working.wav"

# PyAudio parameters
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000


class Transcriber:
    def __init__(self):
        # Suppress warning prints
        warnings.filterwarnings('ignore')
        os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
        logging.getLogger('nemo_logger').setLevel(logging.ERROR)

        self.translator = str.maketrans('', '', string.punctuation)
        
        # === Load NVIDIA Canary‑1B via Nemo ===
        # Note: We use Nemo's CanaryASRModel to load "nvidia/canary-1b".
        self.asr_model = nemo_asr.models.ASRModel.from_pretrained("nvidia/canary-1b")
        
        # Set device: use CUDA if available, else MPS (for macOS), else CPU.
        self.device = torch.device("cuda" if torch.cuda.is_available() 
                            else "mps" if torch.backends.mps.is_available() 
                            else "cpu")
        self.asr_model.to(self.device)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.cleanup()


    def cleanup(self):
        """VRAM cleanup """
        if hasattr(self, 'asr_model'):
            self.asr_model.cpu() 
            del self.asr_model
        
        gc.collect()
        
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
            torch.cuda.synchronize() 
        
        print("VRAM has been cleared.")

    def open_stream(self):
        p = pyaudio.PyAudio()
        stream = p.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        frames_per_buffer=CHUNK)
        return stream, p

    def save_audio_buffer(self, audio_buffer, file_path, sample_rate=RATE):
        """
        Concatenates a list of numpy arrays (audio chunks),
        converts them to a float32 tensor (scaled to [-1, 1]),
        and saves as a WAV file using torchaudio.
        Note: torchaudio.save requires the tensor to be on the CPU.
        """
        audio_input = np.concatenate(audio_buffer, axis=0)
        # Convert int16 to float32 in the range [-1, 1]
        audio_tensor = torch.from_numpy(audio_input.astype(np.float32) / 32768.0).unsqueeze(0)
        torchaudio.save(file_path, audio_tensor.cpu(), sample_rate)

    def process_audio(self, audio_buffer, asr_model):
        """
        Saves the given audio buffer to a temporary WAV file,
        then calls Nemo's transcribe() method on that file.
        """
        temp_wav_path = "temp_audio.wav"
        self.save_audio_buffer(audio_buffer, temp_wav_path, RATE)
        # Nemo's transcribe() expects a list of file paths.
        result = asr_model.transcribe([temp_wav_path],
                                    batch_size=1,
                                num_workers=0,
                                pretokenize=False,
                                return_hypotheses=False
        )
        # We assume result is a list and take the first element.
        return result[0].text.lower()

    def play_audio(self, filePath):
        try:
            playsound.playsound(filePath)
        except Exception as e:
            print(f"Error playing audio: {e}")


    def transcribe(self):
        stream, p = self.open_stream()
        total_audio_buffer = []  # Accumulates audio after keyword detection
        audio_buffer = []        # Temporary buffer for each chunk group
        keyword_detected = False
        text_buffer = ""
        curr_time = time.time()
        
        self.play_audio(ACTIVATE_PATH)
        try:
            while True:
                data = stream.read(CHUNK, exception_on_overflow=False)
                audio_chunk = np.frombuffer(data, np.int16)
                audio_buffer.append(audio_chunk)
                
                # Memory safety: clear accumulated audio if buffer grows too long
                if len(total_audio_buffer) * CHUNK / RATE >= 90:  # 90 seconds max
                    # Clear first 10 seconds worth of audio
                    clear_count = int(10 * RATE / CHUNK)
                    total_audio_buffer = total_audio_buffer[clear_count:]
                    print("Cleared old audio from buffer.")
                    keyword_detected = False
                
                # Process every 5 seconds of accumulated audio
                if len(audio_buffer) * CHUNK / RATE >= 5:
                    time_elapsed = time.time() - curr_time
                    print("Time elapsed:", time_elapsed)
                    transcription = self.process_audio(audio_buffer, self.asr_model)
                    #print(transcription)
                    #transcription = list(map(lambda x:x.lower(), transcription))
                    #print(transcription)
                    # transcription = transcription.translate(self.translator)
                    
                    if keyword_detected:
                        total_audio_buffer.extend(audio_buffer)
                    
                    if not keyword_detected:
                        print(f"No keyword detected yet: {transcription}")
                        #self.play_audio(MAUNA_PATH)
                        print(f"Say {START_KEYWORD} to begin")
                        if START_KEYWORD in transcription:
                            print(f"Keyword '{START_KEYWORD}' recognized! Starting transcription...")
                            self.play_audio(BEEP1_PATH)
                            start_index = transcription.find(START_KEYWORD) + len(START_KEYWORD)
                            text_buffer = transcription[start_index:]
                            keyword_detected = True
                    else:
                        if END_KEYWORD in transcription:
                            print(f"Keyword '{END_KEYWORD}' detected, ending transcription...")
                            self.play_audio(BEEP2_PATH)
                            total_text = self.process_audio(total_audio_buffer, self.asr_model)
                            # Combine initial buffered text with new transcription up to the end keyword
                            total_text = text_buffer + total_text.split(END_KEYWORD)[0]
                            print("Final transcription:", total_text)
                            return total_text
                        else:
                            print("Transcription in progress:", transcription)
                    
                    # Clear temporary audio buffer for the next round
                    audio_buffer = []
        except KeyboardInterrupt:
            print("Stopping...")
        finally:
            stream.stop_stream()
            stream.close()
            p.terminate()

# Constan
def main():
    transcriber = Transcriber()
    final_text = transcriber.transcribe()
    print("Final transcription:", final_text)

if __name__ == "__main__":
    main()
