import sys
import spacy
import os
speech_path = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.append(speech_path)
import torch
import numpy as np
# moving path to project root folder (parent) to import speech module
from speech import Transcriber
from publisher_node import CommandPublisher
from ollama_cmd_parser import CommandExtractor

# # supress warnings
# sys.stderr = open(os.devnull, 'w')

if __name__ == "__main__":
    # Check if CUDA is available and set the device accordingly
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    publisher = CommandPublisher() # publishes commands to ROS
    parser = CommandExtractor(
        model_name="gemma4:31b",
        base_url="http://localhost:11434"
    )

    with Transcriber() as tr:# loads the model in the wrapper class
        text = tr.transcribe()
       
    steps = parser.extract_commands(text) # extract commands
    print(steps)
    for message in steps:
        publisher.publish(message)
