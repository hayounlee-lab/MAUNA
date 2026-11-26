import google.generativeai as genai

def filter_arrow_lines(response_text):
    lines = response_text.splitlines()

    filtered_lines = [line.strip() for line in lines if "->" in line]

    return filtered_lines

class CommandExtractor:
    def __init__(self, api_key):
        genai.configure(api_key=api_key)
        self.gemini_model = genai.GenerativeModel('gemini-3-flash-preview')
        self.prompt_template = f"""
          The name of the location is as follows:
          location A: DNA/RNA extraction
          location B: Robot Base
          location C: Sample Station
          location D: Thermocycler

          There are several small tasks as follows:

          a: Go to location A
          b: Check the state of the switch at location A, and if it is red, press the switch
          c: Go to location B
          d: Vortex the tube at location C
          e: Read whiteboard
          f: Go to location C
          g: Go to location D
          h: Place the microplate into the thermocycler at location D
          i: Initializing

          Instructions:
          1. Using the above small tasks, break down the user request below. 
          2. Starting point can be adjusted using "go to" tasks.
          3. Show only the necessary result. Connect them with arrows "->".
          4. Please start with initializaing. 

          Now, process the following user request:

          User Request:
        """

    def extract_commands(self, user_text):
        full_prompt = self.prompt_template + user_text
        response = self.gemini_model.generate_content(full_prompt)
        response_text = response.text if response.parts else ""

        certain_commands = []
        uncertain_commands = []
        ordered_commands = []
        
        ordered_commands=[]
        if response_text:
            print(response_text)
            ordered_commands=filter_arrow_lines(response_text)
            print(ordered_commands)

        return ordered_commands
        
