import requests
import json

def filter_arrow_lines(response_text):
    lines = response_text.splitlines()
    filtered_lines = [line.strip() for line in lines if "->" in line]
    return filtered_lines

class CommandExtractor:
    def __init__(self, model_name="llama3.2:1b", base_url="http://localhost:11434"):
        self.model_name = model_name
        self.base_url = base_url
        
        self.prompt_template = """
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

    def _call_ollama(self, prompt):
        payload = {
            "model": self.model_name,
            "prompt": prompt,
            "stream": False,
        }
        
        try:
            response = requests.post(
                f"{self.base_url}/api/generate",
                json=payload,
                timeout=3000
            )
            
            if response.status_code == 200:
                return response.json().get("response", "")
            else:
                print(f"Ollama API error: {response.text}")
                return ""
        except requests.exceptions.RequestException as e:
            print(f"Request failed: {str(e)}")
            return ""

    def extract_commands(self, user_text):
        full_prompt = self.prompt_template + user_text
        
        response_text = self._call_ollama(full_prompt)
        
        ordered_commands = []
        if response_text:
            print("--- Model Response ---")
            print(response_text)
            print("----------------------")
            
            ordered_commands = filter_arrow_lines(response_text)
            print(f"Filtered Commands: {ordered_commands}")
            
        return ordered_commands

if __name__ == "__main__":
    extractor = CommandExtractor(model_name="gemma4:31b",base_url="http://172.17.0.3:11434")
    
    user_input = "Please move the microplate to the thermocycler and vortex the tube."
    result = extractor.extract_commands(user_input)
    
    print("\nFinal Result:")
    print(result)
