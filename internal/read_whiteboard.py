import sys
import time
import cv2
import requests
import json
import re

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# QoS settings (Transient Local)
qos_profile = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)

class CameraSnapshotNode(Node):
    """Node to receive a single image from the /image_raw topic and convert it to memory (JPEG bytes)"""
    def __init__(self):
        super().__init__('camera_snapshot_node')
        self.bridge = CvBridge()
        self.image_bytes = None
        
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info("Waiting for image from /image_raw...")

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV BGR
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Encode into memory (JPEG) without saving to disk
            success, encoded_image = cv2.imencode('.jpg', frame)
            if success:
                self.image_bytes = encoded_image.tobytes()
                self.get_logger().info("Image captured successfully in memory!")
            else:
                self.get_logger().error("Failed to encode image to JPEG.")

            # Unsubscribe and trigger shutdown signal
            self.destroy_subscription(self.subscription)
            raise SystemExit

        except SystemExit:
            raise
        except Exception as e:
            self.get_logger().error(f"Could not process snapshot: {e}")
            sys.exit(1)


class CommandPublisherNode(Node):
    """Publisher node to publish final analyzed commands to ROS2 topic"""
    def __init__(self):
        super().__init__('command_publisher_node')
        self.publisher_ = self.create_publisher(String, 'remote_cmd', qos_profile)

    def publish(self, text):
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published message: "{msg.data}"')


def filter_arrow_lines(response_text):
    """Filter lines containing '->' from the response text."""
    lines = response_text.splitlines()
    return [line.strip() for line in lines if "->" in line]


class CommandExtractor:
    """Class to convert OCR text into predefined robot task sequences using Ollama LLM"""
    def __init__(self, model_name="gemma4:31b", base_url="http://192.168.0.69:11434"):
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
        """Internal function to send a request to Ollama API and return response text."""
        payload = {
            "model": self.model_name,
            "prompt": prompt,
            "stream": False,
        }
        try:
            response = requests.post(
                f"{self.base_url}/api/generate",
                json=payload,
                timeout=300
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


def main(args=None):
    rclpy.init(args=args)

    # -------------------------------------------------------------
    # Step 1: Capture image from ROS2 camera topic (stored in memory)
    # -------------------------------------------------------------
    snapshot_node = CameraSnapshotNode()
    try:
        rclpy.spin(snapshot_node)
    except (KeyboardInterrupt, SystemExit):
        pass
    
    image_bytes = snapshot_node.image_bytes
    snapshot_node.destroy_node()

    if not image_bytes:
        print("Error: No image captured from /image_raw.")
        rclpy.shutdown()
        return

    print(
    f"Image captured successfully. Size: {len(image_bytes)} bytes. Sending to"
    " OCR..."
    )
    # -------------------------------------------------------------
    # Step 2: Send image to OCR server on port 8080 and extract text
    # -------------------------------------------------------------
    ocr_url = "http://192.168.0.69:8080/ocr/"
    params = {"det_arch": "fast_base", "reco_arch": "crnn_vgg16_bn"}
    files = [
        ("files", ("snapshot.jpg", image_bytes, "image/jpeg")),
    ]

    print("Sending image to OCR server (8080)...")
    ocr_response = requests.post(ocr_url, params=params, files=files)
    
    if ocr_response.status_code != 200:
        print(f"OCR Server Error: {ocr_response.text}")
        rclpy.shutdown()
        return

    data = ocr_response.json() if hasattr(ocr_response, "json") else json.loads(ocr_response.text)
    
    print(f"\nExtracted OCR Text: {ocr_response.text}")
    extracted_texts = []
    for data_item in data:
        for page in data_item.get("pages", []):
            for block in page.get("blocks", []):
                for line in block.get("lines", []):
                    for word in line.get("words", []):
                        if "value" in word:
                            extracted_texts.append(word["value"])

    result_text = " ".join(extracted_texts)
    extracted_texts = []

    # Corrected loop structure matching the actual JSON response
    for data_item in data:
      for item in data_item.get("items", []):
        for block in item.get("blocks", []):
          for line in block.get("lines", []):
            for word in line.get("words", []):
              if "value" in word and word["value"]:
                extracted_texts.append(word["value"])

    result_text = " ".join(extracted_texts)
    print(f"Extracted OCR Text: {result_text}")

    # -------------------------------------------------------------
    # Step 3: Extract command sequence using Ollama LLM
    # -------------------------------------------------------------
    extractor = CommandExtractor(model_name="gemma4:31b", base_url="http://192.168.0.69:11434")
    result_commands = extractor.extract_commands(result_text)

    print("\nFinal Command Sequences:")
    print(result_commands)

    # -------------------------------------------------------------
    # Step 4: Publish extracted commands to ROS2 topic
    # -------------------------------------------------------------
    publisher_node = CommandPublisherNode()
    
    for message in result_commands:
        publisher_node.publish(message)
        time.sleep(0.1)  # Wait for network propagation

    # Short delay to ensure message propagation
    time.sleep(0.5)

    publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
