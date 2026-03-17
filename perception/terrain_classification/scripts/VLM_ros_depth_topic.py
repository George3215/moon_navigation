#!/usr/bin/env python
"""
VLM-based Terrain Classification for Leo Rover
Single camera version - analyzes terrain and publishes navigation mode
"""
import rospy
import base64
import cv2
import json
import os
from openai import OpenAI
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

client = None

def call_client_in_loop(client, base64_image):

  response = client.chat.completions.create(
    model="gpt-4o",
    temperature=0.1,
    messages=[
      {
        "role": "system",
        "content": (
        "You are an AI assistant analyzing terrain for rover navigation.\n"
        "Terrain Categories:\n"
        "1. Flat: Terrain with few obstacles and minimal elevation changes\n"
        "2. Rocky: Terrain with rocks and obstacles but no significant slopes\n"
        "3. Challenging: Terrain with elevation changes and rocks\n"
        "Evaluation Criteria:\n"
        "1. Rock Distribution: Score 0-1 for obstacle complexity\n"
        "2. Slope: Score 0-1 for elevation complexity\n"
        "Output: Return ONLY a JSON object: {\"rock_distribution\": [0-1], \"slope\": [0-1], \"terrain_complexity\": \"flat|rocky|challenging\", \"explanation\": \"brief reason\"}\n"
        )
      },
      {
        "role": "user",
        "content": [
          {
            "type": "image_url",
            "image_url": {
              "url": f"data:image/jpeg;base64,{base64_image}"
            }
          },
          {
            "type": "text",
            "text": "Analyze this Leo rover terrain image. Evaluate rock distribution and slope, then classify as flat, rocky, or challenging."
          }
        ]
      }
    ],
  )
  return response

# Function to encode the image
def encode_image(image):
  # Encode the image
  _, buffer = cv2.imencode('.png', image)
  # Convert the image to bytes
  image_bytes = buffer.tobytes()
  # Encode the image in base64
  base64_image = base64.b64encode(image_bytes).decode('utf-8')
  return base64_image  

def VLM_processing(depth_image):
  global client
  # Encode the image in base64
  base64_image = encode_image(depth_image)
  response = call_client_in_loop(client, base64_image)
  response_json = response.choices[0].message.content.strip()
  
  # Clean up Markdown formatting
  if response_json.startswith("```") and response_json.endswith("```"):
      response_json = response_json.strip("```").strip()
  if response_json.startswith("json"):
      response_json = response_json[len("json"):].strip()
  
  if not response_json.strip():
      raise ValueError("Response content is empty")

  # Parse JSON response
  try:
      response_data = json.loads(response_json)
  except json.JSONDecodeError as e:
      rospy.logerr("JSONDecodeError: %s", str(e))
      raise ValueError(f"Failed to decode JSON: {e}")
  
  # Extract terrain classification
  rock_distribution = response_data.get('rock_distribution')
  slope = response_data.get('slope')
  terrain_complexity = response_data.get('terrain_complexity')
  
  rospy.logdebug("Rock Distribution: %s | Slope: %s", rock_distribution, slope)
  rospy.logdebug("Terrain Complexity: %s", terrain_complexity)
  
  return terrain_complexity


class CameraImageSubscriber:
  """Simple image subscriber for single camera"""
  def __init__(self, camera_topic):
    self.current_image = None
    self.bridge = CvBridge()
    self.sub = rospy.Subscriber(camera_topic, Image, self.image_callback)
  
  def image_callback(self, msg):
    try:
      self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
      rospy.logerr("Image conversion error: %s", str(e))


if __name__ == "__main__":
  rospy.init_node("vlm_terrain_classifier")

  api_key = rospy.get_param("~api_key", os.getenv("OPENAI_API_KEY", ""))
  if not api_key:
    rospy.logerr("Missing OpenAI API key. Set ~api_key or environment variable OPENAI_API_KEY.")
    raise SystemExit(1)

  client = OpenAI(
      api_key=api_key,
      base_url=rospy.get_param("~openai_base_url", "https://api.openai.com/v1"),
  )

  # Camera configuration
  camera_topic = rospy.get_param("~camera_topic", "/zed2/right/image_rect_color")
  rospy.loginfo("Camera Topic: %s", camera_topic)
  
  # Initialize camera subscriber
  camera_subscriber = CameraImageSubscriber(camera_topic)

  # Navigation mode publisher
  navigation_mode_topic = rospy.get_param("~navigation_mode_topic", "/navigation_mode")
  mode_pub = rospy.Publisher(navigation_mode_topic, String, queue_size=1)

  # Evaluation interval (seconds)
  evaluation_interval = rospy.get_param("~evaluation_interval", 5.0)
  rate = rospy.Rate(1.0 / evaluation_interval)
  mode_msg = String()

  rospy.loginfo("VLM Terrain Classifier initialized. Waiting for images...")

  while not rospy.is_shutdown():
    if camera_subscriber.current_image is not None:
      try:
        # VLM terrain analysis
        terrain_complexity = VLM_processing(camera_subscriber.current_image)
        
        # Publish navigation mode based on terrain classification
        if terrain_complexity == "flat":
          mode_msg.data = "Mode 1"
        elif terrain_complexity == "rocky":
          mode_msg.data = "Mode 2"
        elif terrain_complexity == "challenging":
          mode_msg.data = "Mode 3"
        else:
          mode_msg.data = "Mode 1"
          rospy.logwarn("Unknown terrain complexity: %s", terrain_complexity)
        
        mode_pub.publish(mode_msg)
        rospy.loginfo("Navigation Mode: %s | Terrain: %s", mode_msg.data, terrain_complexity)
        
      except Exception as e:
        rospy.logerr("VLM Processing Error: %s", str(e))
      
      rate.sleep()
    else:
      rospy.logdebug("Waiting for camera image...")
      rospy.sleep(0.5)
