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
import re
import httpx
from openai import OpenAI
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

DEFAULT_LOCAL_QWEN_BASE_URL = "http://127.0.0.1:22002/v1"
DEFAULT_LOCAL_QWEN_MODEL = "/home/lry/OmniLRS/deploy/qwen3vl/models/Qwen3-VL-8B-Instruct-AWQ-4bit"

client = None
vlm_model = DEFAULT_LOCAL_QWEN_MODEL
vlm_temperature = 0.1
vlm_max_tokens = 256

def call_client_in_loop(client, base64_image):
  global vlm_model, vlm_temperature, vlm_max_tokens

  response = client.chat.completions.create(
    model=vlm_model,
    temperature=vlm_temperature,
    max_tokens=vlm_max_tokens,
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
        "1. rock_distribution MUST be a numeric float from 0.0 to 1.0 for obstacle complexity\n"
        "2. slope MUST be a numeric float from 0.0 to 1.0 for elevation complexity\n"
        "Output: Return ONLY a JSON object with this schema: "
        "{\"rock_distribution\": 0.0, \"slope\": 0.0, "
        "\"terrain_complexity\": \"flat|rocky|challenging\", "
        "\"explanation\": \"brief reason\"}\n"
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
  ok, buffer = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
  if not ok:
    raise ValueError("Failed to encode image")
  # Convert the image to bytes
  image_bytes = buffer.tobytes()
  # Encode the image in base64
  base64_image = base64.b64encode(image_bytes).decode('utf-8')
  return base64_image  

def parse_vlm_json(response_json):
  response_json = response_json.strip()

  # Clean up Markdown formatting.
  if response_json.startswith("```") and response_json.endswith("```"):
      response_json = response_json.strip("`").strip()
  if response_json.startswith("json"):
      response_json = response_json[len("json"):].strip()

  if not response_json:
      raise ValueError("Response content is empty")

  try:
      return json.loads(response_json)
  except json.JSONDecodeError:
      match = re.search(r"\{.*\}", response_json, flags=re.DOTALL)
      if not match:
          raise
      return json.loads(match.group(0))

def normalize_score(value, fallback=None):
  if isinstance(value, (int, float)):
      return max(0.0, min(1.0, float(value)))
  if not isinstance(value, str):
      return fallback

  text = value.strip().lower()
  if any(token in text for token in ["none", "no ", "minimal", "flat", "few", "sparse", "gentle", "low"]):
      return 0.15
  if any(token in text for token in ["moderate", "medium", "some"]):
      return 0.5
  if any(token in text for token in ["dense", "many", "high", "steep", "severe", "challenging"]):
      return 0.85
  return fallback

def normalize_terrain_complexity(value, rock_distribution, slope):
  if isinstance(value, str):
      normalized = value.strip().lower()
      if normalized in ["flat", "rocky", "challenging"]:
          return normalized

  if slope is not None and slope >= 0.55:
      return "challenging"
  if rock_distribution is not None and rock_distribution >= 0.35:
      return "rocky"
  return "flat"

def VLM_processing(depth_image):
  global client
  # Encode the image in base64
  base64_image = encode_image(depth_image)
  response = call_client_in_loop(client, base64_image)
  response_json = response.choices[0].message.content.strip()

  # Parse JSON response
  try:
      response_data = parse_vlm_json(response_json)
  except json.JSONDecodeError as e:
      rospy.logerr("JSONDecodeError: %s", str(e))
      rospy.logerr("Raw VLM response: %s", response_json)
      raise ValueError(f"Failed to decode JSON: {e}")
  
  # Extract terrain classification
  rock_distribution = normalize_score(response_data.get('rock_distribution'))
  slope = normalize_score(response_data.get('slope'))
  terrain_complexity = normalize_terrain_complexity(
      response_data.get('terrain_complexity'),
      rock_distribution,
      slope,
  )
  
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

  base_url = rospy.get_param(
      "~base_url",
      os.getenv("QWEN_VL_BASE_URL", os.getenv("LOCAL_VLM_BASE_URL", DEFAULT_LOCAL_QWEN_BASE_URL)),
  )
  api_key = rospy.get_param(
      "~api_key",
      os.getenv("QWEN_VL_API_KEY", os.getenv("LOCAL_VLM_API_KEY", "EMPTY")),
  )
  vlm_model = rospy.get_param(
      "~model",
      os.getenv("QWEN_VL_MODEL", os.getenv("LOCAL_VLM_MODEL", DEFAULT_LOCAL_QWEN_MODEL)),
  )
  vlm_temperature = rospy.get_param("~temperature", 0.1)
  vlm_max_tokens = rospy.get_param("~max_tokens", 256)

  timeout = rospy.get_param("~timeout", 120.0)
  disable_env_proxy = rospy.get_param("~disable_env_proxy", True)

  client = OpenAI(
      api_key=api_key,
      base_url=base_url,
      timeout=timeout,
      http_client=httpx.Client(timeout=timeout, trust_env=not disable_env_proxy),
  )
  rospy.loginfo("VLM endpoint: %s", base_url)
  rospy.loginfo("VLM model: %s", vlm_model)

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
