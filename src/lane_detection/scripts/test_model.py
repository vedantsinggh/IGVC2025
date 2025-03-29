import tensorflow as tf
model_path = "/home/daksh/Robotics_ws/src/lane_detection/scripts/model.h5"
print(f"🔍 Loading model from: {model_path}")
try:
    model = tf.keras.models.load_model(model_path)
    print("✅ Model loaded successfully!")
except Exception as e:
    print(f"❌ Error loading model: {e}")

