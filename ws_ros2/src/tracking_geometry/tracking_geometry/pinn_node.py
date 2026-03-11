"""
PINN Prediction Node for AIRHOUND.

Subscribes to the 3D Kalman filter state and publishes predicted future
positions using a trained Physics-Informed Neural Network. During detection
dropout, the PINN provides anticipatory state estimates that allow the
control node to proactively adjust yaw, pitch, and altitude.

Inputs:
    /perception/target_3d (PointStamped) - Current 3D position from depth fusion
    /tracking/kalman_state (Float64MultiArray) - [x, y, z, vx, vy, vz] from Kalman

Outputs:
    /tracking/pinn_predicted (PointStamped) - Predicted 3D position
    /tracking/pinn_state (Float64MultiArray) - Full predicted [x, y, z, vx, vy, vz]
    /tracking/pinn_active (Bool) - Whether PINN is actively predicting (dropout mode)

Parameters:
    model_path (str): Path to trained PINN model (.pth, .onnx, or .engine)
    norm_stats_path (str): Path to normalization stats (.npz)
    prediction_horizon (float): How far ahead to predict (seconds). Default: 0.1
    dropout_timeout (float): Seconds without detection before activating PINN. Default: 0.1
    max_prediction_time (float): Max duration to predict without detection. Default: 2.0
    use_onnx (bool): Use ONNX Runtime instead of PyTorch. Default: False
"""

import numpy as np
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64MultiArray, Bool


class PINNPredictionNode(Node):
    def __init__(self):
        super().__init__('pinn_prediction_node')

        # Parameters
        self.declare_parameter('model_path', 'models/pinn/pinn_best.pth')
        self.declare_parameter('norm_stats_path', 'models/pinn/norm_stats.npz')
        self.declare_parameter('prediction_horizon', 0.1)
        self.declare_parameter('dropout_timeout', 0.1)
        self.declare_parameter('max_prediction_time', 2.0)
        self.declare_parameter('use_onnx', False)
        self.declare_parameter('publish_rate_hz', 30.0)

        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.norm_path = self.get_parameter('norm_stats_path').get_parameter_value().string_value
        self.prediction_horizon = self.get_parameter('prediction_horizon').get_parameter_value().double_value
        self.dropout_timeout = self.get_parameter('dropout_timeout').get_parameter_value().double_value
        self.max_prediction_time = self.get_parameter('max_prediction_time').get_parameter_value().double_value
        self.use_onnx = self.get_parameter('use_onnx').get_parameter_value().bool_value
        publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        # State
        self.last_detection_time: Optional[float] = None
        self.last_state: Optional[np.ndarray] = None  # [x, y, z, vx, vy, vz]
        self.dropout_start_time: Optional[float] = None
        self.pinn_active = False

        # Load model
        self.model = None
        self.norm_stats = None
        self.ort_session = None
        self.trt_context = None
        self.trt_inputs = None
        self.trt_outputs = None
        self._load_model()

        # QoS
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        qos_default = QoSProfile(depth=10)

        # Subscribers
        self.create_subscription(
            PointStamped, '/perception/target_3d',
            self.on_target_3d, qos
        )
        self.create_subscription(
            Float64MultiArray, '/tracking/kalman_state',
            self.on_kalman_state, qos
        )

        # Publishers
        self.pred_pub = self.create_publisher(
            PointStamped, '/tracking/pinn_predicted', qos_default
        )
        self.state_pub = self.create_publisher(
            Float64MultiArray, '/tracking/pinn_state', qos_default
        )
        self.active_pub = self.create_publisher(
            Bool, '/tracking/pinn_active', qos_default
        )

        # Timer for prediction loop
        self.create_timer(1.0 / publish_rate, self.prediction_loop)

        self.get_logger().info(
            f"PINNPredictionNode initialized. "
            f"Model: {self.model_path}, Horizon: {self.prediction_horizon}s"
        )

    def _load_model(self):
        """Load PINN model (PyTorch or ONNX)."""
        import os

        # Load normalization stats
        if os.path.exists(self.norm_path):
            d = np.load(self.norm_path)
            self.norm_stats = {
                'input_mean': d['input_mean'].astype(np.float32),
                'input_std': d['input_std'].astype(np.float32),
                'target_mean': d['target_mean'].astype(np.float32),
                'target_std': d['target_std'].astype(np.float32),
            }
            self.get_logger().info(f"Loaded normalization stats from {self.norm_path}")
        else:
            self.get_logger().warn(f"Norm stats not found: {self.norm_path}")
            return

        if not os.path.exists(self.model_path):
            self.get_logger().warn(f"Model not found: {self.model_path}. PINN disabled.")
            return

        if self.model_path.endswith('.engine'):
            self._load_tensorrt()
        elif self.use_onnx or self.model_path.endswith('.onnx'):
            self._load_onnx()
        else:
            self._load_pytorch()

    def _load_pytorch(self):
        """Load PyTorch model."""
        try:
            import torch
            # Import model class from training script
            # For deployment, the model architecture is embedded here
            from torch import nn

            class TrajectoryPINN(nn.Module):
                def __init__(self, hidden_dim=128, num_layers=3):
                    super().__init__()
                    layers = []
                    in_dim = 7
                    for _ in range(num_layers):
                        layers.extend([nn.Linear(in_dim, hidden_dim), nn.ReLU()])
                        in_dim = hidden_dim
                    layers.append(nn.Linear(in_dim, 6))
                    self.net = nn.Sequential(*layers)

                def forward(self, x):
                    return self.net(x)

            self.model = TrajectoryPINN()
            state_dict = torch.load(self.model_path, map_location='cpu', weights_only=True)
            self.model.load_state_dict(state_dict)
            self.model.eval()
            self.get_logger().info(f"Loaded PyTorch PINN from {self.model_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load PyTorch model: {e}")
            self.model = None

    def _load_onnx(self):
        """Load ONNX model via ONNX Runtime."""
        try:
            import onnxruntime as ort
            providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
            self.ort_session = ort.InferenceSession(self.model_path, providers=providers)
            self.get_logger().info(f"Loaded ONNX PINN from {self.model_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load ONNX model: {e}")
            self.ort_session = None

    def _load_tensorrt(self):
        """Load TensorRT engine (ctypes CUDA, no pycuda)."""
        try:
            import tensorrt as trt
            import ctypes

            self._ctypes = ctypes
            self._cudart = ctypes.CDLL("libcudart.so")

            logger = trt.Logger(trt.Logger.WARNING)
            with open(self.model_path, "rb") as f:
                runtime = trt.Runtime(logger)
                engine = runtime.deserialize_cuda_engine(f.read())

            self.trt_context = engine.create_execution_context()
            self.trt_context.set_input_shape("state_dt", (1, 7))

            self.trt_inputs = []
            self.trt_outputs = []
            for i in range(engine.num_io_tensors):
                name = engine.get_tensor_name(i)
                shape = tuple(self.trt_context.get_tensor_shape(name))
                dtype = trt.nptype(engine.get_tensor_dtype(name))
                host = np.empty(shape, dtype=dtype)
                ptr = ctypes.c_void_p()
                self._cudart.cudaMalloc(ctypes.byref(ptr), ctypes.c_size_t(host.nbytes))
                buf = {"name": name, "host": host, "device": ptr}
                if engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                    self.trt_inputs.append(buf)
                else:
                    self.trt_outputs.append(buf)

            self._trt_engine = engine  # prevent GC
            self.get_logger().info(f"Loaded TensorRT PINN from {self.model_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load TensorRT model: {e}")
            self.trt_context = None

    def _predict(self, state: np.ndarray, dt: float) -> Optional[np.ndarray]:
        """
        Run PINN inference.

        Parameters
        ----------
        state : np.ndarray
            Current state [x, y, z, vx, vy, vz]
        dt : float
            Prediction horizon (seconds)

        Returns
        -------
        np.ndarray or None
            Predicted state [x', y', z', vx', vy', vz']
        """
        if self.norm_stats is None:
            return None

        # Build input: [x, y, z, vx, vy, vz, dt]
        inp = np.concatenate([state, [dt]]).astype(np.float32)

        # Normalize
        inp_norm = (inp - self.norm_stats['input_mean']) / self.norm_stats['input_std']
        inp_norm = inp_norm.reshape(1, -1)

        if self.trt_context is not None:
            # TensorRT inference
            inp_buf = self.trt_inputs[0]
            np.copyto(inp_buf["host"], inp_norm)
            self._cudart.cudaMemcpy(
                inp_buf["device"],
                inp_buf["host"].ctypes.data_as(self._ctypes.c_void_p),
                self._ctypes.c_size_t(inp_buf["host"].nbytes),
                self._ctypes.c_int(1),  # cudaMemcpyHostToDevice
            )
            for buf in self.trt_inputs + self.trt_outputs:
                self.trt_context.set_tensor_address(buf["name"], buf["device"].value)
            self.trt_context.execute_async_v3(stream_handle=0)
            self._cudart.cudaDeviceSynchronize()
            out_buf = self.trt_outputs[0]
            self._cudart.cudaMemcpy(
                out_buf["host"].ctypes.data_as(self._ctypes.c_void_p),
                out_buf["device"],
                self._ctypes.c_size_t(out_buf["host"].nbytes),
                self._ctypes.c_int(2),  # cudaMemcpyDeviceToHost
            )
            out_norm = out_buf["host"][0]
        elif self.ort_session is not None:
            # ONNX inference
            input_name = self.ort_session.get_inputs()[0].name
            result = self.ort_session.run(None, {input_name: inp_norm})
            out_norm = result[0][0]
        elif self.model is not None:
            # PyTorch inference
            import torch
            with torch.no_grad():
                inp_t = torch.from_numpy(inp_norm)
                out_norm = self.model(inp_t).numpy()[0]
        else:
            return None

        # Denormalize
        pred = out_norm * self.norm_stats['target_std'] + self.norm_stats['target_mean']
        return pred

    def on_target_3d(self, msg: PointStamped):
        """Detection received — update last detection time."""
        self.last_detection_time = time.time()

        # If we were in dropout, log recovery
        if self.pinn_active:
            dropout_dur = time.time() - self.dropout_start_time if self.dropout_start_time else 0
            self.get_logger().info(
                f"Detection recovered after {dropout_dur:.2f}s dropout"
            )
            self.pinn_active = False
            self.dropout_start_time = None

    def on_kalman_state(self, msg: Float64MultiArray):
        """Receive Kalman filter state [x, y, z, vx, vy, vz]."""
        if len(msg.data) >= 6:
            self.last_state = np.array(msg.data[:6], dtype=np.float32)
            # Also count as a detection time if state is being published
            if self.last_detection_time is None:
                self.last_detection_time = time.time()

    def prediction_loop(self):
        """Main prediction timer callback."""
        now = time.time()

        # Check if we're in a detection dropout
        if self.last_detection_time is not None:
            time_since_detection = now - self.last_detection_time
        else:
            time_since_detection = float('inf')

        # Determine if PINN should be active
        was_active = self.pinn_active
        if time_since_detection > self.dropout_timeout and self.last_state is not None:
            if not self.pinn_active:
                self.pinn_active = True
                self.dropout_start_time = now
                self.get_logger().warn("Detection dropout — PINN prediction active")
        else:
            self.pinn_active = False

        # Publish active status
        active_msg = Bool()
        active_msg.data = self.pinn_active
        self.active_pub.publish(active_msg)

        # If PINN is active and we have state, predict
        if self.pinn_active and self.last_state is not None:
            # Check if we've exceeded max prediction time
            if self.dropout_start_time is not None:
                dropout_duration = now - self.dropout_start_time
                if dropout_duration > self.max_prediction_time:
                    self.get_logger().warn(
                        f"PINN timeout ({dropout_duration:.1f}s > {self.max_prediction_time}s)"
                    )
                    return

            # Predict
            dt = self.prediction_horizon
            pred = self._predict(self.last_state, dt)

            if pred is not None:
                # Publish predicted position
                pred_msg = PointStamped()
                pred_msg.header.stamp = self.get_clock().now().to_msg()
                pred_msg.header.frame_id = 'camera_color_optical_frame'
                pred_msg.point.x = float(pred[0])
                pred_msg.point.y = float(pred[1])
                pred_msg.point.z = float(pred[2])
                self.pred_pub.publish(pred_msg)

                # Publish full predicted state
                state_msg = Float64MultiArray()
                state_msg.data = [float(v) for v in pred]
                self.state_pub.publish(state_msg)

                # Update last_state for iterative prediction
                self.last_state = pred.astype(np.float32)


def main(args=None):
    rclpy.init(args=args)
    node = PINNPredictionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
