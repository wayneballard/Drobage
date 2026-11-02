import os

import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge


class GarbagePointCloud(Node):

    def __init__(self):
        super().__init__("garbage_point_cloud")

        self.bridge = CvBridge()

        # ---------------------------------------------------------
        # OAK-D camera intrinsics
        # ---------------------------------------------------------

        self.f_x = 457.798
        self.f_y = 456.957
        self.c_x = 313.368
        self.c_y = 224.113

        # OAK-D stereo baseline [m]
        self.B = 0.075

        # ---------------------------------------------------------
        # State
        # ---------------------------------------------------------

        self.depth_frame = None
        self.detection = None

        # Save only once for each detection
        self.cloud_saved = False

        # ---------------------------------------------------------
        # Subscribers
        # ---------------------------------------------------------

        self.sub_depth = self.create_subscription(
            Image,
            "depth_frame",
            self.depth_callback,
            1
        )

        self.sub_detections = self.create_subscription(
            Detection2DArray,
            "detections",
            self.detection_callback,
            1
        )

        self.get_logger().info(
            "Garbage point cloud node started"
        )

    # =============================================================
    # Depth callback
    # =============================================================

    def depth_callback(self, msg):

        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="passthrough"
            )

        except Exception as e:
            self.get_logger().error(
                f"Failed to convert depth image: {e}"
            )
            return

        # Only create the cloud if we have a detection
        # and haven't already saved it.
        if (
            self.detection is not None
            and not self.cloud_saved
        ):
            self.create_point_cloud()

    # =============================================================
    # Detection callback
    # =============================================================

    def detection_callback(self, msg):

        if len(msg.detections) == 0:
            self.detection = None
            self.cloud_saved = False
            return

        # Use the first detection for now
        self.detection = msg.detections[0]

        # New detection -> allow a new point cloud to be saved
        self.cloud_saved = False

        # If depth is already available, create the cloud now
        if self.depth_frame is not None:
            self.create_point_cloud()

    # =============================================================
    # Create point cloud
    # =============================================================

    def create_point_cloud(self):

        if self.depth_frame is None:
            return

        if self.detection is None:
            return

        if self.cloud_saved:
            return

        depth = self.depth_frame

        # ---------------------------------------------------------
        # Get bounding box
        # ---------------------------------------------------------

        bbox = self.detection.bbox

        center_x = bbox.center.position.x
        center_y = bbox.center.position.y

        width = bbox.size_x
        height = bbox.size_y

        x1 = int(center_x - width / 2)
        x2 = int(center_x + width / 2)

        y1 = int(center_y - height / 2)
        y2 = int(center_y + height / 2)

        # ---------------------------------------------------------
        # Clamp bounding box to image
        # ---------------------------------------------------------

        image_height, image_width = depth.shape[:2]

        x1 = max(0, min(x1, image_width - 1))
        x2 = max(0, min(x2, image_width))

        y1 = max(0, min(y1, image_height - 1))
        y2 = max(0, min(y2, image_height))

        if x2 <= x1 or y2 <= y1:
            self.get_logger().warn(
                "Invalid bounding box"
            )
            return

        self.get_logger().info(
            f"Bounding box: x={x1}:{x2}, y={y1}:{y2}"
        )

        # ---------------------------------------------------------
        # Extract disparity inside bounding box
        # ---------------------------------------------------------

        region = depth[y1:y2, x1:x2]

        valid = (
            np.isfinite(region) &
            (region > 0)
        )

        if not np.any(valid):
            self.get_logger().warn(
                "No valid disparity pixels inside bounding box"
            )
            return

        # ---------------------------------------------------------
        # Get pixel coordinates
        # ---------------------------------------------------------

        v, u = np.where(valid)

        # Convert BB-local coordinates
        # to full image coordinates
        u = u + x1
        v = v + y1

        disparity = region[valid].astype(np.float32)

        # ---------------------------------------------------------
        # Disparity -> depth
        # ---------------------------------------------------------

        Z = (self.f_x * self.B) / disparity

        # Same offset as the existing distance calculation
        Z = Z + 0.05

        # ---------------------------------------------------------
        # Remove unreasonable depth values
        # ---------------------------------------------------------

        valid_z = (
            np.isfinite(Z) &
            (Z > 0.05) &
            (Z < 5.0)
        )

        u = u[valid_z].astype(np.float32)
        v = v[valid_z].astype(np.float32)
        Z = Z[valid_z]

        if len(Z) == 0:
            self.get_logger().warn(
                "No valid 3D points after depth filtering"
            )
            return

        # ---------------------------------------------------------
        # Back-project pixels into 3D camera coordinates
        # ---------------------------------------------------------

        X = (u - self.c_x) * Z / self.f_x
        Y = (v - self.c_y) * Z / self.f_y

        points = np.column_stack(
            (X, Y, Z)
        ).astype(np.float32)

        self.get_logger().info(
            f"Extracted {len(points)} 3D points"
        )

        # ---------------------------------------------------------
        # Save PLY
        # ---------------------------------------------------------

        output_path = "/tmp/garbage.ply"

        try:
            self.save_ply(
                points,
                output_path
            )

        except Exception as e:
            self.get_logger().error(
                f"Failed to save PLY: {e}"
            )
            return

        # Mark as saved only after successful completion
        self.cloud_saved = True

        self.get_logger().info(
            f"Saved point cloud: {output_path}"
        )

    # =============================================================
    # Save PLY safely
    # =============================================================

    def save_ply(self, points, path):

        # Write to a temporary file first.
        # This prevents the final PLY from ever being
        # visible in a partially-written state.

        temp_path = path + ".tmp"

        with open(temp_path, "w") as f:

            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(points)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("end_header\n")

            for x, y, z in points:
                f.write(
                    f"{x:.6f} {y:.6f} {z:.6f}\n"
                )

        # Replace the old file only after the entire
        # temporary file has been successfully written.
        os.replace(
            temp_path,
            path
        )


# ================================================================
# Main
# ================================================================

def main(args=None):

    rclpy.init(args=args)

    node = GarbagePointCloud()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
