#!/usr/bin/env python3
import roslibpy
import json
import time
import traceback
import numpy as np
import argparse
# from compas.geometry import Frame

# 1) connect to remote rosbridge
def connect_to_ros(host='192.168.0.99', port=9090):
    ros = roslibpy.Ros(host=host, port=port)
    try:
        ros.run()
        print(f"Connected to ROS at {host}:{port}")
        return ros
    except Exception as e:
        print(f"Failed to connect to ROS: {e}")
        exit(1)

class MarkerRecorder:
    def __init__(self, ros, duration, output='marker_data.json'):
        self.ros = ros
        self.duration_limited = False
        self.duration = duration
        self.output = output
        
        self.markers = dict()
        self.start_time = None
        self.end_time = None
        self.is_recording = False
        
        # Subscribe to the /tf topic
        self.tf_topic = roslibpy.Topic(ros, '/marker_poses', 'tf2_msgs/TFMessage')
        self.tf_topic.subscribe(self.tf_callback)

    def tf_callback(self, message):
        """Process incoming TF messages"""
        if not self.is_recording:
            return
            
        current_time = time.time()
        
        # Check if recording duration has elapsed
        if self.duration_limited and current_time - self.start_time > self.duration:
            print(f"Recording complete (duration: {self.duration} seconds)")
            self.stop_recording()
            return
            
        for tf in message['transforms']:
            frame_id = tf['child_frame_id']
            if frame_id.startswith('marker_'):
                # Get marker id from frame_id
                marker_id = frame_id.split('_')[-1]
                
                # Extract translation and rotation            
                pt = tf['transform']['translation']
                q = tf['transform']['rotation']
                
                # Convert to lists for easier processing
                position = [float(pt['x']), float(pt['y']), float(pt['z'])]
                orientation = [float(q['x']), float(q['y']), float(q['z']), float(q['w'])]
                
                # Initialize marker data structure if needed
                if marker_id not in self.markers:
                    self.markers[marker_id] = {
                        "frames": [],
                        "timestamps": [],
                        "optimized": None
                    }
                
                # Append the data
                self.markers[marker_id]["frames"].append({
                    "position": position,
                    "orientation": orientation,
                    "stamp": tf['header']['stamp']
                })
                self.markers[marker_id]["timestamps"].append(current_time)
                
                if self.duration_limited:
                    # Print remaining time for the recording
                    remaining = self.duration - (current_time - self.start_time)
                    print(f"[{current_time:.3f}] Recorded marker {marker_id} - {remaining:.1f}s remaining")
    
    def start_recording(self):
        self.markers = dict()
        self.is_recording = True
        self.start_time = time.time()
        print(f"Recording started for {self.duration} seconds")

    def stop_recording(self):
        self.is_recording = False
        self.end_time = time.time()
        
        self.optimize_marker_positions()

        # Save the markers to file
        if self.save_markers():
            print(f"✅ Successfully saved {len(self.markers)} marker detections to {self.output}")
        else:
            print("❌ Failed to save marker data")

    def save_markers(self):
        """Save marker data to file safely."""
        try:
            with open(self.output, 'w') as f:
                json.dump({
                    "metadata": {
                        "start_time": self.start_time,
                        "end_time": self.end_time,
                        "duration": self.end_time - self.start_time
                    },
                    "markers": self.markers
                }, f, indent=2)
            return True
        except Exception as e:
            print(f"Error saving markers: {e}")
            traceback.print_exc()
            return False

    def optimize_marker_positions(self):
        """Apply statistical optimization to marker positions and orientations."""
        for marker_id, data in self.markers.items():
            if len(data["frames"]) < 2:
                print(f"Too few frames for marker {marker_id}, skipping optimization")
                continue
                
            # Extract all position and orientation components
            positions = np.array([frame["position"] for frame in data["frames"]])
            orientations = np.array([frame["orientation"] for frame in data["frames"]])
            
            # Position optimization (component-wise)
            x_values = positions[:, 0]
            y_values = positions[:, 1]
            z_values = positions[:, 2]
            
            # Orientation optimization (component-wise)
            qx_values = orientations[:, 0]
            qy_values = orientations[:, 1]
            qz_values = orientations[:, 2]
            qw_values = orientations[:, 3]
            
            # Calculate statistics for each component
            position_stats = {
                "x": {
                    "mean": float(np.mean(x_values)),
                    "std": float(np.std(x_values)),
                    "min": float(np.min(x_values)),
                    "max": float(np.max(x_values))
                },
                "y": {
                    "mean": float(np.mean(y_values)),
                    "std": float(np.std(y_values)),
                    "min": float(np.min(y_values)),
                    "max": float(np.max(y_values))
                },
                "z": {
                    "mean": float(np.mean(z_values)),
                    "std": float(np.std(z_values)),
                    "min": float(np.min(z_values)),
                    "max": float(np.max(z_values))
                }
            }
            
            orientation_stats = {
                "x": {
                    "mean": float(np.mean(qx_values)),
                    "std": float(np.std(qx_values)),
                    "min": float(np.min(qx_values)),
                    "max": float(np.max(qx_values))
                },
                "y": {
                    "mean": float(np.mean(qy_values)),
                    "std": float(np.std(qy_values)),
                    "min": float(np.min(qy_values)),
                    "max": float(np.max(qy_values))
                },
                "z": {
                    "mean": float(np.mean(qz_values)),
                    "std": float(np.std(qz_values)),
                    "min": float(np.min(qz_values)),
                    "max": float(np.max(qz_values))
                },
                "w": {
                    "mean": float(np.mean(qw_values)),
                    "std": float(np.std(qw_values)),
                    "min": float(np.min(qw_values)),
                    "max": float(np.max(qw_values))
                }
            }
            
            # Optimized values
            optimized_position = [
                position_stats["x"]["mean"],
                position_stats["y"]["mean"],
                position_stats["z"]["mean"]
            ]
            
            # For quaternions, normalize after averaging to ensure it's valid
            optimized_quat = np.array([
                orientation_stats["x"]["mean"],
                orientation_stats["y"]["mean"],
                orientation_stats["z"]["mean"],
                orientation_stats["w"]["mean"]
            ])
            
            # Normalize quaternion
            quat_norm = np.linalg.norm(optimized_quat)
            if quat_norm > 0:
                optimized_quat = optimized_quat / quat_norm
            
            # Store detailed optimization results
            self.markers[marker_id]["optimized"] = {
                "position": optimized_position,
                "orientation": optimized_quat.tolist(),
                "position_stats": position_stats,
                "orientation_stats": orientation_stats,
                "num_frames": len(data["frames"]),
                "recording_duration": self.end_time - self.start_time
            }
            
            print(f"Marker {marker_id}: Optimized from {len(data['frames'])} frames")
            print(f"  Position: [{optimized_position[0]:.4f}, {optimized_position[1]:.4f}, {optimized_position[2]:.4f}]")
            print(f"  Position StdDev: [{position_stats['x']['std']:.5f}, {position_stats['y']['std']:.5f}, {position_stats['z']['std']:.5f}]")

    def cleanup(self):
        """Clean up resources"""
        try:
            self.tf_topic.unsubscribe()
            print("Unsubscribed from TF topic")
        except:
            pass

    def plot_marker_data(self, marker_id):
        """Plot position data for a specific marker with optimization line."""
        print(self.markers.keys())
        if marker_id not in list(self.markers.keys()):
            print(f"Marker {marker_id} not found in recorded data")
            return
            
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        
        data = self.markers.get(marker_id)
        if not data["frames"]:
            print(f"No frames recorded for marker {marker_id}")
            return
            
        # Extract all position data
        positions = np.array([frame["position"] for frame in data["frames"]])
        timestamps = np.array(data["timestamps"]) - self.start_time
        
        # Create figure with subplots - one for each axis
        fig, axes = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
        fig.suptitle(f"Position Data for ArUco Marker {marker_id}", fontsize=16)
        
        components = ['X', 'Y', 'Z']
        colors = ['r', 'g', 'b']
        
        # Get optimized position if available
        if data.get("optimized"):
            opt_pos = data["optimized"]["position"]
        else:
            # Calculate the mean if no optimization has been done
            opt_pos = [np.mean(positions[:, i]) for i in range(3)]
        
        # Plot each component
        for i, (ax, component, color) in enumerate(zip(axes, components, colors)):
            # Raw data points
            ax.scatter(timestamps, positions[:, i], color=color, alpha=0.5, 
                    label=f'Raw data points ({len(timestamps)} samples)')
            
            # Connect points with a light line
            ax.plot(timestamps, positions[:, i], color=color, alpha=0.3, linewidth=0.5)
            
            # Add horizontal line for optimized value
            ax.axhline(y=opt_pos[i], color='k', linewidth=2, 
                    label=f'Optimized value: {opt_pos[i]:.4f}')
            
            # Add mean and median lines for reference
            mean_val = np.mean(positions[:, i])
            median_val = np.median(positions[:, i])
            ax.axhline(y=mean_val, color='blue', linestyle='--', 
                    label=f'Mean: {mean_val:.4f}')
            ax.axhline(y=median_val, color='green', linestyle=':',
                    label=f'Median: {median_val:.4f}')
            
            # Calculate standard deviation
            std_val = np.std(positions[:, i])
            ax.fill_between(timestamps, mean_val-std_val, mean_val+std_val, 
                        color=color, alpha=0.2, label=f'Std Dev: {std_val:.4f}')
            
            ax.set_ylabel(f'{component} position (m)')
            ax.legend(loc='best')
            ax.grid(True, alpha=0.3)
        
        axes[-1].set_xlabel('Time (seconds)')
        
        # Add a 3D plot of all points
        fig2 = plt.figure(figsize=(10, 8))
        ax3d = fig2.add_subplot(111, projection='3d')
        ax3d.scatter(positions[:, 0], positions[:, 1], positions[:, 2], 
                    c=timestamps, cmap='viridis', s=30, alpha=0.6)
        
        # Plot the optimized position
        ax3d.scatter([opt_pos[0]], [opt_pos[1]], [opt_pos[2]], 
                    color='red', s=100, marker='*', label='Optimized')
        
        # Add connecting lines from each point to the optimized point
        for pos in positions:
            ax3d.plot([pos[0], opt_pos[0]], [pos[1], opt_pos[1]], [pos[2], opt_pos[2]], 
                    'k-', alpha=0.1)
        
        ax3d.set_title(f'3D Position Distribution for Marker {marker_id}')
        ax3d.set_xlabel('X (m)')
        ax3d.set_ylabel('Y (m)')
        ax3d.set_zlabel('Z (m)')
        ax3d.legend()
        
        plt.tight_layout()
        plt.show()
        
        return fig, fig2

def main():
    parser = argparse.ArgumentParser(description='Record and optimize ArUco marker positions')
    parser.add_argument('--host', default='192.168.0.99', help='ROS bridge host')
    parser.add_argument('--port', type=int, default=9090, help='ROS bridge port')
    parser.add_argument('--duration', type=float, default=10.0, help='Recording duration in seconds')
    parser.add_argument('--output', default='optimized_markers.json', help='Output JSON file')
    args = parser.parse_args()
    
    # Connect to ROS
    ros = connect_to_ros(args.host, args.port)
    
    # Create recorder
    recorder = MarkerRecorder(ros, duration=args.duration, output=args.output)
    
    try:
        # Start recording
        recorder.start_recording()
        
        # Wait for recording to complete
        while recorder.is_recording:
            time.sleep(0.1)
        
        print("Recording and optimization complete!")
        
    except KeyboardInterrupt:
        print("\nRecording interrupted!")
        recorder.stop_recording()
    except Exception as e:
        print(f"Error: {e}")
        traceback.print_exc()
    finally:
        # Clean up
        recorder.cleanup()
        recorder.plot_marker_data("0")
        ros.terminate()

if __name__ == "__main__":
    main()
