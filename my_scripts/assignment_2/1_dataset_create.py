import os
import cv2
import numpy as np
import torch
from tqdm import tqdm
import shutil
import random

# Import Ultralytics components
try:
    from ultralytics.models.sam import SAM2VideoPredictor
    from ultralytics.utils.ops import masks2segments
except ImportError:
    print("Ultralytics not installed. Please install it with: pip install ultralytics>=8.2")


class SAM2YOLOGenerator:
    def __init__(self, video_path, output_dir="dataset", sam2_model="sam2_b.pt", target_fps=10):
        """
        Initialize the generator with video path and output directory
        
        Args:
            video_path (str): Path to the input video
            output_dir (str): Directory to store output files
            sam2_model (str): SAM2 model name or path
            target_fps (int): Target frames per second for downsampling
        """
        self.video_path = video_path
        self.output_dir = output_dir
        self.frames_dir = os.path.join(output_dir, "frames")
        self.dataset_dir = os.path.join(output_dir, "dataset")
        self.sam2_model = sam2_model
        self.target_fps = target_fps
        
        # Create directories
        for directory in [self.output_dir, self.frames_dir, self.dataset_dir]:
            os.makedirs(directory, exist_ok=True)
        
        # Setup dataset directories
        for split in ['train', 'val']:
            for subdir in ['images', 'labels']:
                os.makedirs(os.path.join(self.dataset_dir, split, subdir), exist_ok=True)
        
        # Initialize device
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"Using device: {self.device}")
        
        # Class mapping
        self.class_map = {}
        
        # Check video properties
        cap = cv2.VideoCapture(self.video_path)
        self.original_fps = cap.get(cv2.CAP_PROP_FPS)
        self.frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        cap.release()
        
        # Calculate frame sampling rate
        self.frame_interval = max(1, int(self.original_fps / self.target_fps))
        self.estimated_frames = self.frame_count // self.frame_interval
        
        print(f"Video properties: {self.frame_count} frames, {self.original_fps:.1f} FPS, {self.width}x{self.height}")
        print(f"Downsampling to {self.target_fps} FPS (extracting 1 frame every {self.frame_interval} frames)")
        print(f"Estimated frames after downsampling: {self.estimated_frames}")
    
    def extract_first_frame(self):
        """Extract only the first frame from the video for annotation"""
        print(f"Extracting first frame from {self.video_path} for annotation...")
        
        cap = cv2.VideoCapture(self.video_path)
        if not cap.isOpened():
            raise ValueError(f"Could not open video file: {self.video_path}")
        
        ret, frame = cap.read()
        if not ret:
            raise ValueError("Could not read first frame from video")
        
        frame_path = os.path.join(self.frames_dir, "00000.jpg")
        cv2.imwrite(frame_path, frame)
        
        cap.release()
        print(f"First frame saved to {frame_path}")
        return frame, frame_path
    
    def create_downsampled_video(self):
        """Create a downsampled version of the input video at target_fps"""
        print(f"Creating downsampled video at {self.target_fps} FPS...")
        
        # Output path for downsampled video
        downsampled_path = os.path.join(self.output_dir, "downsampled_video.mp4")
        
        # Open input video
        cap = cv2.VideoCapture(self.video_path)
        if not cap.isOpened():
            raise ValueError(f"Could not open video file: {self.video_path}")
        
        # Create video writer
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(downsampled_path, fourcc, self.target_fps, (self.width, self.height))
        
        frame_idx = 0
        saved_count = 0
        
        with tqdm(total=self.estimated_frames, desc="Creating downsampled video") as pbar:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                
                if frame_idx % self.frame_interval == 0:
                    out.write(frame)
                    saved_count += 1
                    pbar.update(1)
                
                frame_idx += 1
        
        cap.release()
        out.release()
        
        print(f"Downsampled video saved to {downsampled_path} ({saved_count} frames)")
        return downsampled_path
    
    def extract_frames(self):
        """Extract frames from the original video at the target FPS"""
        print(f"Extracting frames at {self.target_fps} FPS...")
        
        # Clear frames directory first to avoid mixing new and old frames
        for f in os.listdir(self.frames_dir):
            if f.endswith('.jpg'):
                os.remove(os.path.join(self.frames_dir, f))
        
        cap = cv2.VideoCapture(self.video_path)
        if not cap.isOpened():
            raise ValueError(f"Could not open video file: {self.video_path}")
        
        frame_paths = []
        frame_idx = 0
        output_idx = 0
        
        with tqdm(total=self.estimated_frames, desc="Extracting frames") as pbar:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                
                if frame_idx % self.frame_interval == 0:
                    frame_path = os.path.join(self.frames_dir, f"{output_idx:05d}.jpg")
                    cv2.imwrite(frame_path, frame)
                    frame_paths.append(frame_path)
                    output_idx += 1
                    pbar.update(1)
                
                frame_idx += 1
        
        cap.release()
        print(f"Extracted {len(frame_paths)} frames from the video at {self.target_fps} FPS")
        return frame_paths
    
    def get_multiple_annotations(self):
        """Ask for number of objects, then let user draw bboxes and specify class names for each"""
        # 1. Extract the first frame
        first_frame, _ = self.extract_first_frame()
        
        # Confirm frame was loaded properly
        if first_frame is None or first_frame.size == 0:
            raise ValueError("Failed to load first frame properly")
        
        # Ask for number of objects to annotate
        while True:
            try:
                num_objects = int(input("\nEnter the number of objects you want to label: "))
                if num_objects <= 0:
                    print("Please enter a positive number.")
                    continue
                break
            except ValueError:
                print("Please enter a valid number.")
        
        print(f"\nYou will annotate {num_objects} objects.")
        print("For each object, you'll draw a bounding box and then specify its class name.")
        
        annotations = []
        
        for obj_idx in range(num_objects):
            print(f"\n--- Annotating object {obj_idx + 1}/{num_objects} ---")
            
            # 2. Show frame and get bounding box
            print("A window will open with the first frame.")
            print("Draw a bounding box on the object by clicking and dragging the mouse.")
            print("Press ENTER when done or ESC to cancel and try again.")
            
            # Make a copy of the original frame for this annotation session
            # If we already have annotations, show them on the frame
            display_frame = first_frame.copy()
            
            # Draw existing annotations on the frame
            for ann in annotations:
                x1, y1, x2, y2 = ann["bbox"]
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # Add class name near the box
                cv2.putText(display_frame, ann["class_name"], (x1, y1 - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            
            # Variables for drawing bbox
            bbox = [0, 0, 0, 0]  # x1, y1, x2, y2
            drawing = False
            
            def mouse_callback(event, x, y, flags, param):
                nonlocal bbox, drawing, temp_img
                
                if event == cv2.EVENT_LBUTTONDOWN:
                    drawing = True
                    bbox[0], bbox[1] = x, y
                    # Make a copy of the display frame
                    temp_img[:] = display_frame.copy()
                
                elif event == cv2.EVENT_MOUSEMOVE:
                    if drawing:
                        # Update the image with the current rectangle
                        temp_img[:] = display_frame.copy()
                        cv2.rectangle(temp_img, (bbox[0], bbox[1]), (x, y), (255, 0, 0), 2)
                
                elif event == cv2.EVENT_LBUTTONUP:
                    drawing = False
                    bbox[2], bbox[3] = x, y
                    # Draw the final rectangle
                    temp_img[:] = display_frame.copy()
                    cv2.rectangle(temp_img, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (255, 0, 0), 2)
            
            # Create named window explicitly
            window_name = f"Draw bounding box for object {obj_idx + 1}/{num_objects} - Press ENTER when done, ESC to retry"
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            
            # Ensure image is properly sized for display
            h, w = display_frame.shape[:2]
            display_height = min(800, h)
            display_width = int(w * (display_height / h))
            cv2.resizeWindow(window_name, display_width, display_height)
            
            # Create a copy for drawing
            temp_img = np.zeros_like(display_frame)
            temp_img[:] = display_frame.copy()
            
            # Set mouse callback
            cv2.setMouseCallback(window_name, mouse_callback)
            
            while True:
                # Show the image
                cv2.imshow(window_name, temp_img)
                
                # Force GUI update
                key = cv2.waitKey(100) & 0xFF
                
                # Press ESC to retry
                if key == 27:  # ESC key
                    bbox = [0, 0, 0, 0]
                    temp_img[:] = display_frame.copy()
                    print("Bounding box cleared. Try again.")
                
                # Press ENTER to confirm
                elif key == 13:  # Enter key
                    # Validate the bbox
                    x1, y1, x2, y2 = bbox
                    if (x1 != x2) and (y1 != y2):  # Make sure it's not a single point
                        break
                    else:
                        print("Invalid bounding box. Please draw a proper box.")
            
            cv2.destroyAllWindows()
            
            # Ensure coordinates are in the correct order (x1 < x2, y1 < y2)
            x1, y1, x2, y2 = bbox
            x1, x2 = min(x1, x2), max(x1, x2)
            y1, y2 = min(y1, y2), max(y1, y2)
            
            bbox = [x1, y1, x2, y2]
            print(f"Bounding box: {bbox}")
            
            # 3. After box is drawn, get class name
            class_name = input(f"Enter class name for object {obj_idx + 1}: ")
            
            # Add class to class map if it doesn't exist
            if class_name not in self.class_map:
                self.class_map[class_name] = len(self.class_map)
            
            annotations.append({
                "bbox": bbox,
                "class_name": class_name
            })
            
            print(f"Added annotation for '{class_name}' with bbox {bbox}")
        
        print(f"\nCompleted annotation of {num_objects} objects:")
        for idx, ann in enumerate(annotations):
            print(f"{idx + 1}. Class: {ann['class_name']}, Bbox: {ann['bbox']}")
        
        return annotations
    
    def segment_with_sam2_multi(self, annotations_data, output_video_path=None):
        """Use Ultralytics SAM2 to segment multiple objects in the video at the downsampled rate
        with memory management for long videos, and save results to a video file if path is provided"""
        print("\nRunning SAM2 segmentation on downsampled video for multiple objects...")
        
        # Create downsampled video for SAM2 processing
        downsampled_video = self.create_downsampled_video()
        
        # Create SAM2VideoPredictor with overrides
        overrides = dict(conf=0.25, task="segment", mode="predict", imgsz=1024, model=self.sam2_model)
        predictor = SAM2VideoPredictor(overrides=overrides)
        
        # Extract all bounding boxes
        bboxes = [ann["bbox"] for ann in annotations_data]
        class_names = [ann["class_name"] for ann in annotations_data]
        
        print(f"Processing {len(bboxes)} objects with the following bounding boxes:")
        for i, (bbox, class_name) in enumerate(zip(bboxes, class_names)):
            print(f"{i + 1}. Class: {class_name}, Bbox: {bbox}")
        
        # Helper function to delete old state data to manage memory
        def del_old(predictor, frame_idx, oldest=16):
            # Delete old state data to clear space after '_run_single_frame_inference'
            storage_key, obj_key = "non_cond_frame_outputs", "output_dict_per_obj"
            oldest_allowed_idx = frame_idx - oldest
            all_frame_idxs = predictor.inference_state["output_dict"][storage_key].keys()
            old_frame_idxs = [idx for idx in all_frame_idxs if idx < oldest_allowed_idx]
            for old_idx in old_frame_idxs:
                predictor.inference_state["output_dict"][storage_key].pop(old_idx)
                for objid in predictor.inference_state[obj_key].keys():
                    predictor.inference_state[obj_key][objid][storage_key].pop(old_idx)
        
        # Set up video writer if output path is provided
        video_writer = None
        if output_video_path:
            # Get video properties from first frame to set up the writer
            cap = cv2.VideoCapture(downsampled_video)
            if not cap.isOpened():
                print(f"Error: Could not open downsampled video {downsampled_video}")
                return [], []
            
            # Get video properties
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = cap.get(cv2.CAP_PROP_FPS)
            cap.release()
            
            # Create video writer
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # or 'avc1', 'XVID' depending on your system
            video_writer = cv2.VideoWriter(output_video_path, fourcc, fps, (width, height))
            print(f"Saving segmentation results to: {output_video_path}")
        
        try:
            # Run inference with multiple bounding boxes, using stream=True for memory efficiency
            all_results = []
            for i, result in enumerate(predictor(source=downsampled_video, bboxes=bboxes, stream=True)):
                # Process the result (store or save as needed)
                all_results.append(result)
                
                # Save frame with visualization if video_writer is set
                if video_writer:
                    # Get the plotted frame with bounding boxes and segmentation masks
                    plotted_frame = result.plot()
                    
                    # Add class labels to the visualization
                    for j, (box, cls_name) in enumerate(zip(result.boxes.xyxy, class_names)):
                        if j < len(result.boxes.xyxy):
                            x1, y1 = int(box[0]), int(box[1])
                            # Draw class name with background
                            text_size = cv2.getTextSize(cls_name, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                            cv2.rectangle(plotted_frame, 
                                        (x1, y1 - text_size[1] - 10), 
                                        (x1 + text_size[0] + 10, y1), 
                                        (255, 255, 255), 
                                        -1)
                            cv2.putText(plotted_frame, 
                                    cls_name, 
                                    (x1 + 5, y1 - 5), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 
                                    0.6, 
                                    (0, 0, 0), 
                                    2)
                    
                    # Add frame number
                    cv2.putText(plotted_frame, 
                            f"Frame: {i}", 
                            (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 
                            1, 
                            (255, 255, 255), 
                            2)
                    
                    # Write frame to video
                    video_writer.write(plotted_frame)
                
                # Clear memory after processing each frame
                del_old(predictor, i)
                
                # Optional: Print progress every N frames
                if i % 50 == 0:
                    print(f"Processed {i} frames...")
            
            print(f"Segmentation completed. Generated {len(all_results)} frames of annotations.")
            
            # Release video writer if it was created
            if video_writer:
                video_writer.release()
                print(f"Saved segmentation results to: {output_video_path}")
                
            return all_results, class_names
        
        except Exception as e:
            print(f"Error during SAM2 segmentation: {e}")
            print(f"Try checking if the downsampled video was created properly at: {downsampled_video}")
            
            # Make sure to release the video writer in case of exception
            if video_writer:
                video_writer.release()
                
            return [], []
    
    def results_to_yolo_format_multi(self, results, class_names):
        """Convert SAM2 results to YOLO format annotations for multiple objects"""
        print("\nConverting SAM2 results to YOLO format for multiple objects...")
        
        # Extract frames at the target FPS
        frame_paths = self.extract_frames()
        
        # Make sure we have the right number of frames
        if len(results) != len(frame_paths):
            print(f"Warning: Number of results ({len(results)}) doesn't match number of frames ({len(frame_paths)})")
            print("Will process only the available frames")
        
        # Map class names to indices
        class_indices = [self.class_map[class_name] for class_name in class_names]
        
        annotations = {}
        
        # Process each result
        for i, result in enumerate(tqdm(results, desc="Processing results")):
            if i >= len(frame_paths):
                # Skip if we don't have a corresponding frame
                continue
                
            frame_path = frame_paths[i]
            
            # Get image dimensions from result
            if not hasattr(result, 'orig_img') or result.orig_img is None:
                print(f"Warning: No original image found in result {i}")
                continue
                
            orig_img = result.orig_img
            height, width = orig_img.shape[:2]
            
            # Process all detections for this frame
            frame_annotations = []
            
            # Check if we have masks
            if hasattr(result, 'masks') and result.masks is not None and len(result.masks.data) > 0:
                masks_data = result.masks.data
                # Process each mask (should correspond to each object)
                for obj_idx, mask in enumerate(masks_data):
                    if obj_idx >= len(class_indices):
                        # Skip if we don't have a corresponding class
                        continue
                    
                    # Get class index for this object
                    class_idx = class_indices[obj_idx]
                    
                    try:
                        mask_np = mask.cpu().numpy()
                        
                        # Use OpenCV to find contours in the mask
                        mask_binary = (mask_np > 0).astype(np.uint8)
                        contours, _ = cv2.findContours(mask_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        
                        if contours:
                            # Get the bounding rectangle of the largest contour
                            contour = max(contours, key=cv2.contourArea)
                            x, y, w, h = cv2.boundingRect(contour)
                            
                            # Convert to YOLO format (x_center, y_center, width, height) normalized
                            x_center = (x + w/2) / width
                            y_center = (y + h/2) / height
                            bbox_width = w / width
                            bbox_height = h / height
                            
                            frame_annotations.append({
                                'class_idx': class_idx,
                                'bbox': [x_center, y_center, bbox_width, bbox_height]
                            })
                    except Exception as e:
                        print(f"Error processing mask for object {obj_idx}: {str(e)}")
                        continue
            
            # Alternatively, try to get annotations from boxes
            elif hasattr(result, 'boxes') and result.boxes is not None and len(result.boxes.data) > 0:
                boxes_data = result.boxes.data
                # Process each box (should correspond to each object)
                for obj_idx, box_data in enumerate(boxes_data):
                    if obj_idx >= len(class_indices):
                        # Skip if we don't have a corresponding class
                        continue
                    
                    # Get class index for this object
                    class_idx = class_indices[obj_idx]
                    
                    x1, y1, x2, y2, conf, _ = box_data.cpu().numpy()
                    
                    # Convert to YOLO format (x_center, y_center, width, height) normalized
                    x_center = (x1 + x2) / (2 * width)
                    y_center = (y1 + y2) / (2 * height)
                    bbox_width = (x2 - x1) / width
                    bbox_height = (y2 - y1) / height
                    
                    frame_annotations.append({
                        'class_idx': class_idx,
                        'bbox': [x_center, y_center, bbox_width, bbox_height],
                        'conf': float(conf)
                    })
            
            if frame_annotations:
                annotations[frame_path] = frame_annotations
        
        if not annotations:
            print("Warning: No valid annotations were generated!")
        else:
            print(f"Successfully created annotations for {len(annotations)} frames")
            
        return annotations, frame_paths
    
    def create_yolo_dataset(self, all_annotations, frame_paths, train_ratio=0.8):
        """Create YOLO dataset from annotations and frame paths"""
        print("\nCreating YOLO dataset...")
        
        # Get list of frames with annotations
        annotated_frames = list(all_annotations.keys())
        
        if not annotated_frames:
            print("No valid annotations found. Cannot create dataset.")
            return None
        
        # Shuffle and split frames into train/val
        random.shuffle(annotated_frames)
        train_count = int(len(annotated_frames) * train_ratio)
        train_frames = annotated_frames[:train_count]
        val_frames = annotated_frames[train_count:]
        
        print(f"Creating dataset with {len(train_frames)} training and {len(val_frames)} validation frames")
        
        # Process train frames
        for frame_path in tqdm(train_frames, desc="Processing training data"):
            if frame_path in all_annotations:
                self._save_yolo_data(frame_path, all_annotations[frame_path], "train")
        
        # Process val frames
        for frame_path in tqdm(val_frames, desc="Processing validation data"):
            if frame_path in all_annotations:
                self._save_yolo_data(frame_path, all_annotations[frame_path], "val")
        
        # Create dataset.yaml
        yaml_path = os.path.join(self.output_dir, "dataset.yaml")
        
        with open(yaml_path, "w") as f:
            f.write(f"path: {os.path.abspath(self.dataset_dir)}\n")
            f.write("train: train/images\n")
            f.write("val: val/images\n")
            
            # Write class names
            f.write("names:\n")
            for class_name, idx in sorted(self.class_map.items(), key=lambda x: x[1]):
                f.write(f"  {idx}: {class_name}\n")
        
        # Count created files
        train_images = len(os.listdir(os.path.join(self.dataset_dir, "train", "images")))
        val_images = len(os.listdir(os.path.join(self.dataset_dir, "val", "images")))
        
        print(f"\nDataset Statistics:")
        print(f"Total frames with annotations: {len(annotated_frames)}")
        print(f"Training images: {train_images}")
        print(f"Validation images: {val_images}")
        print(f"Classes: {', '.join(self.class_map.keys())}")
        
        return yaml_path
    
    def _save_yolo_data(self, frame_path, annotations, split):
        """Save image and annotations in YOLO format"""
        # Create image and label paths
        frame_name = os.path.basename(frame_path)
        label_name = os.path.splitext(frame_name)[0] + ".txt"
        
        img_dest_path = os.path.join(self.dataset_dir, split, "images", frame_name)
        label_path = os.path.join(self.dataset_dir, split, "labels", label_name)
        
        # Copy image
        shutil.copy(frame_path, img_dest_path)
        
        # Save annotations
        with open(label_path, "w") as f:
            for ann in annotations:
                # Format: class_idx x_center y_center width height
                bbox_str = " ".join([f"{x:.6f}" for x in ann['bbox']])
                f.write(f"{ann['class_idx']} {bbox_str}\n")
    
    def generate_training_data_multi(self):
        """Main function to generate YOLO training data from video using SAM2 with multiple object support"""
        try:
            # 1. Show first frame and let user annotate multiple objects
            annotations_data = self.get_multiple_annotations()
            
            # 2. Segment the video with SAM2 for multiple objects
            results, class_names = self.segment_with_sam2_multi(annotations_data)
            
            if not results:
                print("SAM2 segmentation failed. Please try again.")
                return None
            
            # 3. Convert results to YOLO format for multiple objects
            annotations, frame_paths = self.results_to_yolo_format_multi(results, class_names)
            
            if not annotations:
                print("No valid annotations were created. Please try again.")
                return None
            
            # 4. Create YOLO dataset
            yaml_path = self.create_yolo_dataset(annotations, frame_paths)
            
            if yaml_path:
                print("\nTraining data generation completed!")
                print(f"Dataset created at: {self.dataset_dir}")
                print(f"Configuration file: {yaml_path}")
                return yaml_path
            else:
                print("\nFailed to create dataset.")
                return None
        
        except Exception as e:
            print(f"Error in data generation: {e}")
            import traceback
            traceback.print_exc()
            return None


# Main function - just edit the video path here
def main():
    # Define your parameters directly here - modify these values as needed
    video_path = "/root/catkin_ws/src/my_scripts/assignment_2/blocks_train.mp4"  # <-- Change this to your video file path
    output_dir = "/root/catkin_ws/src/my_scripts/assignment_2/train_yolo/dataset_blocks_statics/"
    sam2_model = "sam2.1_b.pt"  # Using the base model
    target_fps = 7  # Downsample video to 5 FPS
    
    # Create the data generator
    generator = SAM2YOLOGenerator(
        video_path=video_path,
        output_dir=output_dir,
        sam2_model=sam2_model,
        target_fps=target_fps
    )
    
    # Generate training data with multi-object support
    generator.generate_training_data_multi()


if __name__ == "__main__":
    main()