import cv2
import sys
import os
from pathlib import Path

def convert_to_bw(input_path, output_path=None):
    """
    Convert video to black-and-white

    Args:
        input_path (str): Path to input video
        output_path (str, optional): Path for output video.
                                     If None, generates from input path

    Returns:
        str: Path to output video
    """
    # Validate input
    if not os.path.exists(input_path):
        raise FileNotFoundError(f"Input video not found: {input_path}")

    # Generate output path if not provided
    if output_path is None:
        input_stem = Path(input_path).stem
        input_dir = Path(input_path).parent
        output_path = str(input_dir / f"{input_stem}_bw.mp4")

    # Video processing logic
    cap = cv2.VideoCapture(input_path)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open {input_path}")

    fps = cap.get(cv2.CAP_PROP_FPS) or 30
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # common MP4 codec
    writer = cv2.VideoWriter(output_path, fourcc, fps, (w, h), isColor=True)

    frame_count = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        bw = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)  # keep 3 channels for MP4 writer
        writer.write(bw)
        frame_count += 1

    cap.release()
    writer.release()

    print(f"Converted {frame_count} frames")
    print(f"Saved: {output_path}")

    return output_path

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python video_converter.py <input_path> [output_path]")
        sys.exit(1)

    input_path = sys.argv[1]
    output_path = sys.argv[2] if len(sys.argv) > 2 else None

    try:
        result = convert_to_bw(input_path, output_path)
        sys.exit(0)
    except Exception as e:
        print(f"Error: {str(e)}", file=sys.stderr)
        sys.exit(1)
