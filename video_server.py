#!/usr/bin/env python3
"""
Simple Flask server to handle video uploads and TWIST2 pipeline execution
"""

from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
from datetime import datetime
import os
import subprocess
import tempfile
from pathlib import Path
import uuid

app = Flask(__name__)
CORS(app)  # Enable CORS for browser requests

# Configuration
TWIST2_DIR = Path('/home/user/TWIST2')
VIDEOS_DIR = TWIST2_DIR / 'video_2_motion' / 'videos'
SCRIPT_PATH = TWIST2_DIR / 'video_robot_motion.sh'

# Robotics-sim configuration
ROBOTICS_SIM_VIDEOS_DIR = Path('/home/user/mj_teleop/robotics-sim/videos')
VIDEO_CONVERTER_SCRIPT = Path('/home/user/mj_teleop/robotics-sim/video_converter.py')

# Ensure videos directories exist
VIDEOS_DIR.mkdir(parents=True, exist_ok=True)
ROBOTICS_SIM_VIDEOS_DIR.mkdir(parents=True, exist_ok=True)

@app.route('/health', methods=['GET'])
def health_check():
    """Health check endpoint"""
    return jsonify({'status': 'ok', 'message': 'Video server is running'})

@app.route('/upload_video', methods=['POST'])
def upload_video():
    """
    Upload a video file to robotics-sim videos folder
    Returns the path where the video was saved
    """
    if 'video' not in request.files:
        return jsonify({'error': 'No video file provided'}), 400

    video_file = request.files['video']

    if video_file.filename == '':
        return jsonify({'error': 'No selected file'}), 400

    # Get file extension
    ext = Path(video_file.filename).suffix or '.mp4'

    # Use the original filename if provided, otherwise generate unique filename
    if video_file.filename and video_file.filename != 'blob':
        # Use the provided filename (for webcam recordings with timestamp)
        filename = video_file.filename
    else:
        # Generate unique filename for unnamed blobs
        filename = f'webcam_{uuid.uuid4().hex[:8]}{ext}'

    # Save to robotics-sim videos directory
    filepath = ROBOTICS_SIM_VIDEOS_DIR / filename

    # Save the video
    video_file.save(filepath)
    print(f"Saved webcam recording: {filepath}")

    # Convert WebM to MP4 if needed
    if ext.lower() == '.webm':
        mp4_filename = Path(filename).stem + '.mp4'
        mp4_filepath = ROBOTICS_SIM_VIDEOS_DIR / mp4_filename

        print(f"Converting WebM to MP4: {filepath} -> {mp4_filepath}")
        try:
            # Convert using ffmpeg
            result = subprocess.run(
                ['ffmpeg', '-i', str(filepath), '-c:v', 'libx264', '-preset', 'fast',
                 '-crf', '23', '-c:a', 'aac', '-b:a', '128k', '-y', str(mp4_filepath)],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=60
            )

            if result.returncode == 0 and mp4_filepath.exists():
                print(f"Conversion successful: {mp4_filepath}")
                # Delete the original WebM file
                filepath.unlink()
                print(f"Deleted original WebM: {filepath}")

                # Return MP4 info
                return jsonify({
                    'success': True,
                    'message': 'Video uploaded and converted to MP4 successfully',
                    'filepath': str(mp4_filepath),
                    'filename': mp4_filename,
                    'original_format': 'webm',
                    'converted': True
                })
            else:
                print(f"Conversion failed, keeping WebM file")
                # Keep the WebM file if conversion fails
        except Exception as e:
            print(f"Conversion error: {e}, keeping WebM file")

    return jsonify({
        'success': True,
        'message': 'Video uploaded successfully',
        'filepath': str(filepath),
        'filename': filename
    })

@app.route('/run_twist2_pipeline', methods=['POST'])
def run_twist2_pipeline():
    """
    Run the TWIST2 video_robot_motion.sh script
    Expects JSON: {
        "video_path": "/path/to/video.mp4",
        "robot_type": "unitree_g1",  // optional
        "redis_ip": "localhost"      // optional
    }
    """
    data = request.json

    if not data or 'video_path' not in data:
        return jsonify({'error': 'video_path is required'}), 400

    video_path = data['video_path']
    robot_type = data.get('robot_type', 'unitree_g1')
    redis_ip = data.get('redis_ip', 'localhost')

    # Verify video file exists
    if not Path(video_path).exists():
        return jsonify({'error': f'Video file not found: {video_path}'}), 404

    # Verify script exists
    if not SCRIPT_PATH.exists():
        return jsonify({'error': f'TWIST2 script not found: {SCRIPT_PATH}'}), 404

    try:
        # Run the script in the background
        # Note: This will run asynchronously, might want to implement a task queue for production
        cmd = [
            'bash',
            str(SCRIPT_PATH),
            video_path,
            robot_type,
            redis_ip
        ]

        # Run in background and return immediately
        # For production, consider using Celery or similar for task management
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            cwd=str(TWIST2_DIR)
        )

        return jsonify({
            'message': 'TWIST2 pipeline started',
            'video_path': video_path,
            'robot_type': robot_type,
            'redis_ip': redis_ip,
            'process_id': process.pid
        })

    except Exception as e:
        return jsonify({'error': f'Failed to run pipeline: {str(e)}'}), 500

@app.route('/run_twist2_pipeline_sync', methods=['POST'])
def run_twist2_pipeline_sync():
    """
    Run the TWIST2 pipeline synchronously and return the output
    WARNING: This may take several minutes to complete
    """
    data = request.json

    if not data or 'video_path' not in data:
        return jsonify({'error': 'video_path is required'}), 400

    video_path = data['video_path']
    robot_type = data.get('robot_type', 'unitree_g1')
    redis_ip = data.get('redis_ip', 'localhost')

    # Verify video file exists
    if not Path(video_path).exists():
        return jsonify({'error': f'Video file not found: {video_path}'}), 404

    # Verify script exists
    if not SCRIPT_PATH.exists():
        return jsonify({'error': f'TWIST2 script not found: {SCRIPT_PATH}'}), 404

    try:
        cmd = [
            'bash',
            str(SCRIPT_PATH),
            video_path,
            robot_type,
            redis_ip
        ]

        # Run synchronously and wait for completion
        result = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            cwd=str(TWIST2_DIR),
            timeout=600  # 10 minute timeout
        )

        return jsonify({
            'message': 'TWIST2 pipeline completed',
            'video_path': video_path,
            'robot_type': robot_type,
            'redis_ip': redis_ip,
            'return_code': result.returncode,
            'stdout': result.stdout.decode('utf-8'),
            'stderr': result.stderr.decode('utf-8'),
            'success': result.returncode == 0
        })

    except subprocess.TimeoutExpired:
        return jsonify({'error': 'Pipeline execution timed out (>10 minutes)'}), 500
    except Exception as e:
        return jsonify({'error': f'Failed to run pipeline: {str(e)}'}), 500

@app.route('/upload_and_run', methods=['POST'])
def upload_and_run():
    """
    Combined endpoint: upload video and immediately run TWIST2 pipeline
    Accepts multipart/form-data with:
    - video: video file
    - robot_type: optional robot type (default: unitree_g1)
    - redis_ip: optional redis IP (default: localhost)
    """
    if 'video' not in request.files:
        return jsonify({'error': 'No video file provided'}), 400

    video_file = request.files['video']
    robot_type = request.form.get('robot_type', 'unitree_g1')
    redis_ip = request.form.get('redis_ip', 'localhost')

    if video_file.filename == '':
        return jsonify({'error': 'No selected file'}), 400

    # Get file extension
    ext = Path(video_file.filename).suffix or '.mp4'

    # Generate unique filename
    filename = f'webcam_{uuid.uuid4().hex[:8]}{ext}'
    filepath = VIDEOS_DIR / filename

    # Save the video
    video_file.save(filepath)

    # Run the pipeline
    try:
        cmd = [
            'bash',
            str(SCRIPT_PATH),
            str(filepath),
            robot_type,
            redis_ip
        ]

        # Run in background
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            cwd=str(TWIST2_DIR)
        )

        return jsonify({
            'message': 'Video uploaded and TWIST2 pipeline started',
            'filepath': str(filepath),
            'filename': filename,
            'robot_type': robot_type,
            'redis_ip': redis_ip,
            'process_id': process.pid
        })

    except Exception as e:
        return jsonify({'error': f'Pipeline failed: {str(e)}'}), 500

@app.route('/run_gvhmr_only', methods=['POST'])
def run_gvhmr_only():
    """
    Run only GVHMR pose estimation (Step 1 of pipeline)
    Accepts JSON: {
        "video_path": "/path/to/video.mp4",
        "static_cam": true  // optional, default: true
    }
    Returns the path to the GVHMR output file
    """
    data = request.json

    if not data or 'video_path' not in data:
        return jsonify({'error': 'video_path is required'}), 400

    video_path = data['video_path']
    static_cam = data.get('static_cam', True)

    # Verify video file exists
    if not Path(video_path).exists():
        return jsonify({'error': f'Video file not found: {video_path}'}), 404

    # Path to video_to_motion.py
    video_to_motion_script = TWIST2_DIR / 'video_2_motion' / 'video_to_smpl' / 'video_to_motion.py'

    if not video_to_motion_script.exists():
        return jsonify({'error': f'video_to_motion.py not found at {video_to_motion_script}'}), 404

    try:
        # Build the command
        cmd = [
            'bash', '-c',
            f'source ~/miniconda3/etc/profile.d/conda.sh && '
            f'conda activate gmr && '
            f'python {video_to_motion_script} --video="{video_path}"'
        ]

        if static_cam:
            cmd[-1] += ' -s'

        print(f"Running command: {cmd[-1]}")

        # Run the GVHMR processing
        result = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            cwd=str(TWIST2_DIR),
            timeout=600  # 10 minute timeout
        )

        # Determine output path
        video_name = Path(video_path).stem
        gvhmr_output = TWIST2_DIR / 'video_2_motion' / 'gvhmr_outputs' / video_name / 'hmr4d_results.pt'

        if gvhmr_output.exists():
            return jsonify({
                'message': 'GVHMR processing completed',
                'video_path': video_path,
                'gvhmr_output': str(gvhmr_output),
                'video_name': video_name,
                'success': result.returncode == 0,
                'stdout': result.stdout.decode('utf-8'),
                'stderr': result.stderr.decode('utf-8')
            })
        else:
            return jsonify({
                'error': 'GVHMR processing failed - output file not found',
                'expected_output': str(gvhmr_output),
                'stdout': result.stdout.decode('utf-8'),
                'stderr': result.stderr.decode('utf-8')
            }), 500

    except subprocess.TimeoutExpired:
        return jsonify({'error': 'GVHMR processing timed out (>10 minutes)'}), 500
    except Exception as e:
        return jsonify({'error': f'Failed to run GVHMR: {str(e)}'}), 500

@app.route('/upload_and_run_gvhmr', methods=['POST'])
def upload_and_run_gvhmr():
    """
    Combined endpoint: upload video and run only GVHMR (Step 1)
    Accepts multipart/form-data with:
    - video: video file
    - static_cam: optional boolean (default: true)
    """
    if 'video' not in request.files:
        return jsonify({'error': 'No video file provided'}), 400

    video_file = request.files['video']
    static_cam = request.form.get('static_cam', 'true').lower() == 'true'

    if video_file.filename == '':
        return jsonify({'error': 'No selected file'}), 400

    # Get file extension
    ext = Path(video_file.filename).suffix or '.mp4'

    # Generate unique filename
    filename = f'webcam_{uuid.uuid4().hex[:8]}{ext}'
    filepath = VIDEOS_DIR / filename

    # Save the video
    video_file.save(filepath)

    # Run GVHMR processing
    video_to_motion_script = TWIST2_DIR / 'video_2_motion' / 'video_to_smpl' / 'video_to_motion.py'

    if not video_to_motion_script.exists():
        return jsonify({'error': f'video_to_motion.py not found'}), 404

    try:
        # Build the command
        cmd = [
            'bash', '-c',
            f'source ~/miniconda3/etc/profile.d/conda.sh && '
            f'conda activate gmr && '
            f'python {video_to_motion_script} --video="{filepath}"'
        ]

        if static_cam:
            cmd[-1] += ' -s'

        print(f"Running GVHMR on uploaded video: {filename}")

        # Run in background
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            cwd=str(TWIST2_DIR)
        )

        # Determine expected output path
        video_name = Path(filepath).stem
        gvhmr_output = TWIST2_DIR / 'video_2_motion' / 'gvhmr_outputs' / video_name

        return jsonify({
            'message': 'Video uploaded and GVHMR processing started',
            'filepath': str(filepath),
            'filename': filename,
            'expected_output_dir': str(gvhmr_output),
            'process_id': process.pid,
            'static_cam': static_cam
        })

    except Exception as e:
        return jsonify({'error': f'Failed to start GVHMR: {str(e)}'}), 500

@app.route('/upload_and_convert', methods=['POST'])
def upload_and_convert():
    """
    Upload a video file and convert it to black-and-white

    Accepts multipart/form-data with:
    - video: video file

    Returns JSON with:
    - original_filepath: path to uploaded video
    - converted_filepath: path to converted video
    - original_filename: original filename
    - converted_filename: converted filename
    - original_url: URL to access original video
    - converted_url: URL to access converted video
    """
    if 'video' not in request.files:
        return jsonify({'error': 'No video file provided'}), 400

    video_file = request.files['video']

    if video_file.filename == '':
        return jsonify({'error': 'No selected file'}), 400

    try:
        # Get file extension
        ext = Path(video_file.filename).suffix or '.mp4'

        # Generate unique filename for uploaded video
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'upload_{timestamp}{ext}'
        filepath = ROBOTICS_SIM_VIDEOS_DIR / filename

        # Save the uploaded video
        video_file.save(filepath)
        print(f"Saved uploaded video: {filepath}")

        # Generate output path (adds _bw suffix)
        converted_filename = f'upload_{timestamp}_bw{ext}'
        converted_filepath = ROBOTICS_SIM_VIDEOS_DIR / converted_filename

        # Run video_converter.py
        cmd = [
            'python3',
            str(VIDEO_CONVERTER_SCRIPT),
            str(filepath),
            str(converted_filepath)
        ]

        print(f"Running conversion: {' '.join(cmd)}")
        result = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=300,  # 5 minute timeout
            cwd=str(VIDEO_CONVERTER_SCRIPT.parent)
        )

        if result.returncode != 0:
            error_msg = result.stderr.decode('utf-8')
            return jsonify({
                'error': 'Video conversion failed',
                'details': error_msg
            }), 500

        # Verify converted file exists
        if not converted_filepath.exists():
            return jsonify({
                'error': 'Converted video file not found',
                'expected_path': str(converted_filepath)
            }), 500

        # Success response
        return jsonify({
            'message': 'Video uploaded and converted successfully',
            'original_filepath': str(filepath),
            'converted_filepath': str(converted_filepath),
            'original_filename': filename,
            'converted_filename': converted_filename,
            'original_url': f'/videos/{filename}',
            'converted_url': f'/videos/{converted_filename}',
            'conversion_output': result.stdout.decode('utf-8')
        })

    except subprocess.TimeoutExpired:
        return jsonify({'error': 'Video conversion timed out (>5 minutes)'}), 500
    except Exception as e:
        return jsonify({'error': f'Failed to process video: {str(e)}'}), 500


@app.route('/videos/<filename>', methods=['GET'])
def serve_video(filename):
    """
    Serve video files from the robotics-sim videos directory
    """
    try:
        return send_from_directory(ROBOTICS_SIM_VIDEOS_DIR, filename)
    except Exception as e:
        return jsonify({'error': f'Video not found: {str(e)}'}), 404

if __name__ == '__main__':
    print("="*50)
    print("Video Upload Server for TWIST2")
    print("="*50)
    print(f"TWIST2 Directory: {TWIST2_DIR}")
    print(f"Videos Directory: {VIDEOS_DIR}")
    print(f"Script Path: {SCRIPT_PATH}")
    print("="*50)
    print("Starting server on http://localhost:5000")
    print("="*50)

    app.run(host='0.0.0.0', port=5000, debug=True)
