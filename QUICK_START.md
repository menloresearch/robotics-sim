# Quick Start Guide - Webcam to TWIST2 Pipeline

## What This Does

Record video from your webcam in the browser and automatically run the TWIST2 pipeline to convert it into robot motion.

## Quick Setup (3 steps)

### 1. Start the Backend Server

```bash
cd /home/user/mj_teleop/robotics-sim
./start_video_server.sh
```

Keep this terminal open. You should see:
```
Starting server on http://localhost:5000
```

### 2. Open Your Web Application

Open your browser and load your application (the one with the webcam).

### 3. Run the Pipeline from Browser Console

Open the browser console (F12) and run:

```javascript
// Simple one-liner: Record and process
await webcamRecorder.recordAndProcessWithTWIST2();
```

That's it! The system will:
1. Show a countdown (3, 2, 1, GO!)
2. Record for 10 seconds
3. Upload the video to the server
4. Run the TWIST2 pipeline automatically

## Usage Examples

### Example 1: Quick Record and Process
```javascript
// Record and automatically process with default settings
await webcamRecorder.recordAndProcessWithTWIST2();
```

### Example 2: Process Previously Recorded Video
```javascript
// See what videos you have
const videos = await webcamRecorder.getAllVideos();
console.log(videos);

// Process the latest one
await webcamRecorder.uploadAndRunTWIST2({
    videoSource: videos[videos.length - 1].id
});
```

### Example 3: Custom Robot Settings
```javascript
await webcamRecorder.recordAndProcessWithTWIST2({
    robotType: 'unitree_g1',
    redisIp: '192.168.1.100'  // Your robot's IP
});
```

### Example 4: Step by Step with Control
```javascript
// Step 1: Record
await webcamRecorder.startRecording();
// Wait 10 seconds...

// Step 2: Upload
const upload = await webcamRecorder.uploadToServer();
console.log('Video at:', upload.filepath);

// Step 3: Run pipeline
await webcamRecorder.runTWIST2Pipeline(upload.filepath);
```

## Checking Status

### Check if video is recorded
```javascript
const status = webcamRecorder.getStatus();
console.log('Has recording:', status.hasRecording);
console.log('Recording size:', status.recordingSize);
```

### View stored videos
```javascript
const videos = await webcamRecorder.getAllVideos();
videos.forEach(v => {
    console.log(`ID: ${v.id}, Time: ${v.timestamp}, Size: ${v.size} bytes`);
});
```

### Delete old videos
```javascript
const videos = await webcamRecorder.getAllVideos();
// Delete the first (oldest) video
await webcamRecorder.deleteFromIndexedDB(videos[0].id);
```

## Troubleshooting

### "Failed to upload" error
- Make sure the backend server is running: `./start_video_server.sh`
- Check it's accessible: Open `http://localhost:5000/health` in browser

### "No recording available" error
- Record a video first: `await webcamRecorder.startRecording()`
- Wait 10+ seconds for it to complete

### Pipeline doesn't start
- Check the terminal where `start_video_server.sh` is running
- Look for error messages there
- Verify TWIST2 is installed at `/home/user/TWIST2`

### Video quality issues
The webcam records at the native resolution/framerate. For better quality:
- Ensure good lighting
- Keep movements smooth and visible
- Stay centered in frame

## What Happens Behind the Scenes

```
1. Browser records 10-second video
   ↓
2. Video saved to browser IndexedDB
   ↓
3. Video uploaded to Flask server (localhost:5000)
   ↓
4. Server saves to /home/user/TWIST2/video_2_motion/videos/
   ↓
5. Server runs video_robot_motion.sh
   ↓
6. GVHMR processes video (gmr environment)
   ↓
7. Converts to robot motion (gmr environment)
   ↓
8. Deploys to robot (twist2 environment)
```

## Files Created

After recording and processing, you'll find:

- **Uploaded Video**: `/home/user/TWIST2/video_2_motion/videos/webcam_*.mp4`
- **GVHMR Output**: `/home/user/TWIST2/video_2_motion/gvhmr_outputs/<video_name>/`
- **Robot Motion**: `/home/user/TWIST2/video_2_motion/VIDEO_to_SMPL/<video_name>_unitree_g1.pkl`

## Advanced Options

See `TWIST2_INTEGRATION.md` for:
- Synchronous processing (wait for completion)
- Batch processing multiple videos
- Custom server configurations
- API endpoints documentation
- Detailed troubleshooting

## Tips

1. **Test First**: Run a simple recording first to verify everything works
   ```javascript
   await webcamRecorder.startRecording();
   ```

2. **Monitor Progress**: Watch the terminal where `start_video_server.sh` is running to see pipeline progress

3. **Storage Management**: Browser can store many videos, but clean up old ones:
   ```javascript
   const videos = await webcamRecorder.getAllVideos();
   // Delete videos older than you need
   ```

4. **Network Access**: If your robot is on a different machine, update `redisIp`:
   ```javascript
   await webcamRecorder.recordAndProcessWithTWIST2({
       redisIp: '192.168.1.100'
   });
   ```

## Need Help?

- Check `TWIST2_INTEGRATION.md` for detailed documentation
- Check `VIDEO_STORAGE_USAGE.md` for video storage features
- Look at browser console (F12) for error messages
- Look at server terminal for TWIST2 pipeline errors
