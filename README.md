# eye-swipe

Control your desktop with eye gestures using computer vision.

## Features
- Real-time eye tracking with MediaPipe
- Wink detection for left/right navigation
- Configurable blink threshold and swipe delay
- Visual feedback overlay

## Requirements
```
pip install opencv-python mediapipe numpy pyautogui
```

## Usage
```
python main.py
```

- **Left eye wink**: Swipe left
- **Right eye wink**: Swipe right
- **Both eyes closed**: Pause
- **Press 'q'**: Quit

## Configuration
Adjust these variables in the code:
- `BLINK_THRESHOLD`: Eye closure detection sensitivity (default: 12)
- `SWIPE_DELAY`: Cooldown between swipes in seconds (default: 1.0)
- `SWIPE_AMOUNT`: Swipe magnitude (default: 50)

## How It Works
Uses MediaPipe Face Mesh to track 468 facial landmarks, calculating the distance between upper and lower eyelids to detect winks and trigger pyautogui keyboard shortcuts.