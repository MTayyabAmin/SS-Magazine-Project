# Human Detection Dataset

## Classes
| ID | Name     | Description                    |
|----|----------|--------------------------------|
| 0  | standing | Human khara / upright posture  |
| 1  | fallen   | Human leta / fall posture      |

## Folder Structure
```
dataset/human/
  images/train/   ← training photos yahan
  images/val/     ← validation photos yahan
  labels/train/   ← .txt label files (same name as image)
  labels/val/
  data.yaml
```

## Label Format (YOLO)
Har image ke saath same-name `.txt` file:
```
<class_id> <x_center> <y_center> <width> <height>
```
Sab values 0–1 normalized.

Example — standing person center mein:
```
0 0.5 0.5 0.3 0.7
```

Example — fallen person:
```
1 0.5 0.6 0.8 0.3
```

## Tips
- Standing: zyada tall/narrow boxes
- Fallen: wide/flat boxes
- Different angles, lighting, distance se photos lo
- Train minimum 50+ images per class recommended

## Train
```bash
python scripts/train_human_yolo.py
```
