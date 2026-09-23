# Human Detection — Training Guide (ARK-5)
# ARK-3 jaisa workflow: Raw photos → Label Studio → Zip → Colab GPU → best.pt

Yeh guide bilkul wahi process hai jo tumne ARK-3 mein kiya tha
(flowerpot, paper, breaker, plate) — bas ab classes **standing** aur **fallen** hain,
aur model **YOLOv8** hai (YOLOv5 nahi).

---

## Overview (poora flow)

```
1. Photos lo (ESP32-CAM / phone)
      ↓
2. dataset/human/raw/ mein rakho
      ↓
3. Label Studio → bounding boxes label karo
      ↓
4. Export "YOLO" format
      ↓
5. Train/val folders mein organize karo (script ya manually)
      ↓
6. Zip banao → Google Colab T4 par train
      ↓
7. best.pt → human_detector_robot/models/
      ↓
8. project.yaml update → main_controller.py chalao
```

**Haan — Label Studio se ho jayega.** YOLO export built-in hai.

---

## Step 1 — Photos capture karo

| Class      | Kya shoot karo                                      | Minimum (suggested) |
|------------|-----------------------------------------------------|---------------------|
| `standing` | Khara insan — samne, side, door, qareeb, door       | 80+ photos          |
| `fallen`   | Leta / gira hua — floor par, different angles       | 80+ photos          |

**Tips (ARK-3 se seekha hua):**
- Same room, different lighting
- ESP32-CAM se jo stream aati hai waisi quality use karo (realistic)
- Background vary karo — wall, floor, furniture
- Ek photo mein ek se zyada log ho to har ek ka alag box

Photos yahan rakho (label se pehle):
```
human_detector_robot/dataset/human/raw/
  IMG_001.jpg
  IMG_002.jpg
  ...
```

---

## Step 2 — Label Studio setup

### Install (pehli dafa)
```bash
python -m pip install label-studio
```

### Start karna (Windows)

**Option A — Script (recommended, PATH fix automatic):**
```powershell
cd human_detector_robot
powershell -ExecutionPolicy Bypass -File scripts/start_label_studio.ps1
```
Ya double-click: `scripts/start_label_studio.bat`

**Option B — Manual (agar `label-studio` recognized nahi):**
```powershell
$env:Path += ";C:\Users\Asad Irfan\AppData\Roaming\Python\Python313\Scripts"
label-studio start
```
> Python313 ki jagah tumhara version ho sakta hai — Scripts folder check karo:
> `C:\Users\<YourName>\AppData\Roaming\Python\Python313\Scripts\`

**Option C — Bina PATH ke direct:**
```powershell
python -m label_studio start
```

Browser khulega: **http://localhost:8080**

### Naya project banao
1. **Create Project** → name: `ARK5_Human`
2. **Labeling Setup** → template: **Object Detection with Bounding Boxes**
3. Labels add karo (exact spelling):

```xml
<View>
  <Image name="image" value="$image"/>
  <RectangleLabels name="label" toName="image">
    <Label value="standing" background="#00FF00"/>
    <Label value="fallen" background="#FF0000"/>
  </RectangleLabels>
</View>
```

> **Important:** Label names `standing` aur `fallen` hon — data.yaml se match karein.

### Import images
- **Import** → upload from `dataset/human/raw/` (sari photos)

### Label karna
- Har photo kholo
- Insan ke gird **rectangle box** draw karo
- Class select: `standing` ya `fallen`
- Submit → next image

**Standing box:** tall, narrow — poore body ko cover karo (head se feet tak jitna dikhe)

**Fallen box:** wide, flat — leta hua body

---

## Step 3 — Label Studio se Export (YOLO)

1. Project → **Export**
2. Format: **YOLO** (NOT COCO, NOT VOC)
3. Download zip → e.g. `ARK5_Human_yolo_export.zip`

Export ke andar typically:
```
export/
  images/
    img001.jpg
    img002.jpg
  labels/
    img001.txt
    img002.txt
  classes.txt    (optional — standing, fallen)
```

Har `.txt` file format:
```
<class_id> <x_center> <y_center> <width> <height>
```
Sab values 0 se 1 ke beech normalized.

Class IDs (hamare data.yaml ke mutabiq):
```
0 = standing
1 = fallen
```

---

## Step 4 — Dataset folders mein organize karo

Final structure (training ke liye zaroori):

```
dataset/human/
  data.yaml
  images/train/     ← ~80% photos
  images/val/       ← ~20% photos
  labels/train/     ← matching .txt files
  labels/val/
```

### Option A — Script se (recommended)

Label Studio export zip download ke baad:
```bash
cd human_detector_robot
python scripts/prepare_labelstudio_export.py --zip path/to/ARK5_Human_yolo_export.zip
```
Yeh automatically 80/20 train/val split kar dega.

### Option B — Manually

1. Export zip extract karo
2. 80% images + labels → `images/train/` + `labels/train/`
3. 20% → `images/val/` + `labels/val/`
4. **Same filename** hona chahiye: `photo123.jpg` ↔ `photo123.txt`

---

## Step 5 — Zip banao (Colab ke liye)

`dataset/human/` folder se zip banao — **data.yaml zaroor include karo**:

```
human_dataset.zip
  data.yaml
  images/train/  (jpg + ...)
  images/val/
  labels/train/  (txt + ...)
  labels/val/
```

Windows: `dataset/human` par right-click → Send to → Compressed folder

Ya PowerShell:
```powershell
Compress-Archive -Path "dataset\human\*" -DestinationPath "human_dataset.zip"
```

---

## Step 6 — Google Colab par train (T4 GPU)

1. Google Colab kholo: https://colab.research.google.com
2. **Runtime → Change runtime type → T4 GPU**
3. Notebook upload karo: `scripts/colab/train_human_yolov8.ipynb`
   (ya neeche wale cells copy-paste karo)
4. `human_dataset.zip` upload karo jab notebook kahe
5. Train complete → `best.pt` download karo

### Quick Colab cells (copy-paste)

**Cell 1 — Setup**
```python
!pip install ultralytics -q
from ultralytics import YOLO
import zipfile, os
from google.colab import files

print("Upload human_dataset.zip...")
uploaded = files.upload()
zip_name = list(uploaded.keys())[0]
with zipfile.ZipFile(zip_name, 'r') as z:
    z.extractall('/content/human_dataset')
print("Extracted:", os.listdir('/content/human_dataset'))
```

**Cell 2 — Train**
```python
model = YOLO('yolov8n.pt')  # nano — fast; yolov8s.pt for better accuracy

results = model.train(
    data='/content/human_dataset/data.yaml',
    epochs=100,
    imgsz=640,
    batch=16,
    patience=20,
    project='/content/runs',
    name='human_detector',
    device=0,
)
print("Training done!")
```

**Cell 3 — Download best.pt**
```python
from google.colab import files
files.download('/content/runs/human_detector/weights/best.pt')
```

---

## Step 7 — Model project mein lagao

1. Downloaded `best.pt` copy karo:
   ```
   human_detector_robot/models/best.pt
   ```

2. `config/project.yaml` update:
   ```yaml
   yolo:
     weights: best.pt
     custom_model: true
     conf: 0.4
     iou: 0.45
     imgsz: 640
   ```

3. Test:
   ```bash
   python main_controller.py --no-motors
   ```
   Terminal par `[CV] HUMAN STANDING` ya `[CV] HUMAN FALLEN` aana chahiye.

---

## Local train (Colab na ho to)

Agar laptop par train karna ho (slow without GPU):
```bash
cd human_detector_robot
python scripts/train_human_yolo.py
```
Output: `runs/human_detector/weights/best.pt` → copy to `models/best.pt`

---

## ARK-3 vs ARK-5 — farq kya hai?

| | ARK-3 (Trash) | ARK-5 (Human) |
|---|---------------|---------------|
| Model | YOLOv5 | **YOLOv8** |
| Classes | flowerpot, paper, breaker, plate | **standing, fallen** |
| Label Studio | ✅ Same | ✅ Same |
| Export format | YOLO | YOLO (same) |
| Colab GPU | T4 | T4 (same) |
| .pt location | models/ | `models/best.pt` |
| Config | camera.yaml | `config/project.yaml` |

---

## Common mistakes

| Problem | Fix |
|---------|-----|
| Wrong export format | Label Studio → **YOLO** export, COCO nahi |
| Class names mismatch | Labels exactly `standing` / `fallen` |
| Missing label file | Har `.jpg` ke saath same-name `.txt` |
| Empty txt for no person | Photos bina insan ke mat rakho train mein |
| Low accuracy | Zyada photos, different angles/lighting |
| Colab disconnect | Drive par save karo: `/content/drive/MyDrive/` |

---

## Checklist

- [ ] 80+ standing photos, 80+ fallen photos
- [ ] Label Studio project with standing + fallen labels
- [ ] YOLO export downloaded
- [ ] train/val split (80/20)
- [ ] data.yaml classes match (0=standing, 1=fallen)
- [ ] Colab T4 train → best.pt
- [ ] models/best.pt + project.yaml updated
- [ ] main_controller.py test
