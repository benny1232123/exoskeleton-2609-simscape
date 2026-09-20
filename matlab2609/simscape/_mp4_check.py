import os, struct, glob
out = []
d = r"C:\Users\29408\Desktop\外骨骼\matlab2609\out_simscape"
try:
    import cv2
    has = True
except Exception as e:
    has = False
    out.append("cv2 unavailable: %s" % e)
if has:
    for p in sorted(glob.glob(os.path.join(d, "exo_real_*.mp4"))):
        cap = cv2.VideoCapture(p)
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)); h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        ok, fr = cap.read()
        mean = float(fr.mean()) if ok else -1.0
        # 取中间一帧看内容是否真的有东西（非全底色）
        cap.set(cv2.CAP_PROP_POS_FRAMES, n//2)
        ok2, fr2 = cap.read()
        mean2 = float(fr2.mean()) if ok2 else -1.0
        std2 = float(fr2.std()) if ok2 else -1.0
        cap.release()
        out.append(f"{os.path.basename(p):24s} frames={n:4d} fps={fps:5.1f} {w}x{h} "
                   f"frame0_mean={mean:6.2f} mid_mean={mean2:6.2f} mid_std={std2:6.2f}")
open(os.path.join(r"C:\Users\29408\Desktop\外骨骼\matlab2609\simscape", "_mp4_check.txt"), "w", encoding="utf-8").write("\n".join(out))
print("\n".join(out))
