"""
Block detector v2: floor-color subtraction + per-blob color naming. No training.

  python3 detect_blocks_v2.py image.png     # image
  python3 detect_blocks_v2.py 0             # webcam
  python3 detect_blocks_v2.py video.mp4     # video

Keys: q quit | m toggle foreground-mask view | s save out.png
"""
import sys
import cv2
import numpy as np

# ---------------- tunables ----------------
CHROMA_T = 22          # how "colored" (vs floor) a pixel must be to count as foreground
BRIGHT_T = 35          # how much brighter than the floor counts as foreground (white tops)
SAT_MIN = 45           # min saturation to count a pixel as "has a color"
YELLOW_H = (15, 40)    # OpenCV hue ranges (0-179)
GREEN_H = (41, 95)
MIN_AREA_FRAC = 0.002  # smallest blob, as fraction of image area
MAX_AREA_FRAC = 0.30   # largest blob (kills background regions)
MERGE_FRAC = 0.02      # closing size (fraction of short side): joins a block's top+base
DETECT_WHITE = True
# ------------------------------------------

BGR = {"yellow": (0, 220, 255), "green": (0, 200, 0), "white": (200, 200, 200)}


def foreground_mask(frame):
    """Everything that differs from the floor: colored OR brighter than the floor.
    Shadows (same color, darker) are ignored."""
    lab = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2LAB).astype(np.float32)
    bg = np.median(lab.reshape(-1, 3), axis=0)          # floor = most common color
    d_light = lab[..., 0] - bg[0]
    d_chroma = np.hypot(lab[..., 1] - bg[1], lab[..., 2] - bg[2])
    fg = ((d_chroma > CHROMA_T) | (d_light > BRIGHT_T)).astype(np.uint8) * 255
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, k)        # remove grid lines / specks
    m = max(7, int(MERGE_FRAC * min(frame.shape[:2])))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (m, m))
    return cv2.morphologyEx(fg, cv2.MORPH_CLOSE, k)     # join top + base of each block


def color_pixels(hsv, comp, name):
    lo, hi = YELLOW_H if name == "yellow" else GREEN_H
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    return (comp > 0) & (s > SAT_MIN) & (v > 40) & (h >= lo) & (h <= hi)


def make_result(name, cnt):
    x, y, w, h = cv2.boundingRect(cnt)
    M = cv2.moments(cnt)
    cx, cy = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])) if M["m00"] else (x + w // 2, y + h // 2)
    return {"color": name, "bbox": (x, y, w, h), "center": (cx, cy), "area": cv2.contourArea(cnt)}


def detect(frame):
    """Foreground blobs -> color-seeded watershed -> one box per block."""
    H, W = frame.shape[:2]
    min_a, max_a = MIN_AREA_FRAC * H * W, MAX_AREA_FRAC * H * W
    blur = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    fg = foreground_mask(frame)

    # confident color seeds
    seeds = {
        "yellow": (s > 80) & (v > 40) & (h >= YELLOW_H[0]) & (h <= YELLOW_H[1]),
        "green":  (s > 60) & (v > 40) & (h >= GREEN_H[0]) & (h <= GREEN_H[1]),
        "white":  (s < 25) & (v > 190),
    }
    m = max(3, int(MERGE_FRAC * min(H, W)))
    kmerge = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (m, m))
    k3 = np.ones((3, 3), np.uint8)

    markers = np.zeros((H, W), np.int32)
    far_bg = cv2.dilate(fg, np.ones((5, 5), np.uint8)) == 0
    markers[far_bg] = 1
    names, nid = {}, 2
    for name, sd in seeds.items():
        if name == "white" and not DETECT_WHITE:
            continue
        mk = (sd & (fg > 0)).astype(np.uint8) * 255
        mk = cv2.morphologyEx(mk, cv2.MORPH_OPEN, k3)
        mk = cv2.morphologyEx(mk, cv2.MORPH_CLOSE, kmerge)   # join a block's top + base
        mk = mk & fg
        n, lab, stats, _ = cv2.connectedComponentsWithStats(mk)
        for i in range(1, n):
            if stats[i, cv2.CC_STAT_AREA] < 0.15 * min_a:
                continue
            markers[(lab == i) & (markers == 0)] = nid
            names[nid] = name
            nid += 1
    if not names:
        return [], fg

    ws = cv2.watershed(blur, markers)
    results = []
    for i, name in names.items():
        region = ((ws == i) & (fg > 0)).astype(np.uint8)
        cnts, _ = cv2.findContours(region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            continue
        c = max(cnts, key=cv2.contourArea)
        if min_a <= cv2.contourArea(c) <= max_a:
            results.append(make_result(name, c))
    return results, fg


def draw(frame, results):
    out = frame.copy()
    for r in results:
        x, y, w, h = r["bbox"]
        col = BGR[r["color"]]
        cv2.rectangle(out, (x, y), (x + w, y + h), col, 2)
        cv2.circle(out, r["center"], 4, (0, 0, 255), -1)
        cv2.putText(out, r["color"], (x, max(y - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
        cv2.putText(out, r["color"], (x, max(y - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 1)
    cv2.putText(out, f"blocks: {len(results)}", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    return out


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else "0"
    if src.lower().endswith((".png", ".jpg", ".jpeg", ".bmp")):
        frame = cv2.imread(src)
        if frame is None:
            sys.exit(f"Could not read {src}")
        res, fg = detect(frame)
        for r in res:
            print(r["color"], r["bbox"], r["center"])
        out = draw(frame, res)
        cv2.imwrite("out.png", out)
        cv2.imwrite("mask.png", fg)
        cv2.imshow("blocks", out)
        cv2.imshow("foreground mask", fg)
        cv2.waitKey(0)
        return

    cap = cv2.VideoCapture(int(src) if src.isdigit() else src)
    show_mask = False
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        res, fg = detect(frame)
        cv2.imshow("blocks", fg if show_mask else draw(frame, res))
        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"):
            break
        if k == ord("m"):
            show_mask = not show_mask
        if k == ord("s"):
            cv2.imwrite("out.png", draw(frame, res))
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
